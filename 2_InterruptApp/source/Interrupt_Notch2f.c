/*************************************************************************************************
 * Interrupt_Notch2f.c — 二倍工频限波器（陷波器）实现
 *
 * 核心思路：
 *   1. 构造一个只对 2 倍工频（中国 50Hz 工频→100Hz 纹波）敏感的"带通滤波器"
 *   2. 带通滤波器输出 ≈ 纹波本身
 *   3. 原始信号 − 纹波 = 剩下的直流分量（陷波效果）
 *
 * 采样节拍：目标 100us 走一拍陷波运算
 *   - 充电阶段 PWM 周期 TBPRD=1200，累加 5 次 = 6000
 *   - 放电阶段 PWM 周期 TBPRD=1000，累加 6 次 = 6000
 *   - 两种情况都凑到 6000（即 100us）才执行一次
 *
 * 参数更新：
 *   - 系数计算（含 sin/除法）放在 5ms 后台任务里完成
 *   - 中断内只做乘加运算（省时间）
 *   - 双 bank（乒乓缓冲）切换，改参数不打断中断
 *************************************************************************************************/
#include "Includes.h"

/* 把中断函数定位到 RAM 段执行，避免 Flash 等待周期，提高中断实时性 */
#pragma CODE_SECTION(Notch2f_Isr, ".TI.ramfunc");

/* 全局陷波器参数结构体（含两套 bank、当前使用标记、参数值等） */
ST_NOTCH2F_PARAM g_Notch2f;

/* 单通道运行时状态：
 *   fBand ≈ 与输入同相的纹波分量
 *   fQuad ≈ 与输入正交（90°相差）的纹波分量
 *   uSeeded：是否已完成上电首拍初始化（0=未做过）
 * 说明：各通道独立变量，不用数组。C28x 对数组变址寻址开销大。 */
typedef struct
{
    float32 fBand;   /* 同相分量 */
    float32 fQuad;   /* 正交分量 */
    Uint16  uSeeded; /* 首次初始化标记 */
} Notch2fChRt_t;

/* 单路通道运行时状态：继电器 VM 母线（控制主反馈，唯一陷波通道）。
 * 桥臂侧 BusVolt1/2 只用于功率；PCS 端改用 100ms 低通，均不再陷波。 */
static Notch2fChRt_t s_chVmBus;

static float32 s_f32K;       /* 中断里上一拍用的 k 值，用来检测 k 是否变化 */

/* ====================== 后台：计算系数 ======================
 * 后台任务里允许用 sin / 除法这些重运算
 * h 用 sin 精确计算才能准确对准 100Hz，不能图省事用 tan 近似 */
static void Notch2f_CoeffWrite(volatile ST_NOTCH2F_COEFF *p, float32 fLineHz, float32 fK)
{
    float32 fH;
    float32 fD;
    float32 fInvDen;

    /* h = 2*sin(2π*f*Ts)，入参 fLineHz 是工频，内部陷波的是 2 倍工频 */
    fH = 2.0f * sinf(6.28318530718f * fLineHz * NOTCH2F_TS_S);
    fD = fK * fH;                      /* 中间量 d = k*h */
    fInvDen = __divf32(1.0f, 1.0f + 0.5f * fD); /* 分母的倒数，后面都用乘法 */

    p->f32K = fK;                           /* 存 k，中断里要用来算 kDelta */
    p->f32H = fH;                           /* 存 h，中断里更新 Quad 要用 */
    p->f32A = (1.0f - 0.5f * fD) * fInvDen; /* 状态更新系数 A */
    p->f32B = fH * fInvDen;                 /* 状态更新系数 B */
    p->f32C = fD * fInvDen;                 /* 状态更新系数 C */
}

/* ====================== 写"闲置"bank，再翻牌 ====================== */
static void Notch2f_RebuildBanks(float32 fLineHz, float32 fK)
{
    Uint16 u16Next;

    /* 当前中断在用 u16Pub 这套，我们写另一套（异或 1 翻转） */
    u16Next = g_Notch2f.u16Pub ^ 1U;
    if (u16Next == 0U)
    {
        Notch2f_CoeffWrite(&g_Notch2f.stBank0, fLineHz, fK);
    }
    else
    {
        Notch2f_CoeffWrite(&g_Notch2f.stBank1, fLineHz, fK);
    }

    /* 记录当前"已经生效"的参数值，下次比较用 */
    g_Notch2f.f32LineHzApplied = fLineHz;
    g_Notch2f.f32KApplied = fK;

    /* 最后一步才切换 u16Pub：保证中断读到的要么是旧的整套、要么是新的整套，
     * 不会读到"半新半旧"的混合系数 */
    g_Notch2f.u16Pub = u16Next;
}

/* ====================== 初始化 ====================== */
void Notch2f_Init(void)
{
    /* 全局结构体和通道状态清零 */
    memset((void *)&g_Notch2f, 0, sizeof(g_Notch2f));
    memset((void *)&s_chVmBus, 0, sizeof(s_chVmBus));
    s_f32K = 0.0f;

    /* 填入默认工频和默认 k */
    g_Notch2f.f32LineHz = NOTCH2F_LINE_HZ_DEF;
    g_Notch2f.f32K = NOTCH2F_K_DEFAULT;

    /* f32LineHzApplied / f32KApplied 还是 memset 的 0，和默认值不等，
     * 所以下面调用 ParamUpdate5ms 一定会算出首套系数 */
    g_Notch2f.u16Pub = 0U;
    Notch2f_ParamUpdate5ms();
}

/* ====================== 5ms 后台任务：参数更新 ====================== */
void Notch2f_ParamUpdate5ms(void)
{
    float32 fLineHz;
    float32 fK;

    /* 读取工频设定值，做范围保护（40~70Hz 之外视为非法） */
    fLineHz = g_Notch2f.f32LineHz;
    if ((fLineHz < 40.0f) || (fLineHz > 70.0f))
    {
        fLineHz = NOTCH2F_LINE_HZ_DEF; /* 非法就退回默认，防止算出发散的系数 */
    }

    /* 读取 k 值，范围 (0, 2) 之外视为非法 */
    fK = g_Notch2f.f32K;
    if ((fK <= 0.0f) || (fK >= 2.0f))
    {
        fK = NOTCH2F_K_DEFAULT; /* k≤0 会振荡；太大也不稳 */
    }

    /* 参数没变化就不翻 bank，省掉一次系数计算 */
    if ((fLineHz == g_Notch2f.f32LineHzApplied) && (fK == g_Notch2f.f32KApplied))
    {
        return;
    }

    /* 参数有变就重算闲置 bank 再翻牌 */
    Notch2f_RebuildBanks(fLineHz, fK);
}

/* ====================== 单通道一拍运算（强制内联） ======================
 * 输出用新旧 Band 平均：补偿半拍相位误差，陷波深度才能做深。 */
#pragma FUNC_ALWAYS_INLINE(Notch2f_Step);
static inline void Notch2f_Step(Notch2fChRt_t *pCh, float32 fIn, float32 volatile *pOut,
                               float32 fA, float32 fB, float32 fC, float32 fH,
                               float32 fK, float32 fKDelta)
{
    float32 fBandOld;
    float32 fBandNew;

    if (pCh->uSeeded == 0U)
    {
        /* 上电第一拍：按稳态关系把 Quad 预置为 k*输入，缩短收敛过程、少抖 */
        pCh->fBand = 0.0f;
        pCh->fQuad = fK * fIn;
        pCh->uSeeded = 1U;
    }
    else if (fKDelta != 0.0f)
    {
        /* 在线修改了 k：Quad 跟着平移一点，输出不会甩出大尾巴（扰动） */
        pCh->fQuad += fKDelta * fIn;
    }

    /* 保存旧 Band，用于输出平均 */
    fBandOld = pCh->fBand;

    /* 状态更新：标准带通谐振结构
     *   BandNew = A*BandOld - B*Quad + C*In */
    fBandNew = fA * fBandOld - fB * pCh->fQuad + fC * fIn;
    pCh->fBand = fBandNew;

    /* 更新 Quad：必须用新 Band，不然谐振中心会偏 */
    pCh->fQuad += fH * fBandNew;

    /* 输出 = 原值 − 新旧 Band 的平均（≈ 原值 − 瞬时纹波） */
    *pOut = fIn - 0.5f * (fBandOld + fBandNew);
}

/* 由EPWM10_ISR统一按10kHz错峰调用；这里仅执行一次陷波（继电器 VM 母线）。 */
void Notch2f_Isr(void)
{
    volatile ST_NOTCH2F_COEFF *pC;
    float32 fK;
    float32 fA;
    float32 fB;
    float32 fC;
    float32 fH;
    float32 fKDelta;

    /* 读出当前"发布中"的那套系数 bank */
    pC = (g_Notch2f.u16Pub == 0U) ? (&g_Notch2f.stBank0) : (&g_Notch2f.stBank1);
    fK = pC->f32K;
    fA = pC->f32A;
    fB = pC->f32B;
    fC = pC->f32C;
    fH = pC->f32H;

    /* 计算 k 的变化量：如果后台刚改过 k，Step 里要补偿 Quad */
    fKDelta = fK - s_f32K;
    s_f32K = fK;

    /* 只陷波继电器 VM 这一路（电压环反馈） */
    Notch2f_Step(&s_chVmBus, stADC.VmBusVolt.f32IsrResult, &stADC.VmBusVolt.f32Notch,
                 fA, fB, fC, fH, fK, fKDelta);
}
