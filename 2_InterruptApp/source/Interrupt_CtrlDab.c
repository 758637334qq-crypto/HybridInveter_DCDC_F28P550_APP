/*************************************************************************************************
 * Interrupt_CtrlDab.c — DAB 实时控制（EPWM10_ISR：充电 50kHz / 放电 60kHz）
 *
 * 模块划分（自下而上）：
 *   ① 外环     Dab_Ctrl_BusVoltLoop                 电压环 → 总电流给定 Bus.f32Ref
 *   ② 放电内环 Dab_Ctrl_DisFfCalc / DisGainSched / DisCurrLoop / DisDutyMap
 *   ③ 充电内环 Dab_Ctrl_SrStateUpdate / ChgPiStep / ChgCurrLoop
 *   ④ 顶层     Dab_Ctrl_Loop                        工况快照 → 打嗝/封波 → 内外环
 *   ⑤ 发波     Dab_Pwm_ChgUpdate / Dab_Pwm_DisUpdate 占空比 → CMP / AQ
 *   ⑥ 中断     EPWM10_ISR                             采样 → 保护 → 控制 → 发波 → 应答
 *
 * 约束：
 *   - 热路径全部 static inline，无函数调用开销；
 *   - 可变状态只有 g_stDabIsrState（本文件）与任务侧发布的 Bus / stDabPi0..1 /
 *     stDabDuty0..1 / stDabParam，本文件不新增全局变量；
 *   - 整定宏集中在 Interrupt_CtrlDab.h，本文件只做实现。
 *************************************************************************************************/
#include "Includes.h"
#include "App_DabModel.h"

/* 固定发波占空比必须 < 0.5：CMP = 2·d·TBPRD，d ≥ 0.5 时比较沿越过周期沿，
 * 桥臂恒通并与互补臂重叠直通。浮点常量不能 #if，这里用整数源做
 * “数组长度为负”的编译期断言，越界直接编译失败。 */
typedef char DabChgDtopBelowHalf_t[(DAB_CHG_D_TOP_MILLI < 500U) ? 1 : -1];
typedef char DabChgDlowMaxBelowHalf_t[(DAB_CHG_D_LOW_MAX_MILLI < 500U) ? 1 : -1];
/* 变量脉冲必须落在固定窗口内（前沿对齐，后沿截断）：D_LOW_MAX < D_TOP，
 * 否则 S4/S2 脉宽越过 S1/S3，伏秒由固定侧决定、调功失效。 */
typedef char DabChgDlowBelowDtop_t[(DAB_CHG_D_LOW_MAX_MILLI < DAB_CHG_D_TOP_MILLI) ? 1 : -1];
/* Buck 迟滞门限必须 0 < OFF < ON，否则零能力也会保持“就绪”。 */
typedef char DabChgCapOnAboveOff_t[
    ((DAB_CHARGE_CAP_OFF_MILLI > 0U) &&
     (DAB_CHARGE_CAP_OFF_MILLI < DAB_CHARGE_CAP_ON_MILLI)) ? 1 : -1];
/* 放电内环 Δu 上轨 stepMax≥STEP_MIN，必须 >0，否则 DAB_SAT(Δu,−STEP_DOWN,stepMax) 上下轨倒置。 */
typedef char DabDutyStepMinPositive_t[(DAB_DUTY_STEP_MIN_MILLI > 0U) ? 1 : -1];

/*==============================================================================
 * 全局可变状态：全文件唯一一块，字段说明见 Interrupt_CtrlDab.h
 *============================================================================*/
volatile ST_DAB_ISR_STATE g_stDabIsrState =
{
    0.0f,                   /* f32InvBatVolt  : 1/Vbat，0 = 不可用 */
    0.0f,                   /* ▲ f32DisInvE   : 1/(n·Vbat)，0 = 欠压关前馈 */
    1.0f,                   /* ▲ f32DisRatio  : 电压比 Vset/(n·Vbat) */
    {DAB_PWM_STATE_RESET},  /* stChgAqBr1     : 桥 1 动作值 / 相位 */
    {DAB_PWM_STATE_RESET},  /* stChgAqBr2     : 桥 2 动作值 / 相位 */
    0U,                     /* u16ChgBuckOn   : 压差足够，允许发波 */
    0U,                     /* u16BurstOff    : 充放电轻载休眠 */
    0U,                     /* u16ChgPwmActive: 充电 PWM 在发波 */
    0.0f,                   /* f32ChgLimitA   : 整机有效充电限流 */
    0.0f                    /* f32CapFilt     : 充电能力缓放值 */
};

/* 封波后动作值缓存作废：下拍两组动作必写、常量必写 */
#pragma FUNC_ALWAYS_INLINE(Dab_Pwm_AqCacheReset);
static inline void Dab_Pwm_AqCacheReset(void)
{
    g_stDabIsrState.stChgAqBr1.all = DAB_PWM_STATE_RESET;
    g_stDabIsrState.stChgAqBr2.all = DAB_PWM_STATE_RESET;
}

/*==============================================================================
 * 任务侧接口
 *============================================================================*/

/* BatCurr = (ibus × vbus) / Vbat：1/Vbat 在 1ms 任务用 10ms 滤波值预算，ISR 只乘。
 * 欠压 / 断线写 0，避免噪声放大误触发。 */
void Dab_Ctrl_InvBatVoltUpdate(void)
{
    float f32VbatTmp = stADC.BatVolt.Filt.f32Filt10ms;

    g_stDabIsrState.f32InvBatVolt = (f32VbatTmp >= DAB_BAT_VOLT_MIN) ? __divf32(1.0f, f32VbatTmp) : 0.0f;
}

/* ▲ 放电前馈慢变量预算（100us 执行）：算出 1/(n·Vbat) 与电压比 r = Vset/(n·Vbat)。
 * 电池欠压时 f32DisInvE 写 0 关前馈；两变量都是慢变 32 位浮点，无需关中断。 */
void Dab_Ctrl_DisFfUpdate100us(void)
{
    float f32VbatTmp = stADC.BatVolt.Filt.f32Filt10ms;

    if (f32VbatTmp >= DAB_BAT_VOLT_MIN)
    {
        float f32InvE = __divf32(1.0f, DAB_TURNS_RATIO * f32VbatTmp);

        g_stDabIsrState.f32DisInvE  = f32InvE;
        g_stDabIsrState.f32DisRatio = __fmax(Bus.f32Vref * f32InvE, 0.05f);
    }
    else
    {
        g_stDabIsrState.f32DisInvE = 0.0f;
    }
}

/* 起机会话复位：充/放电锁存虽会在相反工况每拍清零，但同工况停机后再起机
 * 不跑 Dab_Ctrl_Loop，锁存仍可能跨会话残留。调用时 PWM 必须已封波。 */
void Dab_Ctrl_SessionReset(void)
{
    g_stDabIsrState.u16ChgBuckOn    = 0U;  /* 充电 Buck 从未发波态起步 */
    g_stDabIsrState.f32ChgLimitA    = 0.0f;
    g_stDabIsrState.f32CapFilt      = 0.0f;
    g_stDabIsrState.u16BurstOff     = 0U;  /* 充放电从非休眠态起步 */
    Dab_Pwm_AqCacheReset();                /* 动作值作废，下拍按新工况重写 */
}

/*==============================================================================
 * ① 电压外环（两路共用）
 *
 * 【核心背景约定】
 *   1. 符号方向：电流 > 0 代表放电（母线往外送电）；电流 < 0 代表充电（母线往里吸电）。
 *   2. 多速率架构：外层函数在每个 PWM 载波周期（充电 50kHz / 放电 60kHz）都会执行一次；
 *                 但核心 PI 算法做了分频，固定在 10kHz（每 100µs，即 5~6 个载波拍）才算一次。
 *============================================================================*/
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_BusVoltLoop);
/**
 * @brief DAB 母线电压外环控制
 * @param f32Vo       当前输出母线电压采样值 (V)
 * @param f32RefMin   电流下限给定 (通常为充电最低流或防倒灌限制) (A)
 * @param u16IsCharge 是否处于充电工况 (1: 充电模式, 0: 放电/逆变模式)
 */
static inline void Dab_Ctrl_BusVoltLoop(float f32Vo, float f32RefMin, Uint16 u16IsCharge)
{
    const float f32IPos = Ramp.iPos;     // 斜坡发生器计算出的正向电流上限
    float f32OutMin = f32RefMin;

    /* 1. 误差一阶低通滤波 (每 PWM 载波中断执行一次，滤除开关纹波) 
     *    公式: Err(k) = Err(k-1) + alpha * [ (Vref - Vo) - Err(k-1) ] */
    Bus.f32Err += DAB_BUS_ALPHA * ((Bus.f32Vref - f32Vo) - Bus.f32Err);

    /* 2. 充电降额平滑保持逻辑:
     *    当充电能力瞬间收紧时，限制下限下降速率，防止电流指令跳变造成母线冲击 */
    if ((u16IsCharge != 0U) && (g_stDabIsrState.u16ChgBuckOn != 0U))
    {
        f32OutMin = __fmin(Bus.f32Out, f32RefMin);
        f32OutMin = __fmax(f32OutMin, f32RefMin - DAB_CHARGE_REF_HOLD_A); // 保留滞后缓冲带
        f32OutMin = __fmax(f32OutMin, Ramp.iNeg);                          // 限制不低于反向斜坡下限
    }

    /* 3. 降频执行 PI 运算 (Tick 分频，如每 100us 运行一次) */
    if (Bus.u16Tick == 0U)
    {
        const float f32ErrRaw = Bus.f32Err;
        const float f32Err = DAB_SAT(f32ErrRaw, -40.0f, 40.0f); // 误差硬限幅防溢出 [-40V, +40V]
        const float f32ErrAbs = fabsf(f32Err);

        /* 3.1 Kp 两段折线：3~6V→0.015~0.03，6~12V→0.03~0.18；Ki 仍用 3~9V 单段 T */
        float f32Kp;
        if (f32ErrAbs <= DAB_BUS_KP_E0)
        {
            f32Kp = DAB_BUS_KP_0;
        }
        else if (f32ErrAbs <= DAB_BUS_KP_E1)
        {
            f32Kp = DAB_BUS_KP_0
                    + (DAB_BUS_KP_1 - DAB_BUS_KP_0) * (f32ErrAbs - DAB_BUS_KP_E0) * DAB_BUS_KP_INV_SPAN01;
        }
        else if (f32ErrAbs <= DAB_BUS_KP_E2)
        {
            f32Kp = DAB_BUS_KP_1
                    + (DAB_BUS_KP_2 - DAB_BUS_KP_1) * (f32ErrAbs - DAB_BUS_KP_E1) * DAB_BUS_KP_INV_SPAN12;
        }
        else
        {
            f32Kp = DAB_BUS_KP_2;
        }

        const float f32T = DAB_SAT((f32ErrAbs - DAB_BUS_GAIN_E_LO) * DAB_BUS_GAIN_INV_SPAN, 0.0f, 1.0f);
        /* 比例故意乘未夹位偏差：大误差时仍按满幅 Kp×真实压差要电流；
         * ±40V 只约束 Kp/Ki 折线与积分，输出再由 CapHi 摆率收口 */
        const float f32Prop = f32Kp * f32ErrRaw;
        float f32Ki = DAB_BUS_KI_LO + DAB_BUS_KI_SPAN * f32T;
        float f32Integ = Bus.f32Integ;                                              // 上一拍积分量 I
        const float f32Cmd = f32Integ + f32Prop;                                    // 当前期望未限幅指令

        /* 3.3 饱和检测与输出摆率(Slew Rate)限制 */
        const Uint16 u16BothDutySat = (stDabPi0.u16DutySat != 0U) && (stDabPi1.u16DutySat != 0U);
        const float f32LoadStep = (u16BothDutySat != 0U) ? 0.0f : DAB_BUS_OUT_SLEW_A; // 饱和时冻结步长

        /* 3.4 动态上下轨约束边界计算 */
        const float f32LimHi = (u16IsCharge != 0U) ? __fmin(f32IPos, BUS_CHG_OUT_MAX) : f32IPos;
        const float f32CapLo = (u16IsCharge != 0U) ? __fmax(f32OutMin, Bus.f32Out - f32LoadStep) : f32OutMin;
        const float f32CapHi = (u16IsCharge != 0U) ? f32LimHi : __fmin(f32LimHi, Bus.f32Out + f32LoadStep);

#if DAB_BUS_LS_ENABLE
        /* 3.5 [可选] 大信号超前预测加速:
         *     利用差分预测未来误差 ErrPred = Err + Tlead * dErr/dt，同向变大时强制提高 Ki */
        {
            const float f32ErrPred = f32ErrRaw + DAB_BUS_T_LEAD * (f32ErrRaw - Bus.f32Err1) * DAB_BUS_FS;
            const float f32ErrDrive = __fmin(f32ErrAbs, (f32ErrRaw >= 0.0f) ? f32ErrPred : -f32ErrPred);
            const float f32TLs = DAB_SAT((f32ErrDrive - DAB_BUS_LS_OFF_V) * DAB_BUS_LS_INV_SPAN, 0.0f, 1.0f);
            f32Ki += (DAB_BUS_KI_FAST - f32Ki) * f32TLs; // 大扰动下提升积分强度
        }
#endif

        /* 3.6 动态抗积分饱和 (Anti-Windup): 
         * 双桥硬件饱和时，只允许向“脱离饱和”的方向积分；未饱和时只累加余量部分 */
        if ((u16BothDutySat == 0U) || (u16IsCharge ? (f32Err > 0.0f) : (f32Err < 0.0f)))
        {
            const float f32DeltaInteg = f32Ki * f32Err;
            if (f32DeltaInteg > 0.0f)
            {
                /* 正向积分：不能超过本拍允许输出的最大上限余量 (CapHi - Cmd) */
                f32Integ += __fmin(f32DeltaInteg, __fmax(f32CapHi - f32Cmd, 0.0f));
            }
            else if (f32DeltaInteg < 0.0f)
            {
                /* 负向积分：不能低于本拍允许输出的最大下限余量 */
                f32Integ += __fmax(f32DeltaInteg, __fmin(__fmax(f32CapLo, f32RefMin) - f32Cmd, 0.0f));
            }
        }

        /* 3.7 状态更新与最终输出限幅 */
        Bus.f32Integ = DAB_SAT(f32Integ, f32OutMin, f32LimHi);                      // 保存限幅后的积分量
        Bus.f32Out = __fmin(__fmax(Bus.f32Integ + f32Prop, f32CapLo), f32CapHi);   // PI叠加后施加摆率硬限幅
        Bus.f32Err1 = f32ErrRaw;                                                    // 记录历史误差供微分使用
    }
    else if (u16IsCharge != 0U)
    {
        /* 4. 在不执行 PI 的快速中断拍中，直接响应充电下限收紧，提高安全性 */
        Bus.f32Out = __fmax(Bus.f32Out, f32OutMin);
    }

    /* 5. 发送给内环电流环的最终目标参考值 Ref (限幅在 [RefMin, IPos] 之间) */
    Bus.f32Ref = __fmin((u16IsCharge != 0U) ? __fmax(Bus.f32Out, f32RefMin) : Bus.f32Out, f32IPos);
}

/*==============================================================================
 * ② 放电内环
 *============================================================================*/

/**
 * @brief DAB 放电模式非线性模型前馈计算函数 (Model-Based Feedforward)
 * @param f32Iref 单个 DAB 桥的目标输出电流给定值 (单位: A)
 * @return float  虚拟总控制油门 u ∈ [0.0f, 1.0f]
 *                - 0.00 ~ 0.50   : HV 固定 0.45，纯调低压侧脉宽占空比 b (0.10 ~ 0.44，降压轻载)
 *                - 0.50 ~ 0.6111 : LV 锁死 0.44，HV 从 0.45 扩宽至 0.50 (q=0 过渡段)
 *                - 0.6111 ~ 1.00 : LV 锁死 0.44，HV 超过 0.50 产生移相重叠量 q = HvDuty - 0.50 (升压或降压重载)
 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_DisFfCalc);
static inline float Dab_Ctrl_DisFfCalc(float f32Iref)
{
    /* =========================================================================
     * 0. 变量读取与安全防呆拦截
     * ========================================================================= */

    /* 电池折算到高压母线侧的等效电压倒数：1 / E = 1 / (n * Vbat)
     * 【大白话】：n 是变压器匝比。比如电池 50V，匝比 8，折算过去就是 E = 400V。
     * 【优化技巧】：外部慢速任务预先算好倒数，中断内用乘法代替除法，节省 DSP 时钟周期。 */
    const float f32InvE = g_stDabIsrState.f32DisInvE;

    /* 电压落差比：R = Vref / E = 设定母线电压 / 电池折算电压
     * - R >= 1.0：目标母线比电池高，水往高处抽 (升压 Boost 模式)
     * - R <  1.0：目标母线比电池低，水往低处放 (降压 Buck 模式)
     * 【工程设计】：采用母线给定值 Vref 计算，不受实测母线开关纹波干扰，前馈输出更平稳。 */
    const float f32R    = g_stDabIsrState.f32DisRatio;

    /* 【1. 异常与极小电流拦截】
     * 规则：电池没电(InvE <= 0) 或 电流给定连 0.05A 都不到(芝麻大点流)，直接关前馈输出 0.0f。
     * 防御机制：采用 !(A && B) 写法，若浮点数溢出变为 NaN，也能被安全抓捕直接返回 0。 */
    if (!(f32InvE > 0.0f && f32Iref > DAB_DIS_FF_IREF_MIN))
    {
        return 0.0f;
    }


    /* =========================================================================
     * 1. 起步防蹿车软上限 & 标幺化电流 J 计算
     * ========================================================================= */

    /* 【公式】：FfMax = min( DAB_DIS_U_MAX, (Iref - 0.05A) * (1 / 0.20A) )
     * 【大白话】：升压区移相段默认从 u = DAB_DIS_FF_U_Q0 (≈0.6111) 起跑。如果给定刚到 0.06A 就直接给 0.6111 的大油门，
     * 车子会猛蹿一下（母线电压过冲）。所以在单桥 0.05A ~ 0.25A 这段极窄起步区间内，
     * 给前馈设一个随电流线性放宽的软上限 FfMax (0.0 -> DAB_DIS_U_MAX)。前馈不足部分由 PI 柔和垫上。 */
    const float f32FfMax   = __fmin(DAB_DIS_U_MAX, (f32Iref - DAB_DIS_FF_IREF_MIN) * DAB_DIS_FF_START_SPAN_INV);

    /* 【公式：无量纲比例系数】 JPerAmp = K / E = (0.8 * fs * Lk) / E
     * 常数 DAB_DIS_FF_K = 0.8 * fs(60kHz) * Lk(35uH) = 1.68（按标称漏感 -20% 留裕量，标称稳态约承担 80% 电流，余量由 PI 修平） */
    const float f32JPerAmp = DAB_DIS_FF_K * f32InvE;

    /* 【公式：将物理安培电流换算为无量纲纯力气 J】 J = Iref * K / E = Iref * (0.8 * fs * Lk) / E
     * 物理意义：不管实际是几百伏、几十安，统一换算成变压器电感储能的无量纲数值 J。 */
    const float f32J       = f32Iref * f32JPerAmp;

    /* 虚拟总油门 u：初始默认卡在 q=0 移相起点 DAB_DIS_FF_U_Q0 (≈0.6111，对应 HV=0.50)。
     * 升压模式或降压大重载直接在此基础上叠加推力 q；降压轻载则会从实际脉宽重新起步。 */
    float f32U = DAB_DIS_FF_U_Q0;
    float f32Q; /* 水泵加压推力分量 q (两侧移相角) */


    /* =========================================================================
     * 2. 分支一：往高处抽水 (升压模式: R >= 1.0)
     * ========================================================================= */
    if (f32R >= 1.0f)
    {
        /* 升压工况下，低压侧脉宽全程开到最大固定死 (b = B = DAB_DIS_FF_B = 0.44)，
         * 纯靠加大水泵推力 q (移相角) 来控制输出电流。 */

        /* 计算升压相对压差比例：ΔR = R - 1 */
        const float f32Dr = f32R - 1.0f;

        /* 【公式：断续三角波(DCM) 与 连续梯形波(CCM) 的物理分界电流 J1】
         * 推导公式：J1 = B^2 * (R - 1) / R^2
         * - 当 J <= J1 时：水流是一瓢一瓢泼过去的，每次电感电流都归零（轻载断续 DCM）；
         * - 当 J >  J1 时：水流连成一条线，电感电流再也不归零（重载连续 CCM）。 */
        const float f32J1 = __divf32(DAB_DIS_FF_B2 * f32Dr, f32R * f32R);

        if (f32J <= f32J1)
        {
            /* --- 2.1 升压轻中载 (DCM 三角波模式)：物理精确开方解 ---
             * 【物理公式】：DCM 下传输电流与推力平方成正比：J = q^2 / (R - 1)
             * 【反推求 q】：两边直接开平方根：q = sqrt( J * (R - 1) ) */
            f32Q = __sqrt(f32J * f32Dr);
        }
        else
        {
            /* --- 2.2 升压重载 (CCM 梯形波模式)：一阶切线光滑近似 (C1 连续) ---
             * 【为什么不用精确解？】：CCM 真实方程是一元二次方程求根，计算耗时且有负数开方风险。
             * 【切线公式推导】：在分界点 J1 处，沿着上面 DCM 抛物线末尾顺延作一条"切线滑梯"：
             *   1. 在 J = J1 处的函数值：q(J1) = B * ΔR / R
             *   2. 在 J = J1 处的切线斜率：dq/dJ = R / (2 * B)
             *   3. 点斜式直线方程：q(J) = q(J1) + (dq/dJ) * (J - J1)
             * 化简后得到本行代码：q = [B * (R - 1) / (2 * R)] + [R / (2 * B)] * J
             * 【优点】：仅用 1 次加法和 1 次乘法，计算极快！且在 J=J1 切换点处，数值与斜率
             *          100% 平滑衔接（C1 连续），模式切换无任何 PWM 抖动与母线冲击！ */
            f32Q = __divf32(DAB_DIS_FF_B * f32Dr, 2.0f * f32R) + f32R * f32J * DAB_DIS_FF_INV_2B;
        }
    }


    /* =========================================================================
     * 3. 分支二：往低处放水 (降压模式: R < 1.0)
     * ========================================================================= */
    else
    {
        /* 降压工况下：
         * 小电流时：推力 q = 0，单靠拧大阀门 b 控制（0.00 ~ 0.50 油门段）；
         * 大电流时：阀门 b 开到最大、HV 扩宽到 0.50 后，启动移相推力 q 助力（0.6111 ~ 1.00 油门段）。 */

        /* 【公式：纯开阀门能管用的最大极限电流分界点 J0】
         * 推导公式：J0 = 2 * (1 - R) * B^2 / (1 + R)
         * 物理意义：不加任何移相推力 (q=0)，把脉宽 b 拉满到最大值 B 时所能输出的最大极限电流。 */
        const float f32J0 = __divf32(2.0f * (1.0f - f32R) * DAB_DIS_FF_B2, 1.0f + f32R);

        /* 判断：当前电流是否处于【纯开阀门段】或者【分界点 J0 后的平滑过渡带】 */
        if (f32J < (1.0f + DAB_DIS_FF_BLEND_WIDTH) * f32J0)
        {
            /* 【安全紧箍咒：阀门最大开度限制 BMax】
             * 【公式】：BMax = min( B, 0.25 * (1 + R) - 0.025 )
             * 【大白话】：在深降压 (R < 0.86) 时，为了保证软开关 (ZVS) 并不发生无功反冲砸管，
             * 阀门不敢开到标称最大值 B = 0.44，必须随电压比 R 线性收紧上限（如 R=0.70 时限制在 0.40）。 */
            const float f32BMax = __fmin(DAB_DIS_FF_B, 0.25f * (1.0f + f32R) - 0.025f);
            float f32B = f32BMax; /* 默认先假定脉宽达到最大限位 */

            /* --- 3.1 纯开阀门轻载段 (J < J0) --- */
            if (f32J < f32J0)
            {
                /* 计算当前电流需求占分界能力 J0 的比例：K = J / J0 ∈ [0, 1) */
                const float f32K = __divf32(f32J, f32J0);

                /* 【公式：反推物理阀门开度 b】 b = min( B * sqrt(K), BMax )
                 * 物理原理：轻载下输出电流与脉宽平方成正比 (J ∝ b^2)，反推脉宽需开根号！ */
                f32B = __fmin(DAB_DIS_FF_B * __sqrt(f32K), f32BMax);
            }
            /* 【线性映射】：将物理脉宽 b 转换为前半截虚拟油门 u (0.0 ~ 0.50)
             * 公式：u = (b - b_min) * 斜率倒数 */
            f32U = (f32B - DAB_DUTY_LV_MIN) * DAB_LV_INV_SLOPE;

            /* 如果完全处于 J <= J0 纯脉宽段，前馈计算直接搞定，安全限幅后返回！ */
            if (f32J <= f32J0)
            {
                return DAB_SAT(f32U, 0.0f, f32FfMax);
            }

            /* --- 3.2 J0 之后的平滑消除断层过渡段 (J0 < J < 1.2*J0) ---
             * 【为什么要这一段？】：纯调脉宽段终点 b = 0.44 对应 u = 0.500（此时 HV 仍为 0.45），
             *  而启动移相的起点 DAB_DIS_FF_U_Q0 = 0.6111（对应 HV = 0.50、q = 0），两段之间天然存在
             *  Δu ≈ 0.111 的 HV 扩宽断层；若 R < 0.86 触发 BMax 限幅，前半段终点低于 0.500，断层还会进一步扩大。
             * 【平滑算法】：在 J0 之后的过渡带内，用三次 S 型平滑曲线把 f32U → DAB_DIS_FF_U_Q0 的跨度平缓接上！ */

            /* 计算分界点距离单桥硬件最大电流极限还剩多少行程：Room = I_limit - J0 */
            const float f32Room = DAB_CURR_LIMIT * DAB_CURR_PER_BRIDGE * f32JPerAmp - f32J0;

            /* 自适应计算过渡带宽度 Width：
             * 【公式】：Width = min( 0.2 * J0, max( 0.40A 对应力气, Room ) )
             * 大白话：平时给 20%*J0 宽度平滑过渡；快到极限电流时按余量缩短，但底线保底留 0.40A 宽度防抖。 */
            const float f32Width = __fmin(DAB_DIS_FF_BLEND_WIDTH * f32J0,
                                         __fmax(DAB_DIS_FF_HI_MIN_A * f32JPerAmp, f32Room));

            /* 归一化过渡进度：t = (J - J0) / Width ∈ [0.0, 1.0] */
            const float f32T = __fmin(1.0f, __divf32(f32J - f32J0, f32Width));

            /* 【公式：三次 Smoothstep 平滑曲线权重】 Weight(t) = 3*t^2 - 2*t^3
             * 【数学之美】：导数 dWeight/dt = 6*t*(1 - t)，在起点 t=0 和终点 t=1 处导数严格归零！
             * 像高铁进出站一样极度平缓，消灭增益跳变引起的极限环抖动。 */
            const float f32Weight = f32T * f32T * (3.0f - 2.0f * f32T);

            /* 将前半段终点平缓过渡到移相起点 DAB_DIS_FF_U_Q0 (≈0.6111)：u = u + Weight * (U_Q0 - u) */
            f32U += f32Weight * (DAB_DIS_FF_U_Q0 - f32U);
        }

        /* --- 3.3 降压大重载水泵推力计算 (J > J0) ---
         * 阀门开满后不够用的超额电流部分 (J - J0)，全部依靠叠加移相推力 q 补齐
         * 【切线公式推导】：q = [(1 + R) / (4 * R * B)] * (J - J0) */
        f32Q = __divf32((1.0f + f32R) * (f32J - f32J0), 4.0f * f32R * DAB_DIS_FF_B);
    }


    /* =========================================================================
     * 4. 最大功率点 (MPP) 机械限位与总油门合成输出
     * ========================================================================= */

    /* 【核心物理保命铁律：防止油门踩爆导致正反馈炸机！】
     * 物理原理：DAB 传输功率关于移相角 q 是一条正弦馒头形曲线。
     * 越过最高点（最大功率点 MPP）之后，继续加大推力 q，输出电流反而会下降！
     * 控制器要是发现电流不够继续死命踩油门，就会陷入"油门越大出力越小"的恶性死循环，瞬间烧管炸机！
     * 【公式：最大安全推力上限】 q_max = (2 * R * B) / (1 + 2 * R)
     * 代码对移相量 q 施加物理绝对死卡：q 绝不准翻过山头！ */
    f32Q = __fmin(f32Q, __divf32(2.0f * f32R * DAB_DIS_FF_B, 1.0f + 2.0f * f32R));

    /* 【总油门合成】：
     * 在基础油门 f32U (稳态为 U_Q0 ≈ 0.6111，降压过渡带内平滑爬升至 0.6111) 之上，叠加移相分量：u = u + q / DAB_HV_SLOPE */
    f32U += f32Q * DAB_HV_INV_SLOPE;

    /* 最终安全饱和限幅：将总油门死死约束在 [0.0f, 当前允许的最大软上限 f32FfMax] 范围内输出！ */
    return DAB_SAT(f32U, 0.0f, f32FfMax);
}

/* 按上拍映射输入 u 选增益调度倍率，分段边界与 Dab_Ctrl_DisDutyMap 一致 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_DisGainSched);
static inline float Dab_Ctrl_DisGainSched(float f32DutyU)
{
    if (f32DutyU <= DAB_U_MID2)
    {
        return DAB_SCHED_S5_SCALE;   /* LV 段 */
    }
    return DAB_SCHED_S4_SCALE;       /* HV 段 */
}

/**
 * @brief 放电单桥电流环：算本拍该发多大油门 Duty
 *
 * 流水线：模型先给一大块前馈 Ff → PI 再微调 Out → Duty = Out + Ff。
 * 两桥各调一次；电流给定恒为正（放电），单桥目标 = 总给定 × 0.5。
 * 关波 / 甩载 / NaN 由上层打嗝门一拍清零（含 f32DutyFf），进来后只处理正给定。
 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_DisCurrLoop);
static inline void Dab_Ctrl_DisCurrLoop(ST_DAB_PI *pstPi, ST_DAB_DUTY *pstDuty,
                                        float f32Current, float f32DffBase)
{
    /* =========================================================================
     * 0. 本拍差多少电流
     * ========================================================================= */

    /* 上一拍已发出去的油门，后面用来算“离顶还剩多少” */
    const float f32Duty    = pstDuty->f32Duty;

    /* 两桥一人一半：单桥目标电流 = 0.5 × Bus.f32Ref */
    const float f32Ref     = DAB_CURR_PER_BRIDGE * Bus.f32Ref;

    /* 【公式】Err = Ref − 实测
     * >0：电流不够，要加油门；<0：超了，要收油门 */
    const float f32Err     = f32Ref - f32Current;


    /* =========================================================================
     * 1. 这一拍最多允许走多远 + 前馈怎么跟
     * ========================================================================= */

    /* 【公式】StepMax = max( (1 − Duty) / 5 , 0.1 )
     * 离满油门还远时，一步最多走剩余路程的 1/5；快到顶时步子变小，但最少也给 0.1。
     * 注意：Duty>0.5 时 (1−Duty)/5 已经 < 0.1，后半段其实一直踩在 0.1 地板上。 */
    const float f32StepMax = __fmax((DAB_DIS_U_MAX - f32Duty) * DAB_DUTY_MAX_STEPS_INV, DAB_DUTY_STEP_MIN);

    /* 【公式】FfRaw = min( 模型理想值 , 上一拍 Ff + StepMax )
     * 往上加：每拍最多涨 StepMax，避免刚醒来前馈猛跳把电流冲高；
     * 往下砍：模型一降就立刻跟，甩载不等待。 */
    const float f32FfRaw   = __fmin(f32DffBase, pstPi->f32DutyFf + f32StepMax);

    /* 再卡死在 [0, 1]。DAB_SAT 会把入参算两遍，所以先落到 f32FfRaw 再送进去。 */
    const float f32Ff      = DAB_SAT(f32FfRaw, 0.0f, DAB_DIS_U_MAX);


    /* =========================================================================
     * 2. PI 算出本拍要加减多少油门
     * ========================================================================= */

    /* 【公式】RawPi = (K1×本拍误差 − K2×上拍误差) × 段增益
     * 只算“这一拍相对上一拍该动多少”，不重新累一大坨。
     * K1=Kp+Ki·Ts，K2=Kp；DisGainSched 看当前 Duty 在前半段还是后半段，
     * 后半段物理增益变钝时把系数放大，两边响应速度尽量差不多。 */
    const float f32RawPi   = (pstPi->f32K1 * f32Err - pstPi->f32K2 * pstPi->f32Err1)
                             * Dab_Ctrl_DisGainSched(f32Duty);

    /* 【公式】Δu = 把 RawPi 夹到 [−0.2 , +StepMax]
     * 加油门：一步不能超过 StepMax；收油门：允许一拍最多掉 0.2，退得更快。 */
    const float f32DeltaPi = DAB_SAT(f32RawPi, -DAB_DUTY_STEP_DOWN, f32StepMax);


    /* =========================================================================
     * 3. PI 还能加到哪 —— 前馈占掉的空间要让出来
     * ========================================================================= */

    /* 【公式】OutHi = 1 − Ff
     * 总油门 Duty = Out + Ff 不能超过 1，前馈已经占了 Ff，PI 最高只能到 1−Ff。
     * 到顶后再往上加也会被下一行夹回来，不会在后台越积越多（抗积分饱和）。 */
    const float f32OutHi   = DAB_DIS_U_MAX - f32Ff;

    /* 先加上本拍 Δu，得到“想去的位置”；真正写入前还要夹一次 */
    const float f32OutNext = pstPi->f32Out + f32DeltaPi;


    /* =========================================================================
     * 4. 写回记忆，拼出最终油门
     * ========================================================================= */

    pstPi->f32Ref     = f32Ref;   /* 记下本拍单桥目标，方便观测 */
    pstPi->f32DutyFf  = f32Ff;    /* 本拍生效的前馈，下拍缓入从这里起步 */
    pstPi->f32Err1    = f32Err;   /* 本拍误差留给下拍做“上拍误差” */

    /* 用“想去的位置”判是否顶满：OutNext ≥ OutHi 就置位，通知外环别再往上积分。
     * 为什么不先夹再判？浮点加减会有一点点尾巴，先夹成 OutHi 再加 Ff，
     * 总和可能变成 0.999999，按 ≥1 去比会漏一拍。 */
    pstPi->u16DutySat = (f32OutNext >= f32OutHi) ? 1U : 0U;

    /* 【公式】Out = 把 OutNext 夹到 [−Ff , OutHi]
     * 上限：Out+Ff ≤ 1；下限：Out+Ff ≥ 0。直接夹死，不再用 if 决定加不加。 */
    pstPi->f32Out     = DAB_SAT(f32OutNext, -f32Ff, f32OutHi);

    /* 【公式】Duty = Out + Ff，再兜底夹到 [0, 1]，交给发波 */
    pstDuty->f32Duty  = DAB_SAT(pstPi->f32Out + f32Ff, 0.0f, DAB_DIS_U_MAX);
}

/* 放电占空比分段映射：u ∈ [0,1] → HV / LV 占空比
 *
 * u < 3% 直接不发波，是刻意不设迟滞的设计（理由见下方注释）：
 *   ① 出死区时 LV 固定从 10% 起，脉宽远大于最小可驱动脉宽，不会产生窄脉冲；
 *   ② 进死区后 HV 边界动作强制 CLEAR、LV 开通沿全改 NO_ACTION，磁通从零起，不累积偏磁；
 *   ③ burst 重复频率随负载变化，轻载自然降到秒级，重载平滑过渡为连续发波。
 * 加迟滞只会把 burst 挪到更低频、单次能量更大，母线纹波和听感噪声反而变差。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_DisDutyMap);
static inline void Dab_Ctrl_DisDutyMap(float f32Duty, float *pf32HvDuty, float *pf32LvDuty)
{
    float f32HvDutyTmp;
    float f32LvDutyTmp;

    f32Duty = DAB_SAT(f32Duty, 0.0f, DAB_DIS_U_MAX);

    if (f32Duty < DAB_U_MIN_PACKET)
    {
        *pf32HvDuty = 0.0f;
        *pf32LvDuty = 0.0f;
        return;
    }

    if (f32Duty <= DAB_U_MID2)
    {
        /* 0.03~0.50：HV 固定 45%，LV = 10% + 0.68·u */
        f32HvDutyTmp = DAB_DUTY_HV_BASE;
        f32LvDutyTmp = DAB_DUTY_LV_MIN + DAB_LV_SLOPE * f32Duty;
    }
    else
    {
        /* >0.50：LV 锁 44%，HV = 45% + 0.45·(u-0.50) */
        f32LvDutyTmp = DAB_DUTY_LV_MAX;
        f32HvDutyTmp = DAB_DUTY_HV_BASE + DAB_HV_SLOPE * (f32Duty - DAB_U_MID2);
    }

    /* LV 不超过 HV；当前常量下 LV 最大 0.44 < HV 最小 0.45 */
    *pf32LvDuty = (f32LvDutyTmp < f32HvDutyTmp) ? f32LvDutyTmp : f32HvDutyTmp;
    *pf32HvDuty = f32HvDutyTmp;
}

/*==============================================================================
 * ③ 充电内环
 *============================================================================*/

/* SR 状态清零（全工程唯一清单：新增 SR 字段只改这里）。
 * 换向/打嗝复位、占空比归零两处共用，防止残留使能小流量倒流。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_SrClear);
static inline void Dab_Ctrl_SrClear(ST_DAB_DUTY *pstDuty)
{
    pstDuty->u16SrEnable = 0U;
    pstDuty->u16SrOnCnt  = 0U;
    pstDuty->u16SrSoft   = 0U;
    pstDuty->f32SrDuty   = 0.0f;
}

/* 清本路内环输出（快断 / 换向 / 充放电打嗝共用）。
 * 电压外环（Bus）不动，解封后不用从头爬。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_ChgOutputReset);
static inline void Dab_Ctrl_ChgOutputReset(ST_DAB_PI *pstPi, ST_DAB_DUTY *pstDuty)
{
    pstPi->f32Out     = 0.0f;   /* 电流环 PI 输出 */
    pstPi->f32Err1    = 0.0f;   /* 上拍误差 */
    pstPi->f32DutyFf  = 0.0f;   /* 占空比前馈 */
    pstPi->u16DutySat = 0U;     /* 清后外环按未饱和正常积分 */
    pstDuty->f32Duty  = 0.0f;
    Dab_Ctrl_SrClear(pstDuty);
}

#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_OutputReset);
static inline void Dab_Ctrl_OutputReset(void)
{
    Dab_Ctrl_ChgOutputReset(&stDabPi0, &stDabDuty0);
    Dab_Ctrl_ChgOutputReset(&stDabPi1, &stDabDuty1);
}

/* SR 使能：只看实测瞬时功率。
 *   关断——否定式，任一拍跌破下限（含 NaN）当拍即撤。滤波值有滞后，
 *         滞后期间电流已回流，外环会失去控制权，所以必须用瞬时量；
 *   开通——要求连续 N 拍都过开通门限，避免纹波/噪声导致反复开关；
 *   软起——使能后前 N 拍窗口按 1/N 线性放开，把万一误开的冲击摊到几拍里。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_SrStateUpdate);
static inline void Dab_Ctrl_SrStateUpdate(ST_DAB_DUTY *pstDuty, float f32InstW)
{
    if (!(f32InstW >= DAB_SR_OFF_INST_W))
    {
        pstDuty->u16SrEnable = 0U;
        pstDuty->u16SrOnCnt  = 0U;
        pstDuty->u16SrSoft   = 0U;
    }
    else if (f32InstW >= DAB_SR_ON_INST_W)
    {
        if (pstDuty->u16SrOnCnt < DAB_SR_ON_CNT)
        {
            pstDuty->u16SrOnCnt++;
        }
        if (pstDuty->u16SrOnCnt >= DAB_SR_ON_CNT)
        {
            pstDuty->u16SrEnable = 1U;
            if (pstDuty->u16SrSoft < DAB_SR_SOFT_CNT)
            {
                pstDuty->u16SrSoft++;
            }
        }
    }
    else
    {
        pstDuty->u16SrOnCnt = 0U;   /* 回差带内保持现状，但重开要重新计数 */
    }
}

/* 充电单路电流环：PI 在电流域工作，再经开方逆模型得占空比。
 * 三段式结构：① 电流域 PI 步（Dab_Ctrl_ChgPiStep）② 开方逆模型 ③ SR 判据。
 * 两路共用的拍级参数收敛进 ST_DAB_CHG_CTX（Main 填一次传同一指针），
 * 形参从 9 个收敛到 4 个，与放电环对齐。
 *
 * 符号约定：f32Iref 恒为正（充电需求幅值）；f32Curr 带符号采样，充电时为负
 * （与 Bus.f32Ref 同号）；f32IcmdMax 恒为正（本路能力上限）。
 *
 *   误差:  e(n)    = Iref - Ifdb          （充电 I < 0，反馈取反即幅值：e = Iref - (−I)）
 *   修正:  Corr(n) = clamp( Corr(n-1) + K1·e(n) - K2·e(n-1),
 *                          -(NEG_K·Iref + ABS), POS_K·Iref + ABS )
 *   指令:  i_cmd   = clamp( Iref + Corr, 0, IcmdMax )
 *   抗饱和: Corr'  = Corr + 0.5·( i_cmd - (Iref + Corr) )
 *   占空比: d      = min( sqrt(KFL·i_cmd/ΔV), DutyMax )
 *
 * 逆模型抵消 d² 非线性，PI 看到的对象增益约为 1，系数不随工作点变；
 * 漏感偏差表现为电流比例误差，由 Corr 吸收。[调用前提] 由 Dab_Ctrl_Loop 保证
 * 发波条件成立（压差足够、未休眠、限流 > 0），本函数内部不再判断。 */

/* 充电电流域 PI 步：误差 → 增量 PI → 对称限幅 → 指令夹位 → back-calc 抗饱和。
 * 单极性控制：实测幅值低于给定(e>0)就加力，高于给定(e<0)就收。
 * 误差直接写成 e = f32Iref + f32Curr（f32Curr 充电为负，加号即减幅值）。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_ChgPiStep);
static inline float Dab_Ctrl_ChgPiStep(ST_DAB_PI *pstPi, float f32Curr,
                                       const ST_DAB_CHG_CTX *pstCtx)
{
    /* ① 本拍给定幅值(A，恒正)：外环算成 max(−0.5·Bus.f32Ref, 0.02) 再被任务限流 fmin 压住。 */
    const float f32Iref    = pstCtx->f32Iref;

    /* ② 本路能力上限(A，恒正)：由压差和占空比上限折算，是“物理能给多少”，不是保护阈值。 */
    const float f32IcmdMax = pstCtx->f32IcmdMax;
    const float f32CorrMax = __fmax(f32IcmdMax - f32Iref, 0.0f);

    float f32Err;    /* 本拍误差 e(n) */
    float f32Corr;   /* 本拍修正量 Corr(n)：叠在给定上的“垫量” */
    float f32Raw;    /* 未夹位的指令 Iref+Corr */
    float f32Icmd;   /* 夹位后的指令，交给开方逆模型 */

    /* ③ 对外记账：系统约定充电记负、放电记正的带符号口径，CAN 观测和换向判决都读它。
     *    环内全程用幅值(正数)，只在这里翻一次符号。 */
    pstPi->f32Ref = -f32Iref;

    /* ④ 本拍误差 = 要的 − 实际。例：给定 3.0A、实测 −2.8A → e = 3.0 + (−2.8) = +0.2A（还差，加力）。 */
    f32Err  = f32Iref + f32Curr;

    /* ⑤ 增量式 PI：Corr(n) = Corr(n−1) + K1·e(n) − K2·e(n−1)
     *      K1 = α·(1+wz·Ts) = 0.31416×1.08 = 0.3393，K2 = α = 0.31416
     *    电流偏大时，最多减掉整个给定，让电流指令降到 0。
     *    还差电流时，最多加到本拍能力上限。给定已到上限时，正向余量为 0。 */
    f32Corr = DAB_SAT(pstPi->f32Out + DAB_CHARGE_K1 * f32Err - DAB_CHARGE_K2 * pstPi->f32Err1,
                      -f32Iref, f32CorrMax);

    /* ⑥ 用完正向余量时直接取能力上限，避免先减后加的舍入把指令留在上限下方。
     *    给定已超过能力时，保留超出的给定，供后面的限幅回算。 */
    f32Raw = (f32Corr >= f32CorrMax) ? __fmax(f32Iref, f32IcmdMax) : (f32Iref + f32Corr);

    /* ⑦ 指令夹位到 [0, IcmdMax]：下限 0 是硬约定（反向功率归放电逻辑，充电环不许出负指令），
     *    上限是物理能力，超了只是把占空比顶到天花板、白让 PI 积分。 */
    f32Icmd = DAB_SAT(f32Raw, 0.0f, f32IcmdMax);

    /* ⑧ back-calc 抗饱和：只改“下一拍用的积分器”，不改本拍输出。被夹住时把差额一半扣回 Corr，
     *    约 3~4 拍退干净；没夹住时差额为 0，Corr 原样保留。
     *      顶上限 → 差额为负 → Corr 往下退；  到下限 → 差额为正 → Corr 往上补。 */
    pstPi->f32Out  = f32Corr + 0.5f * (f32Icmd - f32Raw);

    /* ⑨ 存本拍原始误差（不是 Corr），供下一拍 −K2·e(n−1) 用。 */
    pstPi->f32Err1 = f32Err;

    /* ⑩ 输出本拍电流指令(A)。 */
    return f32Icmd;
}

/* 充电单路电流环（本拍动作）：① 电流域 PI 出 i_cmd ② 开方逆模型出占空比 ③ SR 判据。
 * 谁决定模式：Dab_Ctrl_Loop 的上层状态机；本函数只管本拍占空比和 SR，不再判发波条件。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_ChgCurrLoop);
static inline void Dab_Ctrl_ChgCurrLoop(const ST_DAB_CHG_CTX *pstCtx, ST_DAB_PI *pstPi,
                                        ST_DAB_DUTY *pstDuty, float f32Curr)
{
    /* 本拍占空比上限：绝对上限 0.43 / ZCS 允许 / 峰值电流允许 三者取小，Main 每拍算好。
     * 母线越高、压差越小，它能给的上限越低——是“能开多宽”，不是保护阈值。 */
    const float f32DutyMax = pstCtx->f32DutyMax;

    /* ① 电流域 PI 步 → 出本拍电流指令 i_cmd(A)。饱和与抗饱和已在上一节做完。 */
    const float f32Icmd = Dab_Ctrl_ChgPiStep(pstPi, f32Curr, pstCtx);

    /* ② 开方逆模型(TMU 硬件 sqrt)：i(d) = ΔV·d²/KFL ⇒ d = sqrt(KFL·i_cmd/ΔV)
     *    KFL = fs·Lk = 50kHz×35µH = 1.75；ΔV = Vbus − n·Vbat 取倒数存进 f32InvDv（乘法替除法）。
     *    d 随 i 走平方根，PI 看到的对象在电流域近似线性，K1/K2 才敢固定。
     *    最后 fmin 到 DutyMax：算多了按上限发，差额由 u16DutySat 上报外环。 */
    const float f32Duty = __fmin(__sqrt(DAB_CHARGE_KFL * f32Icmd * pstCtx->f32InvDv), f32DutyMax);

    /* ③ 本拍占空比下发，这一行之后 PWM 按它发波（占空比 = CMP/2/TBPRD）。 */
    pstDuty->f32Duty  = f32Duty;

    /* ④ CAN 观测复用同一次开方：充电侧此字段存的就是本拍实际占空比，Main 不再重算标称值。 */
    pstPi->f32DutyFf  = f32Duty;

    /* ⑤ 饱和标志交给外环：电流域顶 IcmdMax 或占空比域顶 DutyMax 都算饱和，外环据此停积分。
     *    在“夹位发生的那个域”里比较，操作数精确相等，避开 sqrt 反算占空比的 1ULP 漏判。
     *    用 >=：正好等于上限也算饱和。 */
    pstPi->u16DutySat = ((f32Icmd >= pstCtx->f32IcmdMax) || (f32Duty >= f32DutyMax)) ? 1U : 0U;

    /* ⑥ 占空比为 0 说明本拍不发波(i_cmd 被夹到 0，sqrt(0)=0)：立即撤 SR，
     *    不留使能态，防止无主管导通时低边管开通造成小流量倒流。给定撤掉当拍停波就落在这里。 */
    if (f32Duty <= 0.0f)
    {
        Dab_Ctrl_SrClear(pstDuty);
    }
    else
    {
        /* ⑦ 按瞬时功率判 SR 态：W = Vbus_slow × |I| = −f32Vslow × f32Curr（Curr 充电为负，取负后为正）。
         *    用瞬时量不用滤波量：该关的那一拍电流已经倒流，滤波值有滞后会错过。
         *    关断：任一拍跌破 OFF 门限当拍就撤（含 NaN）；开通：连续 N 拍过 ON 门限才认。 */
        Dab_Ctrl_SrStateUpdate(pstDuty, -pstCtx->f32Vslow * f32Curr);

        /* ⑧ SR 占空比 = 主占空比 × 匝比折算 − 死区余量，夹在 [0,0.44]。f32SrRatio = min(Vslow,VbusRaw)/E，
         *    取较低母线算保证不超调。再乘软起 u16SrSoft/20：使能后 20 拍从 5% 放到 100%
         *    （50kHz 下 400µs），把误开的冲击摊开。未使能直接给 0，不复用上一拍。 */
        pstDuty->f32SrDuty = (pstDuty->u16SrEnable != 0U)
            ? DAB_SAT(stDabParam.f32SrRatio * f32Duty - DAB_SR_DUTY_MARGIN, 0.0f, DAB_SR_DUTY_MAX)
              * ((float)pstDuty->u16SrSoft * DAB_SR_SOFT_INV)
            : 0.0f;
    }
}

/*==============================================================================
 * ④ 顶层控制步：工况快照 → 充电包络 → 外环 → 内环
 *
 * 电池电压统一取 10ms 滤波值（已夹在 [DAB_BAT_VOLT_MIN, 65V] 内）；
 * 瞬时值只给 EPWM10_ISR 的快保护用，不进控制计算。
 *============================================================================*/
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_Loop);
static inline void Dab_Ctrl_Loop(Uint16 u16IsCharge, Uint16 u16HoldOff)
{
    const float f32VbusRaw = stADC.VmBusVolt.f32IsrResult;
    const float f32I0      = stADC.BusCurr1.f32IsrResult;
    const float f32I1      = stADC.BusCurr2.f32IsrResult;
    const float f32Vslow   = stADC.VmBusVolt.f32Notch;
    ST_DAB_CHG_CTX stChgCtx;
    float f32RefLo = BUS_DISCH_OUT_MIN;
    float f32E     = 0.0f;
    float f32Dv    = 0.0f;
    float f32Dmd;

    Bus.f32Vref = g_CtrlBusRef_V;

    if (u16IsCharge != 0U)
    {
        const float f32VbusPk = __fmax(f32Vslow, f32VbusRaw);
        float f32DutyMax;
        float f32ChgImax;
        float f32CapFilt;

        f32E  = DAB_TURNS_RATIO * stADC.BatVolt.Filt.f32Filt10ms;
        f32Dv = f32Vslow - f32E;

        /* 用较高的母线电压核算 ZCS 和峰值电流上限。 */
        f32DutyMax = __fmin(DAB_CHG_D_LOW_MAX,
                            __fmin(__divf32(DAB_CHARGE_ZCS_MAX * f32E, f32VbusPk),
                                   __divf32(DAB_CHARGE_IPK_MAX * DAB_CHARGE_KFL_MIN,
                                            __fmax(f32VbusPk - f32E, 1.0f))));

        /* 母线侧两桥能力用慢压差和标称漏感计算，下降直跟，上升限步。 */
        f32ChgImax = (float)DAB_BRIDGE_NUM * f32Dv * f32DutyMax * f32DutyMax * DAB_CHARGE_INV_KFL;
        f32CapFilt = __fmin(__fmax(f32ChgImax, 0.0f),
                            g_stDabIsrState.f32CapFilt + DAB_CHARGE_CAP_RISE_A);
        g_stDabIsrState.f32CapFilt = f32CapFilt;

        f32RefLo = -__fmin(__fmax(-Ramp.iNeg, 0.0f), f32CapFilt);
        g_stDabIsrState.f32ChgLimitA = -f32RefLo;

        stChgCtx.f32DutyMax = f32DutyMax;
        stChgCtx.f32IcmdMax = f32ChgImax * DAB_CURR_PER_BRIDGE;

        /* Buck 回差只看物理能力，任务限流只约束电流给定。 */
        if (!(f32CapFilt >= DAB_CHARGE_CAP_OFF_A))
        {
            g_stDabIsrState.u16ChgBuckOn = 0U;
        }
        else if (f32CapFilt >= DAB_CHARGE_CAP_ON_A)
        {
            g_stDabIsrState.u16ChgBuckOn = 1U;
        }
    }
    else
    {
        g_stDabIsrState.u16ChgBuckOn = 0U;
    }

    Dab_Ctrl_BusVoltLoop(f32Vslow, f32RefLo, u16IsCharge);

    /* 方向封波时保留外环给定供换向判决，跳过打嗝和内环。 */
    if (u16HoldOff != 0U)
    {
        goto resetOutput;
    }

    /* 充、放电共用同一套休眠/唤醒阈（充电先取 −Ref 翻成正需求）：
     *   dmd < 0 → 休眠；dmd > A → 唤醒；中间保持；NaN 因 !(dmd >= 0) 归休眠。
     * 充电特例（对照 Bus.f32Ref）：
     *   Ref > 0  → dmd < 0，本拍休眠封波，走不到下面的 0.02A 地板；
     *   Ref == 0 → dmd == 0，不改休眠态：若仍唤醒则继续跑，地板会保活 0.02A/桥；
     *   Ref < 0  → 有充电需求，幅值够大才唤醒（dmd > A）。
     * 是否改放电由状态机另判（如 Ref>+0.5A 计票），与本拍封波不是同一层。 */
    f32Dmd = (u16IsCharge != 0U) ? -Bus.f32Ref : Bus.f32Ref;
    if (!(f32Dmd >= 0.0f))
    {
        g_stDabIsrState.u16BurstOff = 1U;
    }
    else if (f32Dmd > DAB_BURST_HYST_A)
    {
        g_stDabIsrState.u16BurstOff = 0U;
    }

    if ((g_stDabIsrState.u16BurstOff != 0U) ||
        ((u16IsCharge != 0U) && ((g_stDabIsrState.u16ChgBuckOn == 0U) || (f32RefLo >= 0.0f))))
    {
        goto resetOutput;
    }

    if (u16IsCharge == 0U)
    {
        /* 两桥共用模型前馈，按单桥给定计算。 */
        const float f32Dff = Dab_Ctrl_DisFfCalc(DAB_CURR_PER_BRIDGE * Bus.f32Ref);

        Dab_Ctrl_DisCurrLoop(&stDabPi0, &stDabDuty0, f32I0, f32Dff);
        Dab_Ctrl_DisCurrLoop(&stDabPi1, &stDabDuty1, f32I1, f32Dff);
    }
    else
    {
        /* 能走到这里：未休眠，且 Ref <= 0（Ref > 0 上面已封波）。
         * 【公式】Iref = min( max(−0.5·Ref, 0.02), −0.5·RefLo )
         *   Ref < 0 且 |0.5·Ref| < 0.02 → 抬到 0.02A/桥（小电流保活）；
         *   Ref == 0                     → max(0, 0.02)=0.02A/桥（零给定保活，属有意设计）；
         *   任务限流 RefLo 更紧时外层 min 按上限放行（安全优先于保活）。 */
        stChgCtx.f32Iref  = __fmin(__fmax(-DAB_CURR_PER_BRIDGE * Bus.f32Ref, DAB_CHARGE_I_KEEP_MIN),
                                   -DAB_CURR_PER_BRIDGE * f32RefLo);
        stChgCtx.f32Vslow = f32Vslow;
        stChgCtx.f32InvDv = __divf32(1.0f, __fmax(f32Dv, 1.0f));

        /* SR 取较低母线电压，两桥共用。 */
        stDabParam.f32SrRatio = __divf32(__fmin(f32Vslow, f32VbusRaw), f32E);

        Dab_Ctrl_ChgCurrLoop(&stChgCtx, &stDabPi0, &stDabDuty0, f32I0);
        Dab_Ctrl_ChgCurrLoop(&stChgCtx, &stDabPi1, &stDabDuty1, f32I1);
    }
    return;

resetOutput:
    Dab_Ctrl_OutputReset();
}

/*==============================================================================
 * ⑤ 发波：把占空比写进 ePWM
 *
 * 拓扑（每桥 4 个 ePWM 模块，双桥镜像 a/b）：
 *   ePWM1: 1A=S1 1B=S2（HV 左臂）  ePWM2: 2A=S3 2B=S4（HV 右臂）
 *   ePWM3: 3A=S5 3B=S6（LV 左臂）  ePWM4: 4A=S7 4B=S8（LV 右臂）
 *
 * up-down 计数，有效脉宽 ≈ CMP / 2 / TBPRD：CMP = 2·d·TBPRD（四舍五入），
 * 负半周镜像 = TBPRD - CMP。正半周（ZRO 开 / 上数比较关），负半周（PRD 开 / 下数比较关）。
 *
 * 只改 CMP 与开通沿 AQ（SET ↔ NO_ACTION）；关断沿 CLEAR 在初始化时已配好。
 * 动作值只在翻转时写，稳态绝大多数拍不碰 AQ；常量写（RLDCSF / AQCSFRC）
 * 只在 OFF→ON 那一拍做，OSHTLD 每拍必武装。
 *============================================================================*/

/* 只在 OFF→ON 时改强制配置，所有影子写完后再武装本桥 GLD。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Pwm_Commit);
static inline void Dab_Pwm_Commit(volatile struct EPWM_REGS *pstP1,
                                  volatile struct EPWM_REGS *pstP2,
                                  volatile struct EPWM_REGS *pstP3,
                                  volatile struct EPWM_REGS *pstP4,
                                  const volatile DabAqLast_t *pstAqLast,
                                  Uint16 u16HvForce)
{
    if (pstAqLast->bit.u16Phase == 0U)
    {
        pstP1->AQSFRC.bit.RLDCSF = 0U;
        pstP2->AQSFRC.bit.RLDCSF = 0U;
        pstP3->AQSFRC.bit.RLDCSF = 0U;
        pstP4->AQSFRC.bit.RLDCSF = 0U;
        pstP1->AQCSFRC.all = u16HvForce;
        pstP2->AQCSFRC.all = u16HvForce;
        pstP3->AQCSFRC.all = 0U;
        pstP4->AQCSFRC.all = 0U;
    }

    /* AQCSFRC 参与 GLD：本桥影子全部写完后才武装，防止同桥分拍释放。
     * 离装载事件不足 3 TBCLK 时，由硬件推迟到下一次装载。 */
    EALLOW;
    pstP1->GLDCTL2.bit.OSHTLD = 1U;
    EDIS;
}

/* 充电发波：S1~S4 为固定前沿 + 移相后沿，S5~S8 跟随 SR。
 * 返回 1 表示本桥实际在发波（供均流读取）。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Pwm_ChgUpdate);
static inline Uint16 Dab_Pwm_ChgUpdate(volatile struct EPWM_REGS *pstP1,
                                       volatile struct EPWM_REGS *pstP2,
                                       volatile struct EPWM_REGS *pstP3,
                                       volatile struct EPWM_REGS *pstP4,
                                        float f32S2LowDuty, float f32S3SrDuty,
                                       volatile DabAqLast_t *pstAqLast)
{
    Uint16 u16S1TopCmpTmp;
    Uint16 u16S2LowNegCmpTmp;
    Uint16 u16S3TopNegCmpTmp;
    Uint16 u16S4LowCmpTmp;
    Uint16 u16S5S8SrCmpTmp;
    Uint16 u16S6S7SrNegCmpTmp;
    Uint16 u16S1S4ActTmp;
    Uint16 u16S5S8ActTmp;
    Uint16 u16ActiveTmp;

    f32S2LowDuty = DAB_SAT(f32S2LowDuty, 0.0f, DAB_CHG_DUTY_LV_MAX);
    f32S3SrDuty  = DAB_SAT(f32S3SrDuty, 0.0f, DAB_CHG_DUTY_SR_MAX);

    u16S1TopCmpTmp     = (Uint16)(2.0f * DAB_CHG_D_TOP * (float)DAB_CHG_TBPRD + 0.5f);
    u16S4LowCmpTmp     = (Uint16)(2.0f * f32S2LowDuty * (float)DAB_CHG_TBPRD + 0.5f);
    u16S5S8SrCmpTmp    = (Uint16)(2.0f * f32S3SrDuty  * (float)DAB_CHG_TBPRD + 0.5f);
    u16S2LowNegCmpTmp  = (Uint16)(DAB_CHG_TBPRD - u16S4LowCmpTmp);
    u16S3TopNegCmpTmp  = (Uint16)(DAB_CHG_TBPRD - u16S1TopCmpTmp);
    u16S6S7SrNegCmpTmp = (Uint16)(DAB_CHG_TBPRD - u16S5S8SrCmpTmp);

    /* 脉宽过窄（d < DAB_DUTY_MIN）不发波：S1~S4 两路都够宽才开，S5~S8 跟随 */
    u16S1S4ActTmp = ((DAB_CHG_D_TOP >= DAB_DUTY_MIN) && (f32S2LowDuty >= DAB_DUTY_MIN)) ?
                    AQ_SET : AQ_NO_ACTION;
    u16ActiveTmp  = (u16S1S4ActTmp == AQ_SET) ? 1U : 0U;
    u16S5S8ActTmp = ((u16S1S4ActTmp == AQ_SET) && (f32S3SrDuty >= DAB_DUTY_MIN)) ?
                    AQ_SET : AQ_NO_ACTION;

    pstP1->CMPA.bit.CMPA = u16S1TopCmpTmp;     /* S1 */
    pstP2->CMPA.bit.CMPA = u16S3TopNegCmpTmp;  /* S3 */
    pstP2->CMPB.bit.CMPB = u16S4LowCmpTmp;     /* S4 */
    pstP1->CMPB.bit.CMPB = u16S2LowNegCmpTmp;  /* S2 */
    pstP3->CMPA.bit.CMPA = u16S5S8SrCmpTmp;    /* S5 */
    pstP4->CMPB.bit.CMPB = u16S5S8SrCmpTmp;    /* S8 */
    pstP3->CMPB.bit.CMPB = u16S6S7SrNegCmpTmp; /* S6 */
    pstP4->CMPA.bit.CMPA = u16S6S7SrNegCmpTmp; /* S7 */

    /* 每组动作值只跟自己上次写入的值比：相同就不写，翻转才重写这一组 4 个 AQ 位 */
    if (u16S1S4ActTmp != pstAqLast->bit.u16S1S4Act)
    {
        pstP1->AQCTLA.bit.ZRO = u16S1S4ActTmp; /* S1 */
        pstP2->AQCTLB.bit.ZRO = u16S1S4ActTmp; /* S4 */
        pstP1->AQCTLB.bit.PRD = u16S1S4ActTmp; /* S2 */
        pstP2->AQCTLA.bit.PRD = u16S1S4ActTmp; /* S3 */
        pstAqLast->bit.u16S1S4Act = u16S1S4ActTmp;
    }

    if (u16S5S8ActTmp != pstAqLast->bit.u16S5S8Act)
    {
        pstP3->AQCTLA.bit.ZRO = u16S5S8ActTmp; /* S5 */
        pstP4->AQCTLB.bit.ZRO = u16S5S8ActTmp; /* S8 */
        pstP3->AQCTLB.bit.PRD = u16S5S8ActTmp; /* S6 */
        pstP4->AQCTLA.bit.PRD = u16S5S8ActTmp; /* S7 */
        pstAqLast->bit.u16S5S8Act = u16S5S8ActTmp;
    }

    Dab_Pwm_Commit(pstP1, pstP2, pstP3, pstP4, pstAqLast, 0U);

    return u16ActiveTmp;
}

/* 放电发波：HV 两臂占空比可不对称（移相调功），LV 接近 50% 方波。
 *   d_hv ≤ 50%：c4 = ticks，c2 = TBPRD - ticks，内部比较点用 CLEAR 关断；
 *   d_hv > 50%：内部动作改 NO_ACTION，脉冲连在一起，等效占空比 > 50%。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Pwm_DisUpdate);
static inline void Dab_Pwm_DisUpdate(volatile struct EPWM_REGS *pstP1,
                                     volatile struct EPWM_REGS *pstP2,
                                     volatile struct EPWM_REGS *pstP3,
                                     volatile struct EPWM_REGS *pstP4,
                                     float f32HvDuty, float f32LvDuty,
                                     volatile DabAqLast_t *pstAqLast)
{
    Uint16 u16DutyTicksTmp;
    Uint16 u16S2HvNegCmpTmp;
    Uint16 u16S4HvCmpTmp;
    Uint16 u16S5S8LvCmpTmp;
    Uint16 u16S6S7LvNegCmpTmp;
    Uint16 u16S1S4HvActTmp;
    Uint16 u16S5S8LvActTmp;
    Uint16 u16InnerActTmp;      /* 内部比较点动作：CLEAR 正常 / NO_ACTION 超调 */
    Uint16 u16BoundaryActTmp;   /* PRD / ZRO 边界动作 */

    /* 硬限幅：杜绝负数转 Uint16（未定义行为）和 TBPRD-CMP 下溢 */
    f32HvDuty = DAB_SAT(f32HvDuty, 0.0f, DAB_HV_DUTY_HARD_MAX);
    f32LvDuty = DAB_SAT(f32LvDuty, 0.0f, DAB_LV_DUTY_HARD_MAX);

    u16DutyTicksTmp    = (Uint16)(2.0f * f32HvDuty * (float)DAB_DIS_TBPRD + 0.5f);
    u16S5S8LvCmpTmp    = (Uint16)(2.0f * f32LvDuty * (float)DAB_DIS_TBPRD + 0.5f);
    u16S6S7LvNegCmpTmp = (Uint16)(DAB_DIS_TBPRD - u16S5S8LvCmpTmp);

    u16S1S4HvActTmp = (f32HvDuty >= DAB_DUTY_MIN) ? AQ_SET : AQ_NO_ACTION;
    u16S5S8LvActTmp = ((u16S1S4HvActTmp == AQ_SET) && (f32LvDuty >= DAB_DUTY_MIN)) ?
                      AQ_SET : AQ_NO_ACTION;

    if (u16DutyTicksTmp <= DAB_DIS_TBPRD)
    {
        u16S4HvCmpTmp     = u16DutyTicksTmp;
        u16InnerActTmp    = AQ_CLEAR;
    }
    else
    {
        u16S4HvCmpTmp     = (Uint16)(2U * DAB_DIS_TBPRD - u16DutyTicksTmp);
        u16InnerActTmp    = AQ_NO_ACTION;
    }
    u16S2HvNegCmpTmp = (Uint16)(DAB_DIS_TBPRD - u16S4HvCmpTmp);

    /* 打嗝时边界强写 CLEAR，防止残留窄脉冲 */
    u16BoundaryActTmp = (u16S1S4HvActTmp == AQ_NO_ACTION) ? AQ_CLEAR : u16InnerActTmp;

    /* 三组动作值都没变就一次 AQ 都不碰（稳态绝大多数拍）；
     * 任一变则整块重写，并记下本次写入的三个值。 */
    if ((u16S1S4HvActTmp != pstAqLast->bit.u16S1S4Act) ||
        (u16S5S8LvActTmp != pstAqLast->bit.u16S5S8Act) ||
        (u16InnerActTmp  != pstAqLast->bit.u16InnerAct))
    {
        pstP2->AQCTLB.bit.CBU = u16InnerActTmp;      /* S4 上数比较关断 */
        pstP2->AQCTLB.bit.PRD = u16BoundaryActTmp;   /* S4 PRD 边界 */
        pstP2->AQCTLB.bit.ZRO = u16S1S4HvActTmp;     /* S4 ZRO 开通 */

        pstP1->AQCTLB.bit.CBD = u16InnerActTmp;      /* S2 下数比较关断 */
        pstP1->AQCTLB.bit.ZRO = u16BoundaryActTmp;   /* S2 ZRO 边界 */
        pstP1->AQCTLB.bit.PRD = u16S1S4HvActTmp;     /* S2 PRD 开通 */

        /* 另一路比较点仅在正常区间写；超调区“保持不写”≠“写 NO_ACTION” */
        if (u16InnerActTmp == AQ_CLEAR)
        {
            pstP2->AQCTLB.bit.CBD = AQ_CLEAR;
            pstP1->AQCTLB.bit.CBU = AQ_CLEAR;
        }

        pstP3->AQCTLA.bit.ZRO = u16S5S8LvActTmp;     /* S5 */
        pstP4->AQCTLB.bit.ZRO = u16S5S8LvActTmp;     /* S8 */
        pstP3->AQCTLB.bit.PRD = u16S5S8LvActTmp;     /* S6 */
        pstP4->AQCTLA.bit.PRD = u16S5S8LvActTmp;     /* S7 */

        pstAqLast->bit.u16S1S4Act  = u16S1S4HvActTmp;
        pstAqLast->bit.u16S5S8Act  = u16S5S8LvActTmp;
        pstAqLast->bit.u16InnerAct = u16InnerActTmp;
    }

    pstP1->CMPB.bit.CMPB = u16S2HvNegCmpTmp;
    pstP2->CMPB.bit.CMPB = u16S4HvCmpTmp;
    pstP3->CMPA.bit.CMPA = u16S5S8LvCmpTmp;
    pstP4->CMPB.bit.CMPB = u16S5S8LvCmpTmp;
    pstP3->CMPB.bit.CMPB = u16S6S7LvNegCmpTmp;
    pstP4->CMPA.bit.CMPA = u16S6S7LvNegCmpTmp;

    /* 放电时 S1/S3 保持强制低。 */
    Dab_Pwm_Commit(pstP1, pstP2, pstP3, pstP4, pstAqLast, 0x0001U);
}

/*==============================================================================
 * ⑥ 中断：采样 → 快保护 → 控制 → 发波 → 应答
 *============================================================================*/

/* 封波：清任务侧可见的发波标记，硬件强制低，动作值缓存作废。
 * 注意：不清 f32ChgLimitA —— 它由充电包络唯一拥有，封波期间仍每拍更新；
 * 在这里清会让解封后从 0 重爬，且两个主人互相覆盖。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Pwm_Seal);
static inline void Dab_Pwm_Seal(void)
{
    g_stDabIsrState.u16ChgPwmActive = 0U;
    DAB_PWM_FORCE_LOW();
    Dab_Pwm_AqCacheReset();     /* 与强制低同步：相位归 0、动作值作废 */
}

/* 本拍控制 + 发波（封波门 / 方向门 / 发波都在这里，EPWM10_ISR 只剩采样与应答）
 *
 * 门顺序不能动：
 *   ① 故障 / 升级中 / 非闭环态 → 强制低，直接返回（不跑控制）；
 *      升级标志与 1ms 软停、2ms OST 互补：标志有效期间每拍 seal，
 *      避免 OST 锁存前发波口重写 AQCSFRC 顶掉软强制；
 *   ② openTest 开环测试 → 跳过内外环与方向门，直接按给定占空比发波；
 *   ③ 方向封波命中 → 先跑外环（100µs 判向要用 Bus.f32Ref），再清内环、强制低；
 *   ④ 否则正常发波。方向门必须在 Dab_Pwm_*Update 之前：后者会清 AQCSFRC 并置
 *      OSHTLD，若先发波再强制低会被下一拍的加载顶掉。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_fun);
static inline void Dab_Ctrl_fun(Uint16 u16IsCharge)
{
    const Uint16 u16State = (Uint16)RunState.state;
    Uint16 u16OpenTest;
    float  f32D0;
    float  f32D1;

    /* ① 故障 / 升级中 / 非闭环态 → 强制低，直接返回 */
    if (FAULT_ANY() || CfgParam.SysCmd.bit.faultcmd ||
        (stBootProto.u16FlashUpdateFlag != 0U) ||
        ((u16State != (Uint16)SM_STATE_RAMP_CLOSE) && (u16State != (Uint16)SM_STATE_RUN)))
    {
        goto seal;
    }

    /* ② 开环测试 vs ③ 闭环控制 + 方向封波门 */
    u16OpenTest = CfgParam.SysCmd.bit.openTest;
    if (u16OpenTest != 0U)
    {
        f32D0 = CfgParam.OpenLoopDuty * 0.001f;             /* 0.1% → 0~1 */
        f32D1 = f32D0;
        g_stDabIsrState.f32ChgLimitA = 0.0f;
        if (u16IsCharge != 0U)
        {
            stDabDuty0.f32SrDuty = f32D0 * 1.1f;            /* SR 略宽于主占空比 */
            stDabDuty1.f32SrDuty = stDabDuty0.f32SrDuty;
        }
    }
    else
    {
        Uint16 u16Mode    = (Uint16)RunState.workMode;
        Uint16 u16SealBit = (u16Mode == (Uint16)WORK_MODE_CHARGE)    ? SM_SEAL_NO_CHARGE    :
                            (u16Mode == (Uint16)WORK_MODE_DISCHARGE) ? SM_SEAL_NO_DISCHARGE : 0U;
        Uint16 u16HoldOff = ((RunState.curLowSealing & u16SealBit) != 0U) ? 1U : 0U;

        Dab_Ctrl_Loop(u16IsCharge, u16HoldOff);

        if (u16HoldOff != 0U)
        {
            /* 封波只削掉被封方向的积分和输出，反方向保留；f32Ref 同向钳位供 100µs 判向 */
            if (u16Mode == (Uint16)WORK_MODE_DISCHARGE)
            {
                Bus.f32Integ = __fmin(Bus.f32Integ, 0.0f);
                Bus.f32Out   = __fmin(Bus.f32Out,   0.0f);
                Bus.f32Ref   = __fmin(Bus.f32Ref,   0.0f);
            }
            else
            {
                Bus.f32Integ = __fmax(Bus.f32Integ, 0.0f);
                Bus.f32Out   = __fmax(Bus.f32Out,   0.0f);
                Bus.f32Ref   = __fmax(Bus.f32Ref,   0.0f);
            }

            goto seal;
        }

        f32D0 = stDabDuty0.f32Duty;
        f32D1 = stDabDuty1.f32Duty;
    }

    /* ④ 开环与闭环共用发波出口。 */
    if (u16IsCharge != 0U)
    {
        Uint16 u16ActiveTmp;

        u16ActiveTmp  = Dab_Pwm_ChgUpdate(&DabEpwm1Regs, &DabEpwm2Regs, &DabEpwm3Regs, &DabEpwm4Regs,
                                          f32D0, stDabDuty0.f32SrDuty,
                                          &g_stDabIsrState.stChgAqBr1);
        u16ActiveTmp |= Dab_Pwm_ChgUpdate(&DabEpwm1bRegs, &DabEpwm2bRegs, &DabEpwm3bRegs, &DabEpwm4bRegs,
                                          f32D1, stDabDuty1.f32SrDuty,
                                          &g_stDabIsrState.stChgAqBr2);
        if (u16OpenTest == 0U)
        {
            g_stDabIsrState.u16ChgPwmActive = u16ActiveTmp;
        }
    }
    else
    {
        float f32Hv;
        float f32Lv;

        if (u16OpenTest == 0U)
        {
            g_stDabIsrState.u16ChgPwmActive = 0U;           /* 放电时清充电发波标记 */
            g_stDabIsrState.f32ChgLimitA    = 0.0f;
            g_stDabIsrState.f32CapFilt      = 0.0f;
        }

        Dab_Ctrl_DisDutyMap(f32D0, &f32Hv, &f32Lv);
        Dab_Pwm_DisUpdate(&DabEpwm1Regs, &DabEpwm2Regs, &DabEpwm3Regs, &DabEpwm4Regs,
                          f32Hv, f32Lv, &g_stDabIsrState.stChgAqBr1);
        if (u16OpenTest == 0U)
        {
            Dab_Ctrl_DisDutyMap(f32D1, &f32Hv, &f32Lv);
        }
        Dab_Pwm_DisUpdate(&DabEpwm1bRegs, &DabEpwm2bRegs, &DabEpwm3bRegs, &DabEpwm4bRegs,
                          f32Hv, f32Lv, &g_stDabIsrState.stChgAqBr2);
    }

    /* 两桥常量均已按本拍相位写完/确认，置 1 供下一拍跳过常量写 */
    g_stDabIsrState.stChgAqBr1.bit.u16Phase = 1U;
    g_stDabIsrState.stChgAqBr2.bit.u16Phase = 1U;
    return;

seal:
    Dab_Pwm_Seal();
}

/*==============================================================================
 * 软件示波器采样（经典 scopeGet 结构，与 sci_ex1 参考工程一致）
 *   未触发：前 PRE_TRIGGER_NUM(64) 点环形覆写；
 *   触发  ：从第 64 点起线性存满 SCOPENUM(900) 后锁定，等 CAN 传完重新布防。
 *   触发源：母线过压 / 通道过流 / 给定方向异常；手动触发由 0x20 索引 8 置位
 *           uTran_CMD（与故障触发同一路径）。
 *   放 RAM 执行（与 EPWM10_ISR 同段）；只存数与比较，f32I0/f32I1 复用 ISR 局部量。
 *============================================================================*/
#pragma CODE_SECTION(DabScope_Sample, ".TI.ramfunc");   /* 必须写在函数定义之前 */
static void DabScope_Sample(float f32I0, float f32I1)
{
    const Uint16 u16Triggered = DataScope.uTran_CMD;
    Uint16 save_idx;

    /* 未触发时检查过压、过流，否则只更新预触发环。 */
    if ((u16Triggered == 0U) &&
        !((stADC.VmBusVolt.f32IsrResult > DAB_DISCHARGE_BUS_OVP_VOLT) ||
          (fabsf(f32I0) > FAULT_CH_OC) || (fabsf(f32I1) > FAULT_CH_OC)))
    {
        save_idx = DataScope.uPreTriggerIdx;
        DataScope.uPreTriggerIdx = (Uint16)((DataScope.uPreTriggerIdx + 1U) & PRE_TRIGGER_MASK);
    }
    else
    {
        /* 自动触发从首个后触发槽开始；手动触发也不得覆盖预触发环。 */
        if (u16Triggered == 0U)
        {
            DataScope.uTran_CMD = 1U;
            DataScope.uCurrent_Save_subscript = PRE_TRIGGER_NUM;
        }
        else if (DataScope.uCurrent_Save_subscript < (Uint16)PRE_TRIGGER_NUM)
        {
            DataScope.uCurrent_Save_subscript = (Uint16)PRE_TRIGGER_NUM;
        }

        if (DataScope.uCurrent_Save_subscript >= (Uint16)SCOPENUM)
        {
            return;
        }
        save_idx = DataScope.uCurrent_Save_subscript++;
    }

    /* 四通道统一放大 10 倍存储。 */
    DataScope.uDataScope0[save_idx] = (int16)(stADC.BusCurr1.f32IsrResult  * 10.0f);
    DataScope.uDataScope1[save_idx] = (int16)(stADC.BusCurr2.f32IsrResult   * 10.0f);
    DataScope.uDataScope2[save_idx] = (int16)(stADC.BusVolt1.f32IsrResult  * 10.0f);
    DataScope.uDataScope3[save_idx] = (int16)(stADC.BusVolt2.f32IsrResult      * 10.0f);
}

#pragma CODE_SECTION(EPWM10_ISR, ".TI.ramfunc"); /* 放 RAM，去 Flash 等待周期 */
interrupt void EPWM10_ISR(void)
{
    float f32Vbus;       /* 母线瞬时电压（继电器 VM 侧） */
    float f32Vbat;       /* 电池电压瞬时值（只给快保护） */
    float f32I0;         /* 电流采样 0 */
    float f32I1;         /* 电流采样 1 */
    Uint16 u16IsCharge;  /* 工况快照：本拍内所有环节共用 */

    /* ===== 采样：ADRESULT_* ↔ Bsp SOC；IDD1/IDD2 为各 ADC 首采；双 SOCA@PRD→ZERO 读 ===== */
    f32Vbus = (float)ADRESULT_VMBUS_ADC * BUS_VOLT_RATIO;   /* ADCA:A12 */
    f32Vbat = (float)ADRESULT_VDC_ADC * LOAD_VOLT_RATIO;    /* ADCC:C10 */
    f32I0   = ((float)ADRESULT_IDD1_ADC - stADC.CurrOffset.f32Idd1) * BUS_CURR_RATIO; /* ADCA SOC0 */
    f32I1   = ((float)ADRESULT_IDD2_ADC - stADC.CurrOffset.f32Idd2) * BUS_CURR_RATIO; /* ADCB SOC0 B8 */
    u16IsCharge = stDabParam.u16IsCharge;

    stADC.BusVolt1.f32IsrResult   = (float)ADRESULT_VBUS1_ADC * BUS_VOLT_RATIO;
    stADC.BusVolt2.f32IsrResult   = (float)ADRESULT_VBUS2_ADC * BUS_VOLT_RATIO;
    stADC.VmBusVolt.f32IsrResult  = f32Vbus;
    stADC.PcsBusVolt.f32IsrResult = (float)ADRESULT_PCS_VBUS * BUS_VOLT_RATIO;  /* B4 PCS 端 */
    stADC.BatVolt.f32IsrResult    = f32Vbat;
    stADC.BusCurr1.f32IsrResult   = f32I0;
    stADC.BusCurr2.f32IsrResult   = f32I1;
    stADC.BatCurr.f32IsrResult    = (f32I0 + f32I1) * f32Vbus * g_stDabIsrState.f32InvBatVolt;

    /* ① 快保护：只判断置位（母线过压取继电器 VM 瞬时值 f32Vbus，桥侧 VBUS1/VBUS2 不参与） */
    Fault_FastProtect(f32I0, f32I1, f32Vbus);

    /* 加热膜 1kHz 软 PWM（仅翻转） */
    HeatFilm_Isr(u16IsCharge);

    /* 外环相位 0，陷波相位 4；分频随已装载的 PWM 周期走 */
    if (++Bus.u16Tick >= g_u16DabPwmTicks100us)
    {
        Bus.u16Tick = 0U;
    }
    if (Bus.u16Tick == 4U)
    {
        Notch2f_Isr();
    }

    /* 软件示波器：64 点预触发环 + 触发后线性存满 900 点（只存数与比较） */
    DabScope_Sample(f32I0, f32I1);

    /* ② 控制 + 发波（含封波门、方向门） */
    Dab_Ctrl_fun(u16IsCharge);

    /* ③ 应答 */
    DabEpwm1Regs.ETCLR.bit.INT = 1U;          /* EPWM10 ZERO */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
