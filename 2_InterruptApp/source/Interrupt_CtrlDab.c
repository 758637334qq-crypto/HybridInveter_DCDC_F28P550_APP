/*************************************************************************************************
 * Interrupt_CtrlDab.c — DAB 实时控制（EPWM10_ISR：充电 50kHz / 放电 60kHz）
 *
 * 模块划分（自下而上）：
 *   ① 外环     Dab_Ctrl_BusVoltLoop                 电压环 → 总电流给定 Bus.f32Ref
 *   ② 放电内环 Dab_Ctrl_DisFfCalc / DisGainSched / DisCurrLoop / DisDutyMap
 *   ③ 充电内环 Dab_Ctrl_SrStateUpdate / SrDutyCalc / ChgCurrLoop
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
 * ① 电压外环（两路共用，10kHz）
 *
 * 输入：母线电压 / 负载电流 / 本模式给定下限 / 工况
 * 输出：Bus.f32Ref 总电流给定（正 = 放电，负 = 充电）
 *============================================================================*/
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_BusVoltLoop);
static inline void Dab_Ctrl_BusVoltLoop(float f32Vo, float f32Iload, float f32RefMin, Uint16 u16IsCharge)
{
    float f32Ff;        /* 前馈：充电 0，放电 0.9 × 滤波负载电流 */
    float f32OutMin;    /* 本拍 PI 状态下限 */
    float f32IPos;      /* 本拍放电上限 */

    f32IPos = Ramp.iPos;

    /* 滤波与前馈（每载波拍执行） */
    Bus.f32IloadFf += 0.20f * (f32Iload - Bus.f32IloadFf);
    Bus.f32Err     += DAB_BUS_ALPHA * ((Bus.f32Vref - f32Vo) - Bus.f32Err);
    f32Ff           = u16IsCharge ? 0.0f : BUS_FF_COEF * Bus.f32IloadFf;
    Bus.f32VoltFf   = f32Ff;

    f32OutMin = f32RefMin - f32Ff;
    if ((u16IsCharge != 0U) && (g_stDabIsrState.u16ChgBuckOn != 0U))
    {
        /* 充电为负。只允许旧请求占用余量，新请求最多走到本拍限值。 */
        f32OutMin = __fmin(Bus.f32Out, f32RefMin);
        f32OutMin = __fmax(f32OutMin, f32RefMin - DAB_CHARGE_REF_HOLD_A);
        f32OutMin = __fmax(f32OutMin, Ramp.iNeg);
    }

    /* 外环分频执行：Bus.u16Tick == 0 时为 10kHz */
    if (Bus.u16Tick == 0U)
    {
        float  f32Err;
        float  f32T;
        float  f32Kp;
        float  f32Ki;
        float  f32DeltaPi;
        Uint16 u16DutySat;

        /* 任一桥上一拍顶到上限即饱和。标志由内环在各自夹位点精确置位，
         * 判据与内环实际限幅同源，动态上限生效时也不会漏判。 */
        u16DutySat = (stDabPi0.u16DutySat != 0U) || (stDabPi1.u16DutySat != 0U);

        f32Err = DAB_SAT(Bus.f32Err, -40.0f, 40.0f);   /* 误差限幅 ±40V */

        /* 变增益：|e| 在 [E_LO, E_HI] 线性插值，区间外保持端点 */
        f32T  = DAB_SAT((fabsf(f32Err) - DAB_BUS_GAIN_E_LO) * DAB_BUS_GAIN_INV_SPAN, 0.0f, 1.0f);
        f32Kp = DAB_BUS_KP_LO + DAB_BUS_KP_SPAN * f32T;
        f32Ki = DAB_BUS_KI_LO + DAB_BUS_KI_SPAN * f32T;

        /* 增量式 PI：始终全量（已无 e·de 分支） */
        f32DeltaPi = f32Kp * (f32Err - Bus.f32Err1) + f32Ki * f32Err;

        /* 抗饱和：未饱和正常积分；饱和时只允许朝放开饱和的方向积分 */
        if ((u16DutySat == 0U) || (u16IsCharge ? (f32DeltaPi > 0.0f) : (f32DeltaPi < 0.0f)))
        {
            Bus.f32Out += f32DeltaPi;
        }

        /* 动态限幅（下限优先）：先保证 Out ≥ OutMin，再用 iPos - Ff 封顶。
         * iPos = 0（放电被禁）时充电的正向余量也为 0，与禁向设计一致。 */
        Bus.f32Out = __fmin(__fmax(Bus.f32Out, f32OutMin), f32IPos - f32Ff);

        Bus.f32Err1 = f32Err;
    }

    /* 校正量单边限幅（每载波拍执行） */
    if (u16IsCharge)
    {
        Bus.f32Out = __fmin(Bus.f32Out, BUS_CHG_OUT_MAX);       /* 充电：Out ≤ +1A */
    }
    else
    {
        Bus.f32Out = __fmax(Bus.f32Out, BUS_DISCH_OUT_MIN);     /* 放电：Out ≥ -1A */
    }

    /* 每拍校正 PI 状态下限：能力收紧也会发生在两次外环更新之间。
     * 与上方 -1A 校正下限并非冗余：Ff 变号时两者轮流更紧。 */
    Bus.f32Out = __fmax(Bus.f32Out, f32OutMin);

    /* 叠前馈，用本模式限流封顶 */
    if (u16IsCharge)
    {
        Bus.f32Ref = __fmax(Bus.f32Out + f32Ff, f32RefMin); /* 充电：用整机限流幅值的负值封底 */
    }
    else
    {
        Bus.f32Ref = __fmin(Bus.f32Out + f32Ff, f32IPos);   /* 放电：封顶 iPos（正值） */
    }
}

/*==============================================================================
 * ② 放电内环
 *============================================================================*/

/* ▲ 放电模型前馈：按单桥目标电流 f32Iref 反解归一化占空比 u ∈ [0,1]
 *   1) 算归一化电流 j = K·Iref / (n·Vbat)；
 *   2) 升压区 (r ≥ 1)：低压锁 0.44，轻载用精确开方、重载用切线求高压交叠量 q；
 *   3) 降压区 (r < 1)：轻载开方求低压脉宽 b（受 bmax 限幅）并反查 u，末尾 2% 平滑桥接，
 *      超过 j0 后再按线性叠加高压交叠量 q；
 *   4) q 在物理最大功率点封顶，再反算回 u（与 Dab_Ctrl_DisDutyMap 互逆）。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_DisFfCalc);
static inline float Dab_Ctrl_DisFfCalc(float f32Iref)
{
    float f32InvE = g_stDabIsrState.f32DisInvE;
    float f32R    = g_stDabIsrState.f32DisRatio;
    float f32J;
    float f32Q;
    float f32U;

    /* 电池欠压或给定极小时直接输出 0 */
    if ((f32InvE <= 0.0f) || (f32Iref < DAB_DIS_FF_IREF_MIN))
    {
        return 0.0f;
    }

    f32J = DAB_DIS_FF_K * f32Iref * f32InvE;

    if (f32R >= 1.0f)
    {
        /* ▲ 升压区：Vset ≥ n·Vbat */
        float f32Dr = f32R - 1.0f;
        float f32J1 = __divf32(DAB_DIS_FF_B2 * f32Dr, f32R * f32R);

        if (f32J <= f32J1)
        {
            f32Q = __sqrt(f32J * f32Dr);                                                    /* 轻中载 DCM：精确开方 */
        }
        else
        {
            f32Q = __divf32(DAB_DIS_FF_B * f32Dr, 2.0f * f32R) + f32R * f32J * DAB_DIS_FF_INV_2B; /* 重载：一阶切线 */
        }
    }
    else
    {
        /* ▲ 降压区：Vset < n·Vbat */
        float f32J0 = __divf32(2.0f * (1.0f - f32R) * DAB_DIS_FF_B2, 1.0f + f32R);

        if (f32J <= f32J0)
        {
            float f32K    = __divf32(f32J, f32J0);                                  /* 当前电流占 j0 的比例 (0~1] */
            float f32BMax = __fmin(DAB_DIS_FF_B, 0.25f * (1.0f + f32R) - 0.025f);   /* ▲ 保留原模型随电压比收紧的 bmax */
            float f32B    = __fmin(DAB_DIS_FF_B * __sqrt(f32K), f32BMax);
            float f32T    = __fmax((f32K - 0.98f) * 50.0f, 0.0f);                   /* ▲ 仅末尾 2% 线性桥接到 u_q0，消除跳变 */

            f32U = (f32B <= 0.30f) ? ((f32B - DAB_DUTY_LV_MIN) * DAB_LV_INV_SLOPE1)
                                   : (DAB_U_MID1 + (f32B - 0.30f) * DAB_LV_INV_SLOPE2);
            f32U += f32T * (DAB_DIS_FF_U_Q0 - f32U);
            return DAB_SAT(f32U, 0.0f, 1.0f);
        }

        f32Q = __divf32((1.0f + f32R) * (f32J - f32J0), 4.0f * f32R * DAB_DIS_FF_B);
    }

    /* ▲ 限制 q 不越过最大功率点，再反算回 u */
    f32Q = __fmin(f32Q, __divf32(2.0f * f32R * DAB_DIS_FF_B, 1.0f + 2.0f * f32R));
    f32U = DAB_DIS_FF_U_Q0 + f32Q * DAB_HV_INV_SLOPE;

    return DAB_SAT(f32U, 0.0f, 1.0f);
}

/* 按上拍映射输入 u 选增益调度倍率，分段边界与 Dab_Ctrl_DisDutyMap 一致 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_DisGainSched);
static inline float Dab_Ctrl_DisGainSched(float f32DutyU)
{
    if (f32DutyU <= DAB_U_MID1)
    {
        return DAB_SCHED_S5_SCALE_1;
    }
    if (f32DutyU <= DAB_U_MID2)
    {
        return DAB_SCHED_S5_SCALE_2;
    }
    return DAB_SCHED_S4_SCALE_3;
}

/* 放电单路电流环
 *
 * 符号约定：f32Curr 带符号采样，放电时为正；Bus.f32Ref 放电需求为正，
 * f32Ref = 0.5×Ref 恒为正。
 *
 * 增量式 PI 输出归一化 u，再叠前馈得到本桥占空比。
 * 给定撤销 / 甩载 / NaN 由上层打嗝休眠门单拍统一清零，本函数只处理正给定；
 * 恰好为零的给定在前馈软起下系数为 0，PI 增量被映射死区拦截，无积累风险。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_DisCurrLoop);
static inline void Dab_Ctrl_DisCurrLoop(ST_DAB_PI *pstPi, ST_DAB_DUTY *pstDuty,
                                        float f32Current, float f32DffBase)
{
    const float f32OutMax = pstPi->f32OutMax;               /* 缓存，避免重复读内存 */
    const float f32Duty   = pstDuty->f32Duty;               /* 本拍开始时的占空比 */
    const float f32Ref    = DAB_CURR_PER_BRIDGE * Bus.f32Ref;
    float f32Err;
    float f32DeltaPi;
    float f32StepMax;
    float f32Ff;
    float f32OutHi;

    pstPi->f32Ref = f32Ref;

    /* 上坡限步：剩余行程 / 5，最低 DAB_DUTY_STEP_MIN；
     * 下坡用固定步长（甩载不等待），刻意非对称。 */
    f32StepMax = __fmax((f32OutMax - f32Duty) * DAB_DUTY_MAX_STEPS_INV, DAB_DUTY_STEP_MIN);

    /* ▲ 前馈缓入（零新增内存，复用休眠已清零的 f32DutyFf）：
     * 上升每拍最多加 f32StepMax（唤醒首拍最多 0.20，约 5 拍爬满），下降当拍直跟。 */
    f32Ff            = __fmin(f32DffBase, pstPi->f32DutyFf + f32StepMax);
    pstPi->f32DutyFf = f32Ff;

    /* 增量 PI × 调度倍率：段③映射斜率变小、环路增益下降，用斜率反比补回。
     * 只缩放 PI 增量，前馈、映射、限步都不参与。 */
    f32Err     = f32Ref - f32Current;
    f32DeltaPi = pstPi->f32K1 * f32Err - pstPi->f32K2 * pstPi->f32Err1;
    f32DeltaPi *= Dab_Ctrl_DisGainSched(f32Duty);

    if (f32DeltaPi > 0.0f)
    {
        f32DeltaPi = __fmin(f32DeltaPi, f32StepMax);
    }
    else
    {
        f32DeltaPi = __fmax(f32DeltaPi, -DAB_DUTY_STEP_DOWN);
    }

    /* 抗饱和：在占空比上限仍要增、在 0 仍要减时不积分 */
    if (((f32Duty < f32OutMax) || (f32DeltaPi <= 0.0f)) &&
        ((f32Duty > 0.0f) || (f32DeltaPi >= 0.0f)))
    {
        pstPi->f32Out += f32DeltaPi;
    }

    /* 饱和标志在 Out 夹位点直接判定：若先夹到 OutMax-Ff 再回加 Ff，
     * 浮点 1ULP 漂移会让 Duty ≥ OutMax 漏判一拍。 */
    f32OutHi          = f32OutMax - f32Ff;
    pstPi->u16DutySat = (pstPi->f32Out >= f32OutHi) ? 1U : 0U;
    pstPi->f32Out     = DAB_SAT(pstPi->f32Out, -f32Ff, f32OutHi);
    pstPi->f32Err1    = f32Err;

    /* 位置式叠前馈，输出本桥占空比 */
    pstDuty->f32Duty = DAB_SAT(pstPi->f32Out + f32Ff, 0.0f, f32OutMax);
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

    f32Duty = DAB_SAT(f32Duty, 0.0f, 1.0f);

    if (f32Duty < DAB_U_MIN_PACKET)
    {
        *pf32HvDuty = 0.0f;
        *pf32LvDuty = 0.0f;
        return;
    }

    if (f32Duty <= DAB_U_MID2)
    {
        f32HvDutyTmp = DAB_DUTY_HV_BASE;                    /* HV 固定 45% */
        if (f32Duty <= DAB_U_MID1)
        {
            f32LvDutyTmp = DAB_DUTY_LV_MIN + DAB_LV_SLOPE1 * f32Duty;
        }
        else
        {
            f32LvDutyTmp = 0.30f + DAB_LV_SLOPE2 * (f32Duty - DAB_U_MID1);
        }
    }
    else
    {
        f32LvDutyTmp = DAB_DUTY_LV_MAX;                     /* LV 到顶，HV 开始加 */
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

/* SR 占空比 = SrRatio × 主占空比 - 死区，限幅 [0, 0.44]（不得超过低压桥最大值） */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_SrDutyCalc);
static inline float Dab_Ctrl_SrDutyCalc(const volatile ST_DAB_PARAM *pstParam, Uint16 u16Enable, float f32Duty)
{
    if (u16Enable == 0U)
    {
        return 0.0f;
    }
    return DAB_SAT(pstParam->f32SrRatio * f32Duty - DAB_SR_DUTY_MARGIN, 0.0f, DAB_SR_DUTY_MAX);
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
 * 全程在电流域（A）工作，返回本拍电流指令 i_cmd。
 * 因为这里不是buckboost，依然要做成单极性来做控制：
 * ival>iref,这时候要降低占空比，err就要<0；
 * ival<iref,这时候要增加占空比，err就要>0。
 * 变量对应：iref = f32Iref（需求幅值）；ival = (−f32Curr) 取反后的反馈幅值，
 * 即 e = f32Iref − (−f32Curr) = −Ref − Ical（每路口径）。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_ChgPiStep);
static inline float Dab_Ctrl_ChgPiStep(ST_DAB_PI *pstPi, float f32Curr,
                                       const ST_DAB_CHG_CTX *pstCtx)
{
    const float f32Iref    = pstCtx->f32Iref;
    const float f32IcmdMax = pstCtx->f32IcmdMax;
    float f32Err;
    float f32Corr;
    float f32Raw;
    float f32Icmd;

    pstPi->f32Ref = -f32Iref;   /* 系统约定：充电给定为负。外面谈的是电流大小（正数好懂），
                                 * 进环记账换带符号：充电记负账、放电记正账 */
    f32Err  = f32Iref + f32Curr;
    f32Corr = DAB_SAT(pstPi->f32Out + DAB_CHARGE_K1 * f32Err - DAB_CHARGE_K2 * pstPi->f32Err1,
                      -(DAB_CHARGE_CORR_NEG_K * f32Iref + DAB_CHARGE_CORR_ABS),
                      DAB_CHARGE_CORR_POS_K * f32Iref + DAB_CHARGE_CORR_ABS);
    f32Raw  = f32Iref + f32Corr;
    /* 通俗版：最终电流指令夹在 [0, 能力上限]——不许为负（反向功率归放电逻辑管），
     * 不许超能力（超过也顶不出占空比，白搭）。 */
    f32Icmd = DAB_SAT(f32Raw, 0.0f, f32IcmdMax);

    pstPi->f32Out  = f32Corr + 0.5f * (f32Icmd - f32Raw);
    pstPi->f32Err1 = f32Err;

    return f32Icmd;
}

#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_ChgCurrLoop);
static inline void Dab_Ctrl_ChgCurrLoop(const ST_DAB_CHG_CTX *pstCtx, ST_DAB_PI *pstPi,
                                        ST_DAB_DUTY *pstDuty, float f32Curr)
{
    const float f32DutyMax = pstCtx->f32DutyMax;
    float f32Icmd;
    float f32Duty;

    /* ① 电流域 PI 步（误差/对称限幅/指令夹位/back-calc 抗饱和，输出电流指令） */
    f32Icmd = Dab_Ctrl_ChgPiStep(pstPi, f32Curr, pstCtx);

    /* ② 开方逆模型（TMU）：电流指令 → 占空比 */
    f32Duty = __fmin(__sqrt(DAB_CHARGE_KFL * f32Icmd * pstCtx->f32InvDv), f32DutyMax);
    pstDuty->f32Duty = f32Duty;

    /* CAN 观测直接取环内结果：充电侧 f32DutyFf 记录本拍实际占空比，
     * 不再在 Main 里另算一遍标称开方（省一次 TMU 和一份重复代码）。
     * 注：放电侧该字段是真前馈 u_ff，由放电环自己写，语义不受影响。 */
    pstPi->f32DutyFf = f32Duty;

    /* 饱和在夹位发生的域内判：操作数精确相等，避免 sqrt 反算回占空比域出现 1ULP 漂移 */
    pstPi->u16DutySat = ((f32Icmd >= pstCtx->f32IcmdMax) || (f32Duty >= f32DutyMax)) ? 1U : 0U;

    /* ③ SR：占空比为 0 立即撤销；否则按瞬时功率判开通/关断 */
    if (f32Duty <= 0.0f)
    {
        Dab_Ctrl_SrClear(pstDuty);
    }
    else
    {
        float f32InstW = -pstCtx->f32Vslow * f32Curr;   /* 本拍瞬时功率，倒灌时为负 */

        Dab_Ctrl_SrStateUpdate(pstDuty, f32InstW);
        pstDuty->f32SrDuty = Dab_Ctrl_SrDutyCalc(&stDabParam, pstDuty->u16SrEnable, f32Duty)
                             * ((float)pstDuty->u16SrSoft * DAB_SR_SOFT_INV);
    }
}

/*==============================================================================
 * ④ 顶层控制步：工况快照 → 充电包络 → 外环 → 内环
 *
 * 电池电压统一取 10ms 滤波值（已夹在 [DAB_BAT_VOLT_MIN, 65V] 内）；
 * 瞬时值只给 EPWM10_ISR 的快保护用，不进控制计算。
 *============================================================================*/

#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_Loop);
static inline void Dab_Ctrl_Loop(volatile ST_DAB_PARAM *pstParam, float f32VbusRaw,
                                 float f32I0, float f32I1, Uint16 u16IsCharge, Uint16 u16HoldOff)
{
    float f32Vslow;                                     /* 陷波母线电压，外环反馈 */
    float f32Iload;                                     /* 母线总电流，外环前馈 */
    float f32E     = 0.0f;                              /* 充电：N·Vbat */
    float f32Dv    = 0.0f;                              /* 充电：压差 */
    float f32InvDv = 0.0f;                              /* 充电：1/压差 */
    float f32DutyMax = 0.0f;                            /* 充电：动态占空比上限 */
    float f32ChgImax     = 0.0f;                            /* 充电：本压差下两路合计可达电流 */
    float f32IrefMax     = 0.0f;                            /* 充电：单路给定上限，放电不用 */
    float f32Dmd;                                       /* 打嗝：同向需求深度（放电=Ref，充电=-Ref） */
    float f32SealThr;                                   /* 打嗝：反向休眠阈（NaN 落休眠侧） */
    float f32WakeThr;                                   /* 打嗝：同向唤醒阈 */
    Uint16 u16BurstSleep;                               /* 打嗝：休眠门 */
    float f32RefLo;                                     /* 外环总给定下限 */

    /* 电压给定 = 1ms 斜坡基准 + 均流修正 */
    f32Iload    = f32I0 + f32I1;
    Bus.f32Vref = g_CtrlBusRef_V;
    f32Vslow    = stADC.VmBusVolt.f32Notch;

    /* 充电包络：Buck 发波回差（能力域）+ 动态占空比上限。
     * 发波判据用平滑后的物理能力 f32Cap，不用固定电压回差：
     * ΔV 是两个 ADC 通道相减再经匝比放大，量化误差按同比例进入电流域后才有效。 */
    if (u16IsCharge != 0U)
    {//=1充电
        float f32DvPos;     /* 仅用于除法与能力估算的慢压差 */
        float f32VbusPk;    /* 峰值 / ZCS 共用的保守母线电压 */
        float f32CapFilt;   /* 能力缓放值：下降立即，上升限步 */
        float f32ChgLimitA; /* 整机充电限流目标 */

        f32E     = DAB_TURNS_RATIO * stADC.BatVolt.Filt.f32Filt10ms;
        /* 电流口径 = 高压母线输入电流：母线只在电感电流爬升段供电，衰减段能量
         * 来自电感储能、经 SR 流向电池，与母线无关。故母线侧平均电流
         * I = ΔV·d²/(f·L)，不含 Vbus/E；电池侧才是 ×(Vbus/E)，该折算已在
         * 任务层限流(60A·Vbat/Vbus)体现，这里不要重复计入。 */
        f32Dv    = f32Vslow - f32E;
        f32DvPos = __fmax(f32Dv, 1.0f);         /* 1V 下限，避免 ΔV≈0 时除法出 inf */
        f32InvDv = __divf32(1.0f, f32DvPos);

        /* 峰值支路取保守的瞬时压差：纹波波峰才是峰值电流的发生点，
         * max(Vslow,Vraw) ≥ Vslow，只会收紧不会放宽。 */
        f32VbusPk = __fmax(f32Vslow, f32VbusRaw);

        /* 占空比上限三者取小：ZCS 约束、峰值电流约束、任务侧硬上限 */
        f32DutyMax = __fmin(pstParam->f32DutyMax,
                        __fmin(__divf32(DAB_CHARGE_ZCS_MAX * f32E, f32VbusPk),
                            __divf32(DAB_CHARGE_IPK_MAX * DAB_CHARGE_KFL_MIN, __fmax(f32VbusPk - f32E, 1.0f))
                              )
                           );

        /* 两桥合计可达电流（母线侧口径），漏感按标称值：能力是限流依据，应取
         * 每台机器可达的典型值——用 0.8 倍漏感会对 1.2 倍漏感机器虚报能力，
         * 占空比顶死仍达不到给定。压差失效能力为负，由下方回差关断。 */
        f32ChgImax = (float)DAB_BRIDGE_NUM * f32Dv * f32DutyMax * f32DutyMax * DAB_CHARGE_INV_KFL;

        /* 能力缓放：下降当拍跟随，上升每拍限步；Buck 等待期间同样更新。
         * 不复用 f32ChgLimitA 当记忆：它混有任务设定，会被低设定拉低。 */
        f32CapFilt = __fmin(__fmax(f32ChgImax, 0.0f),
                            g_stDabIsrState.f32CapFilt + DAB_CHARGE_CAP_RISE_A);
        g_stDabIsrState.f32CapFilt = f32CapFilt;

        /* 有效限流 = 缓放能力 ∩ 任务请求；任务侧上升已有 2A/ms 斜坡。 */
        f32ChgLimitA = __fmin(__fmax(-Ramp.iNeg, 0.0f), f32CapFilt);
        g_stDabIsrState.f32ChgLimitA = f32ChgLimitA;

        /* Buck 开通/退出只看缓放后的物理能力 f32CapFilt：f32ChgLimitA 里还夹着
         * 任务侧请求（-Ramp.iNeg），请求低于 CAP_ON 时用它判决会永远开不了波。
         * 限流值只做“能力 ∩ 请求”的取值，不参与开关判决。缓放下降当拍跟随，
         * 退出速度与用原值相同；上升限步让开通自带约 0.6ms 缓起。 */
        if (!(f32CapFilt >= DAB_CHARGE_CAP_OFF_A))
        {
            g_stDabIsrState.u16ChgBuckOn = 0U;
        }
        else if (f32CapFilt >= DAB_CHARGE_CAP_ON_A)
        {
            g_stDabIsrState.u16ChgBuckOn = 1U;
        }
        f32RefLo = -f32ChgLimitA;
    }
    else
    {//=0  放电
        g_stDabIsrState.u16ChgBuckOn    = 0U;
        f32RefLo = BUS_DISCH_OUT_MIN;
    }

    /* 外环：两模式共用，输出总电流给定 Bus.f32Ref */
    Dab_Ctrl_BusVoltLoop(f32Vslow, f32Iload, f32RefLo, u16IsCharge);

    /* 本路电流前馈 = 电压环前馈的一半 */
    stDabPi0.f32CurrFf = DAB_CURR_PER_BRIDGE * Bus.f32VoltFf;
    stDabPi1.f32CurrFf = stDabPi0.f32CurrFf;

    /* 方向封波命中：外环以上已跑完（Bus.f32Ref 必须更新，100µs 判向要用），
     * 内环结果本拍会被 OutputReset 清掉，故跳过 sqrt / 除法。 */
    if (u16HoldOff != 0U)
    {
        return;
    }

    /* ===== 轻载打嗝（充放电统一）：一段判决 + 一道休眠门 =====
     * 需求深度 d：同向需求为正（放电 d=Ref，充电 d=-Ref）。迟滞幅值 A = 最大外环
     * Kp × 允许电压漂移（见 App_DabModel.h），迟滞带随工况镜像——
     * 休眠阈/唤醒阈：充电 -A/0，放电 0/+A。反向越过休眠阈（否定式，含 NaN）
     * → 休眠；同向深过唤醒阈 → 唤醒；带内保持原态。充放电互斥，共用锁存。 */
    f32Dmd     = (u16IsCharge != 0U) ? -Bus.f32Ref : Bus.f32Ref;
    f32SealThr = (u16IsCharge != 0U) ? -DAB_BURST_HYST_A : 0.0f;
    f32WakeThr = (u16IsCharge != 0U) ? 0.0f : DAB_BURST_HYST_A;

    if (!(f32Dmd >= f32SealThr))
    {
        g_stDabIsrState.u16BurstOff = 1U;
    }
    else if (f32Dmd > f32WakeThr)
    {
        g_stDabIsrState.u16BurstOff = 0U;
    }

    /* 休眠门：放电 = 打嗝锁存；充电再加 Buck 未开 / 单路限流为 0。
     * 命中即清两路 PI/前馈/占空比并封波（不锁故障）；电压外环继续运行，
     * 唤醒首拍由内环前馈软起给出，不额外叠复位暂态。 */
    if (u16IsCharge != 0U)
    {
        f32IrefMax = -DAB_CURR_PER_BRIDGE * f32RefLo;
    }
    u16BurstSleep = g_stDabIsrState.u16BurstOff;
    if ((u16IsCharge != 0U) &&
        ((g_stDabIsrState.u16ChgBuckOn == 0U) || (f32IrefMax <= 0.0f)))
    {
        u16BurstSleep = 1U;
    }
    if (u16BurstSleep != 0U)
    {
        Dab_Ctrl_OutputReset();
        return;
    }

    /* 内环（休眠门已在上方统一拦截，分支内只剩正常调节数学） */
    if (u16IsCharge == 0U)//放电
    {
        /* ▲ 两路共用单桥模型前馈：入参必须是单桥给定（总给定 × 0.5） */
        float f32Dff = Dab_Ctrl_DisFfCalc(DAB_CURR_PER_BRIDGE * Bus.f32Ref);

        Dab_Ctrl_DisCurrLoop(&stDabPi0, &stDabDuty0, f32I0, f32Dff);
        Dab_Ctrl_DisCurrLoop(&stDabPi1, &stDabDuty1, f32I1, f32Dff);
    }
    else
    {//充电
        ST_DAB_CHG_CTX stChgCtx;    /* 拍级参数包：两路共用的本拍快照 */

        /* 单路给定：总给定 × 0.5，先抬到保持电流再用限流封顶（限流优先）。
         * 通俗版：Ref 充电时记负账，前面加负号翻成"正的电流大小"——好比油门踩多深；
         * 两桥各抬一半 = 两人抬一桶水。
         * __fmax 是怠速：需求快没了也留 0.02A 小油门，维持发波不打嗝；
         * __fmin 是红线：油门踩到底也不能超过限流，红线说了算（所以限流写在外层）。 */
        stChgCtx.f32Iref = __fmin(__fmax(-DAB_CURR_PER_BRIDGE * Bus.f32Ref, DAB_CHARGE_I_KEEP_MIN), f32IrefMax);

        /* 其余拍级共用参数打包（定义见 Interrupt_CtrlDab.h），Main 填一次、两路传同一指针 */
        stChgCtx.f32Vslow   = f32Vslow;                         /* 陷波母线电压 */
        stChgCtx.f32InvDv   = f32InvDv;                         /* 1/压差 */
        stChgCtx.f32DutyMax = f32DutyMax;                       /* 占空比上限 */
        stChgCtx.f32IcmdMax = f32ChgImax * DAB_CURR_PER_BRIDGE; /* 电流指令上限 */

        /* SR 伏秒比：取两路电压较小者，SR 宁短勿长；必须在两路 SR 计算之前发布 */
        pstParam->f32SrRatio = __divf32(__fmin(f32Vslow, f32VbusRaw), f32E);

        Dab_Ctrl_ChgCurrLoop(&stChgCtx, &stDabPi0, &stDabDuty0, f32I0);
        Dab_Ctrl_ChgCurrLoop(&stChgCtx, &stDabPi1, &stDabDuty1, f32I1);
    }
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

/* 充电发波：S1~S4 为固定前沿 + 移相后沿，S5~S8 跟随 SR。
 * 返回 1 表示本桥实际在发波（供均流读取）。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Pwm_ChgUpdate);
static inline Uint16 Dab_Pwm_ChgUpdate(volatile struct EPWM_REGS *pstP1,
                                       volatile struct EPWM_REGS *pstP2,
                                       volatile struct EPWM_REGS *pstP3,
                                       volatile struct EPWM_REGS *pstP4,
                                       float f32S1TopDuty, float f32S2LowDuty, float f32S3SrDuty,
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

    u16S1TopCmpTmp     = (Uint16)(2.0f * f32S1TopDuty * (float)DAB_CHG_TBPRD + 0.5f);
    u16S4LowCmpTmp     = (Uint16)(2.0f * f32S2LowDuty * (float)DAB_CHG_TBPRD + 0.5f);
    u16S5S8SrCmpTmp    = (Uint16)(2.0f * f32S3SrDuty  * (float)DAB_CHG_TBPRD + 0.5f);
    u16S2LowNegCmpTmp  = (Uint16)(DAB_CHG_TBPRD - u16S4LowCmpTmp);
    u16S3TopNegCmpTmp  = (Uint16)(DAB_CHG_TBPRD - u16S1TopCmpTmp);
    u16S6S7SrNegCmpTmp = (Uint16)(DAB_CHG_TBPRD - u16S5S8SrCmpTmp);

    /* 脉宽过窄（d < DAB_DUTY_MIN）不发波：S1~S4 两路都够宽才开，S5~S8 跟随 */
    u16S1S4ActTmp = ((f32S1TopDuty >= DAB_DUTY_MIN) && (f32S2LowDuty >= DAB_DUTY_MIN)) ?
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

    /* 释放强制：只在 OFF→ON 那一拍重写常量（相位已 ON 就跳过） */
    if (pstAqLast->bit.u16Phase == 0U)
    {
        pstP1->AQSFRC.bit.RLDCSF = 0U;
        pstP2->AQSFRC.bit.RLDCSF = 0U;
        pstP3->AQSFRC.bit.RLDCSF = 0U;
        pstP4->AQSFRC.bit.RLDCSF = 0U;
        pstP1->AQCSFRC.all = 0U;
        pstP2->AQCSFRC.all = 0U;
        pstP3->AQCSFRC.all = 0U;
        pstP4->AQCSFRC.all = 0U;
    }

    /* 【重要】武装必须放在本拍所有影子写之后：
     *   ① 只武装本桥 one-shot GLD，另一桥的影子此刻可能还没写完；
     *   ② AQCSFRC 参与 GLD，RLDCSF=0 时写入的是影子，若在 4 次 AQCSFRC 写之间
     *      被 ZERO 装载，只能拿到部分模块的释放值，首拍就会出现同桥分拍释放；
     *   ③ 武装距装载事件不足 3 TBCLK 时会推迟到下一个事件（首拍晚一个载波），方向安全。 */
    EALLOW;
    pstP1->GLDCTL2.bit.OSHTLD = 1U;
    EDIS;

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
        u16S2HvNegCmpTmp  = (Uint16)(DAB_DIS_TBPRD - u16S4HvCmpTmp);
        u16InnerActTmp    = AQ_CLEAR;
    }
    else
    {
        u16S2HvNegCmpTmp  = (Uint16)(u16DutyTicksTmp - DAB_DIS_TBPRD);
        u16S4HvCmpTmp     = (Uint16)(DAB_DIS_TBPRD - u16S2HvNegCmpTmp);
        u16InnerActTmp    = AQ_NO_ACTION;
    }

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

    /* 发波：先切 ZERO 装载，再解除强制，避免立即释放 */
    if (pstAqLast->bit.u16Phase == 0U)
    {
        pstP1->AQSFRC.bit.RLDCSF = 0U;
        pstP2->AQSFRC.bit.RLDCSF = 0U;
        pstP3->AQSFRC.bit.RLDCSF = 0U;
        pstP4->AQSFRC.bit.RLDCSF = 0U;
        pstP1->AQCSFRC.all = 0x0001U;   /* S1 常闭 */
        pstP2->AQCSFRC.all = 0x0001U;   /* S3 常闭 */
        pstP3->AQCSFRC.all = 0x0000U;   /* 低压桥不强制 */
        pstP4->AQCSFRC.all = 0x0000U;
    }

    /* 武装放在所有影子写之后，理由同充电发波 */
    EALLOW;
    pstP1->GLDCTL2.bit.OSHTLD = 1U;
    EDIS;
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
 *   ① 故障或非闭环态 → 强制低，直接返回（不跑控制）；
 *   ② openTest 开环测试 → 跳过内外环与方向门，直接按给定占空比发波；
 *   ③ 方向封波命中 → 先跑外环（100µs 判向要用 Bus.f32Ref），再清内环、强制低；
 *   ④ 否则正常发波。方向门必须在 Dab_Pwm_*Update 之前：后者会清 AQCSFRC 并置
 *      OSHTLD，若先发波再强制低会被下一拍的加载顶掉。 */
#pragma FUNC_ALWAYS_INLINE(Dab_Ctrl_fun);
static inline void Dab_Ctrl_fun(float f32Vbus, float f32I0, float f32I1, Uint16 u16IsCharge)
{
    Uint16 u16State = (Uint16)RunState.state;
    Uint16 u16HoldOff;
    Uint16 u16Seal;
    Uint16 u16Mode;

    if (FAULT_ANY() || CfgParam.SysCmd.bit.faultcmd ||
        ((u16State != (Uint16)SM_STATE_RAMP_CLOSE) && (u16State != (Uint16)SM_STATE_RUN)))
    {
        Dab_Pwm_Seal();
        return;
    }

    /* 方向封波判定只算一次：门（清输出/钳位）与 Main（跳过内环）共用同一份结论 */
    u16Seal = RunState.curLowSealing;
    u16Mode = (Uint16)RunState.workMode;
    u16HoldOff = ((((u16Seal & SM_SEAL_NO_DISCHARGE) != 0U) && (u16Mode == (Uint16)WORK_MODE_DISCHARGE)) ||
                  (((u16Seal & SM_SEAL_NO_CHARGE)    != 0U) && (u16Mode == (Uint16)WORK_MODE_CHARGE)))
                 ? 1U : 0U;

    /* ===== ② 开环发波工程测试（openTest）=====
     * 台架无高压源 / 不接闭环时使用：跳过内外环与方向判定，两桥直接按
     * CfgParam.OpenLoopDuty（单位 0.1%，100 = 10.0%）发波。
     * 需要自动扫占空比时，可把 f32TestDuty 换成 0~1 循环的 funum 斜坡（原实现）。
     * 本分支不更新 Bus.f32Ref / u16ChgPwmActive；欠压等判据由 Fault_OpenTest 门控。 */
    if (CfgParam.SysCmd.bit.openTest != 0U)
    {
        float f32TestDuty = CfgParam.OpenLoopDuty * 0.001f;   /* 0.1% → 0~1 */

        g_stDabIsrState.f32ChgLimitA = 0.0f;

        if (u16IsCharge != 0U)
        {
            stDabDuty0.f32SrDuty = f32TestDuty * 1.1f;        /* SR 略宽于主占空比 */
            stDabDuty1.f32SrDuty = stDabDuty0.f32SrDuty;

            (void)Dab_Pwm_ChgUpdate(&DabEpwm1Regs, &DabEpwm2Regs, &DabEpwm3Regs, &DabEpwm4Regs,
                                    DAB_CHG_D_TOP, f32TestDuty, stDabDuty0.f32SrDuty,
                                    &g_stDabIsrState.stChgAqBr1);
            (void)Dab_Pwm_ChgUpdate(&DabEpwm1bRegs, &DabEpwm2bRegs, &DabEpwm3bRegs, &DabEpwm4bRegs,
                                    DAB_CHG_D_TOP, f32TestDuty, stDabDuty1.f32SrDuty,
                                    &g_stDabIsrState.stChgAqBr2);
        }
        else
        {
            float f32Hv;
            float f32Lv;

            Dab_Ctrl_DisDutyMap(f32TestDuty, &f32Hv, &f32Lv); /* 两桥同一映射 */
            Dab_Pwm_DisUpdate(&DabEpwm1Regs, &DabEpwm2Regs, &DabEpwm3Regs, &DabEpwm4Regs,
                              f32Hv, f32Lv, &g_stDabIsrState.stChgAqBr1);
            Dab_Pwm_DisUpdate(&DabEpwm1bRegs, &DabEpwm2bRegs, &DabEpwm3bRegs, &DabEpwm4bRegs,
                              f32Hv, f32Lv, &g_stDabIsrState.stChgAqBr2);
        }

        /* 常量已写：置相位，后续拍只刷比较值 */
        g_stDabIsrState.stChgAqBr1.bit.u16Phase = 1U;
        g_stDabIsrState.stChgAqBr2.bit.u16Phase = 1U;
        return;
    }

    Dab_Ctrl_Loop(&stDabParam, f32Vbus, f32I0, f32I1, u16IsCharge, u16HoldOff);

    if (u16HoldOff != 0U)
    {
        Dab_Ctrl_OutputReset();

        /* 反积分饱和：把外环输出钳到被封方向的 0 点，禁止往封禁方向继续攒给定，
         * 但保留反方向积分能力。f32Ref 必须同向一起钳，否则 100µs 判向会读到
         * 本拍的虚假电流给定而无法换向。 */
        if (u16Mode == (Uint16)WORK_MODE_DISCHARGE)
        {
            Bus.f32Out = __fmin(Bus.f32Out, 0.0f);   /* 正 = 放电 */
            Bus.f32Ref = __fmin(Bus.f32Ref, 0.0f);
        }
        else
        {
            Bus.f32Out = __fmax(Bus.f32Out, 0.0f);   /* 负 = 充电 */
            Bus.f32Ref = __fmax(Bus.f32Ref, 0.0f);
        }

        Dab_Pwm_Seal();
        return;
    }

    /* 两路电流环各算各的，分别写各自桥；同一母线，两桥时基相差 90° 交错 */
    if (u16IsCharge != 0U)
    {
        Uint16 u16ActiveTmp;

        u16ActiveTmp  = Dab_Pwm_ChgUpdate(&DabEpwm1Regs, &DabEpwm2Regs, &DabEpwm3Regs, &DabEpwm4Regs,
                                         DAB_CHG_D_TOP, stDabDuty0.f32Duty, stDabDuty0.f32SrDuty,
                                         &g_stDabIsrState.stChgAqBr1);
        u16ActiveTmp |= Dab_Pwm_ChgUpdate(&DabEpwm1bRegs, &DabEpwm2bRegs, &DabEpwm3bRegs, &DabEpwm4bRegs,
                                         DAB_CHG_D_TOP, stDabDuty1.f32Duty, stDabDuty1.f32SrDuty,
                                         &g_stDabIsrState.stChgAqBr2);
        g_stDabIsrState.u16ChgPwmActive = u16ActiveTmp;
    }
    else
    {
        float f32Hv0, f32Lv0, f32Hv1, f32Lv1;

        g_stDabIsrState.u16ChgPwmActive = 0U;   /* 放电时清充电发波标记 */
        g_stDabIsrState.f32ChgLimitA    = 0.0f;
        g_stDabIsrState.f32CapFilt      = 0.0f;

        Dab_Ctrl_DisDutyMap(stDabDuty0.f32Duty, &f32Hv0, &f32Lv0);
        Dab_Ctrl_DisDutyMap(stDabDuty1.f32Duty, &f32Hv1, &f32Lv1);
        Dab_Pwm_DisUpdate(&DabEpwm1Regs, &DabEpwm2Regs, &DabEpwm3Regs, &DabEpwm4Regs,
                          f32Hv0, f32Lv0, &g_stDabIsrState.stChgAqBr1);
        Dab_Pwm_DisUpdate(&DabEpwm1bRegs, &DabEpwm2bRegs, &DabEpwm3bRegs, &DabEpwm4bRegs,
                          f32Hv1, f32Lv1, &g_stDabIsrState.stChgAqBr2);
    }

    /* 两桥常量均已按本拍相位写完/确认，置 1 供下一拍跳过常量写 */
    g_stDabIsrState.stChgAqBr1.bit.u16Phase = 1U;
    g_stDabIsrState.stChgAqBr2.bit.u16Phase = 1U;
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
    Uint16 save_idx;

    /* 1. 未触发阶段 (Pre-Trigger)：在 [0, PRE_TRIGGER_NUM - 1] 维持环形缓冲 */
    if (DataScope.uTran_CMD == 0U)
    {
        /* --- 触发判断 (母线过压 / 通道过流 / 给定方向与工况矛盾) --- */
        if ((stADC.VmBusVolt.f32IsrResult > DAB_DISCHARGE_BUS_OVP_VOLT) ||  /* VM 母线过压 */
            (fabsf(f32I0) > FAULT_CH_OC) ||  (fabsf(f32I1) > FAULT_CH_OC))                               /* 通道 1 过流 */
        {
            /* 触发命中：锁定标志，冻结前 64 点环，转线性记录后触发数据 */
            DataScope.uTran_CMD = 1U;
            DataScope.uCurrent_Save_subscript = PRE_TRIGGER_NUM;
            save_idx = DataScope.uCurrent_Save_subscript++;
        }
        else
        {
            /* 正常工况：预触发环槽位循环覆写 */
            save_idx = DataScope.uPreTriggerIdx;
            DataScope.uPreTriggerIdx = (Uint16)((DataScope.uPreTriggerIdx + 1U) & PRE_TRIGGER_MASK);
        }
    }
    /* 2. 已触发阶段 (Post-Trigger)：线性连续存满剩余缓冲区至 SCOPENUM - 1 */
    else
    {
        /* 防御性纠偏（经典 scopeGet 结构）：若触发时下标未正确初始化（如任务置
           uTran_CMD 与 ISR 读写的交错），强制从第 PRE_TRIGGER_NUM 点起记录，
           避免写入预触发环槽位破坏触发前波形 */
        if (DataScope.uCurrent_Save_subscript < (Uint16)PRE_TRIGGER_NUM)
        {
            DataScope.uCurrent_Save_subscript = (Uint16)PRE_TRIGGER_NUM;
        }

        if (DataScope.uCurrent_Save_subscript < (Uint16)SCOPENUM)
        {
            save_idx = DataScope.uCurrent_Save_subscript++;
        }
        else
        {
            /* 录波已存满（900 点），保持锁定，等上位机 CAN 发送完毕 */
            return;
        }
    }

    /* 3. 统一记录 4 通道关键控制量（定点放大 10 倍存储） */
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

    /* ① 快保护：只判断置位（母线过压取 max(VBUS1, VBUS2)） */
    Fault_FastProtect(f32I0, f32I1);

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
    Dab_Ctrl_fun(f32Vbus, f32I0, f32I1, u16IsCharge);

    /* ③ 应答 */
    DabEpwm1Regs.ETCLR.bit.INT = 1U;          /* EPWM10 ZERO */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
