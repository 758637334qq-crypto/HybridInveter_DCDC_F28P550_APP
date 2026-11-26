/*************************************************************************************************
 * App_CtrlData.c — DAB 给定、缓起、复位（1ms 任务，不在中断里算环）
 *
 *   Ctrl_Init / Ctrl_StandbyClear / Ctrl_SetMode  起机；待机入口清均流；出态装工况+环清零
 *   Ctrl_RampCfg                 每 1ms：配置 → 限值/禁向 → 斜坡 → 电压给定
 *   Ctrl_RampInitCold / Ctrl_RampInitSwitch  冷启/换向：清 Bus + 斜坡播种
 *
 * 电压外环和电流内环在 Interrupt_CtrlDab.c。
 * 运行中换向由状态机在封波、关中断窗口内提交。
 *************************************************************************************************/
#include "Includes.h"
#include "App_DabModel.h"

/* 初始化增益满幅误差的倒数(编译期常量) */
#define DAB_INIT_VERR_FULL_INV  (1.0f / DAB_INIT_VERR_FULL_V)

/*==============================================================================
 * 斜坡范围与步长
 *============================================================================*/
#define DAB_VOLT_MIN            (360.0f)   /* 电压下限(V)，再低会倒灌 */
#define DAB_VOLT_MAX            (450.0f)   /* 电压上限(V) */
#define DAB_VOLT_RAMP_STEP      (1.0f)     /* 常态电压步长：1V/ms */
#define DAB_VOLT_FAST_EXIT      (4.0f)     /* 换向快爬退出偏差(V) */
#define DAB_CURR_RAMP_STEP      (2.0f)     /* 电流限值放宽步长：2A/ms */
#define DAB_CHG_INIT_A          (2.0f)     /* 充电冷启：Ramp.iNeg 起步幅值(A) */
#define DAB_DIS_INIT_A          (10.0f)    /* 放电冷启：Ramp.iPos 起步幅值(A) */
#define DAB_INIT_VERR_FULL_V    (40.0f)    /* 压差≥此值：初值增益=1（满幅） */
#define DAB_VOLT_INIT_BIAS_MAX  (10.0f)    /* Ramp.v 初值朝设定最多预偏(V) */
#define DAB_RAMP_VOLT_ERR       (0.5f)     /* 电压到位判据(V) */
#define DAB_RAMP_CURR_ERR       (0.05f)    /* 电流到位判据(A) */

/*==============================================================================
 * 对象
 *============================================================================*/
volatile ST_DAB_PARAM stDabParam;
volatile ST_DAB_BUS Bus;
ST_DAB_PI stDabPi0, stDabPi1;
ST_DAB_DUTY stDabDuty0, stDabDuty1;

ST_DAB_RAMP Ramp;
ST_DAB_RAMP RampTarget;
float g_CtrlBusRef_V;

/*==============================================================================
 * 限幅与斜坡步进
 *============================================================================*/
static inline float sClampF(float f32V, float f32Lo, float f32Hi)
{
    if (f32V < f32Lo)
    {
        return f32Lo;
    }
    if (f32V > f32Hi)
    {
        return f32Hi;
    }
    return f32V;
}

/* 偏差大于一步只挪一步，不足一步直接落到目标。 */
static inline float sRampTrack(float f32Value, float f32Target, float f32Step)
{
    float f32DeltaTmp;

    f32DeltaTmp = f32Target - f32Value;

    if (f32DeltaTmp > f32Step)
    {
        return f32Value + f32Step;
    }
    if (f32DeltaTmp < -f32Step)
    {
        return f32Value - f32Step;
    }
    return f32Target;
}

/*==============================================================================
 * 电流环与发波
 *============================================================================*/
/* 只清运行记忆。K1/K2/OutMax 由 Ctrl_SetMode 装载。 */
static void sPiReset(ST_DAB_PI *pstPi)
{
    pstPi->f32Err1 = 0.0f;
    pstPi->f32Out = 0.0f;
    pstPi->f32Ref = 0.0f;
    pstPi->f32DutyFf = 0.0f;
    pstPi->u16DutySat = 0U;   /* 饱和标志一并清零，换向/复位后外环从"未饱和"起步 */
}

static void sPiInit(ST_DAB_PI *pstPi, float f32K1, float f32K2, float f32OutMax)
{
    sPiReset(pstPi);
    pstPi->f32K1 = f32K1;
    pstPi->f32K2 = f32K2;
    pstPi->f32OutMax = f32OutMax;
}

static void sDutyClear(ST_DAB_DUTY *pstDuty)
{
    pstDuty->f32Duty = 0.0f;
    pstDuty->f32SrDuty = 0.0f;
    pstDuty->u16SrEnable = 0U;
    pstDuty->u16SrOnCnt = 0U;
    pstDuty->u16SrSoft = 0U;
}

/*==============================================================================
 * 电压外环
 * 清动态量，保留 f32Vref。
 *============================================================================*/
static void sBusClear(void)
{
    Bus.f32Err         = 0.0f;
    Bus.f32Err1        = 0.0f;
    Bus.f32Integ       = 0.0f;
    Bus.f32Out         = 0.0f;
    Bus.f32Ref         = 0.0f;

    Bus.u16Tick        = 0U;
    Bus.u16RecoverRamp = 0U;
}

/* 清动态量后把 Vref 贴到实测。调用方已经清过内环和占空比。 */
static void sBusSeed(float vo)
{
    sBusClear();
    Bus.f32Vref = vo;
}

/*==============================================================================
 * 斜坡发布
 * 唯一写 Ramp 的入口，仅任务上下文。换向/播种时调用方已封波。
 *============================================================================*/
static void sRampPublish(float f32V, float f32IPos, float f32INeg)
{
    Ramp.v    = f32V;
    Ramp.iPos = f32IPos;
    Ramp.iNeg = f32INeg;
}

/* 当前值与终点一起对齐，用于起机和冷启动。 */
static void sRampCommit(float f32V, float f32IPos, float f32INeg)
{
    sRampPublish(f32V, f32IPos, f32INeg);
    RampTarget.v    = f32V;
    RampTarget.iPos = f32IPos;
    RampTarget.iNeg = f32INeg;
}

/* 常态 1V/ms。u16RecoverRamp=1 时按剩余压差分档，进入 4V 以内回到 1V/ms。
 *   ≤4V → 1    4~12V → 2    12~25V → 3    25~40V → 6    >40V → 10 */
static float sVoltStepSelect(float f32Diff)
{
    if (Bus.u16RecoverRamp == 0U)
    {
        return DAB_VOLT_RAMP_STEP;
    }

    f32Diff = fabsf(f32Diff);

    if (f32Diff <= DAB_VOLT_FAST_EXIT)
    {
        return DAB_VOLT_RAMP_STEP;
    }
    if (f32Diff > 40.0f)
    {
        return 10.0f;
    }
    if (f32Diff > 25.0f)
    {
        return 6.0f;
    }
    if (f32Diff > 12.0f)
    {
        return 3.0f;
    }
    return 2.0f;
}

/* 电压限速前进。电流限值放宽走斜坡，收紧立即生效。终点记入 RampTarget。 */
static void Ctrl_RampSet(float f32VoltRef, float f32DischargeLimit, float f32ChargeLimit)
{
    float f32VoltStepTmp;
    float f32VoltNext, f32PosNext, f32NegNext;

    f32VoltRef = sClampF(f32VoltRef, DAB_VOLT_MIN, DAB_VOLT_MAX);
    f32DischargeLimit = sClampF(f32DischargeLimit, 0.0f, DAB_CURR_LIMIT);
    f32ChargeLimit = sClampF(f32ChargeLimit, -DAB_CURR_LIMIT, 0.0f);

    f32VoltStepTmp = sVoltStepSelect(f32VoltRef - Ramp.v);

    f32VoltNext = sRampTrack(Ramp.v, f32VoltRef, f32VoltStepTmp);
    f32PosNext = __fmin(sRampTrack(Ramp.iPos, f32DischargeLimit, DAB_CURR_RAMP_STEP), f32DischargeLimit);
    f32NegNext = __fmax(sRampTrack(Ramp.iNeg, f32ChargeLimit, DAB_CURR_RAMP_STEP), f32ChargeLimit);

    if (fabsf(f32VoltRef - f32VoltNext) <= DAB_VOLT_FAST_EXIT)
    {
        Bus.u16RecoverRamp = 0U;
    }

    RampTarget.v = f32VoltRef;
    RampTarget.iPos = f32DischargeLimit;
    RampTarget.iNeg = f32ChargeLimit;
    sRampPublish(f32VoltNext, f32PosNext, f32NegNext);
}

/*==============================================================================
 * 限值与禁向
 * 1ms 刷新、冷启动和换向共用，避免解封到下一次 1ms 之间禁向松开。
 * 放电限值 = BusDischgCurSet×0.1；充电限值 = −BusChgCurSet×0.1，再与功率降额取较大（更接近 0）。
 * 某方向设定 ≤0.5A：限值归 0，并置对应禁向位。
 *============================================================================*/
static void sLimitSealSynth(float *pf32Dis, float *pf32Chg, Uint16 *pu16Seal)
{
    float f32VbusTmp, f32VbatTmp, f32LimitTmp;
    float f32Dis, f32Chg;
    Uint16 u16SealTmp = 0U;

    f32Dis = (float)CfgParam.BusDischgCurSet * 0.1f;
    f32Chg = -(float)CfgParam.BusChgCurSet * 0.1f;

    /* 1ms 功率降额：整机高压侧电流与下发充电 iset 取较小幅值。充电为负，取 max。 */
    f32VbusTmp = stADC.VmBusVolt.f32Notch;
    f32VbatTmp = stADC.BatVolt.Filt.f32Filt10ms;
    f32LimitTmp = 0.0f;

    if (f32VbusTmp > 0.0f && f32VbusTmp < 500.0f &&
        f32VbatTmp >= DAB_BAT_VOLT_MIN && f32VbatTmp < DAB_BAT_OVP_VOLT)
    {
        f32LimitTmp = -__divf32(__fmin(DAB_CHARGE_POWER_MAX,
            DAB_CHARGE_BAT_CUR_MAX * f32VbatTmp), f32VbusTmp);
    }
    f32Chg = __fmax(f32Chg, f32LimitTmp);

    if (f32Dis <= SM_IBUS_DISCHAR_STANDBY_MIN)
    {
        f32Dis = 0.0f;
        u16SealTmp |= SM_SEAL_NO_DISCHARGE;
    }

    if (f32Chg >= -SM_IBUS_CHAR_STANDBY_MIN)
    {
        f32Chg = 0.0f;
        u16SealTmp |= SM_SEAL_NO_CHARGE;
    }

    *pf32Dis = f32Dis;
    *pf32Chg = f32Chg;
    *pu16Seal = u16SealTmp;
}

/* 全量计算有效给定+禁向位图：与冷启/1ms 斜坡共用 sLimitSealSynth，供 PRECHECK
 * 判向拍一次算出，结果经 RunState.f32ColdDis/f32ColdChg 交冷启装载复用。 */
Uint16 Ctrl_CalcGiven(float *pf32Dis, float *pf32Chg)
{
    Uint16 u16Seal;

    sLimitSealSynth(pf32Dis, pf32Chg, &u16Seal);
    return u16Seal;
}

/*==============================================================================
 * 起机与换向装载
 *============================================================================*/
void Ctrl_Init(void)
{
    sRampCommit(DAB_VOLT_MIN, DAB_CHG_INIT_A, -DAB_CHG_INIT_A);
    CurShare_ResetComp();
    g_CtrlBusRef_V = Ramp.v;

    sBusClear();

    sPiReset(&stDabPi0);
    sPiReset(&stDabPi1);
    sDutyClear(&stDabDuty0);
    sDutyClear(&stDabDuty1);
}

/* STANDBY 入口：只清均流补偿，避免旧 voltAdj/droop 带到出态。
 * 不在此清 Bus/PI/Duty：驻留期 ISR 白名单外 FORCE_LOW、不跑 Dab_Ctrl_Loop，
 * 不读旧 f32Out；环清零集中在出态关中断窗 SetMode + RampInit*。 */
void Ctrl_StandbyClear(void)
{
    CurShare_ResetComp();
}

/* 封波关中断窗口内装载工况：内环系数、PI 占空比上限、u16IsCharge，并清两路 PI 与 Duty。
 * 出 STANDBY / 冷启共用此环清零点；Bus 动态量与 Ramp 由随后 RampInit*（含 sBusSeed）覆盖。
 * 充电 K1/K2 置 0，ISR 用专用系数，PI 上限 DAB_CHG_D_LOW_MAX；放电 K1=Kp+Ki·Ts，K2=Kp，上限 1。
 * 非充电入参一律按放电装，避免残留上一档参数。
 * 误差低通用固定宏 DAB_BUS_ALPHA，不再按载波装载。 */
void Ctrl_SetMode(Uint16 workMode)
{
    float f32Kp, f32KiTs, f32DutyMax;

    if (workMode == (Uint16)WORK_MODE_CHARGE)
    {
        f32Kp = 0.0f;
        f32KiTs = 0.0f;
        f32DutyMax = DAB_CHG_D_LOW_MAX;
        stDabParam.u16IsCharge = 1U;
    }
    else
    {
        f32Kp = DAB_DISCHARGE_KP;
        f32KiTs = DAB_DISCHARGE_KI_TS;
        f32DutyMax = 1.0f;
        stDabParam.u16IsCharge = 0U;
    }

    sPiInit(&stDabPi0, f32Kp + f32KiTs, f32Kp, f32DutyMax);
    sPiInit(&stDabPi1, f32Kp + f32KiTs, f32Kp, f32DutyMax);

    sDutyClear(&stDabDuty0);
    sDutyClear(&stDabDuty1);
}

/*==============================================================================
 * 1ms 斜坡
 *============================================================================*/
/* 闭环缓起及之后的充放电才推进斜坡和禁向。均流补偿每拍都发布。
 * 待机仅关波：不推进斜坡，避免解除待机时满额冲击。
 * curLowSealing 一次写完：分步写会被 EPWM ISR 读到中间态。 */
void Ctrl_RampCfg(void)
{
    float f32VoltRefTmp;
    float f32DischargeLimitTmp;
    float f32ChargeLimitTmp;
    Uint16 u16ModeTmp;
    Uint16 u16StateTmp;
    Uint16 u16SealTmp;

    u16StateTmp = (Uint16)RunState.state;
    u16ModeTmp = RunState.workMode;
    /* 白名单：仅 RAMP_CLOSE/RUN 爬坡（避免 state=MODE_SWITCH 等穿透） */
    if (((u16StateTmp == (Uint16)SM_STATE_RAMP_CLOSE) ||
         (u16StateTmp == (Uint16)SM_STATE_RUN)) &&
        ((u16ModeTmp == (Uint16)WORK_MODE_CHARGE) ||
         (u16ModeTmp == (Uint16)WORK_MODE_DISCHARGE)))
    {
        f32VoltRefTmp = (float)CfgParam.BusVolSet * 0.1f;
        sLimitSealSynth(&f32DischargeLimitTmp, &f32ChargeLimitTmp, &u16SealTmp);
        RunState.curLowSealing = u16SealTmp;
        Ctrl_RampSet(f32VoltRefTmp, f32DischargeLimitTmp, f32ChargeLimitTmp);
    }

    CurShare_Update1ms(DAB_VOLT_MIN - Ramp.v, DAB_VOLT_MAX - Ramp.v);
    g_CtrlBusRef_V = Ramp.v + CurShare.voltAdj;
}

/* 电压差 ≤0.5V 且两向电流差 ≤0.05A 才算到位。 */
Uint16 Ctrl_RampDone(void)
{
    return ((fabsf(RampTarget.v - Ramp.v) <= DAB_RAMP_VOLT_ERR) &&
            (fabsf(RampTarget.iPos - Ramp.iPos) <= DAB_RAMP_CURR_ERR) &&
            (fabsf(RampTarget.iNeg - Ramp.iNeg) <= DAB_RAMP_CURR_ERR)) ? 1U : 0U;
}

/*==============================================================================
 * 斜坡初值（冷启 / 换向装载 Ramp、RampTarget）
 *============================================================================*/
/* 冷启/换向共用：装限值与禁向，写 Ramp 初值与 RampTarget 终点，清外环。
 * 压差增益连续：|Verr|/40V → [0,1]；电压预偏满幅 DAB_VOLT_INIT_BIAS_MAX；
 * 充电换向：2A→满设定同增益；放电冷启 10A、换向满设定；充电冷启 −2A。 */
static void sRampInitCommon(float f32VoltFbk, Uint16 u16IsCold)
{
    float f32VoltAct = sClampF(f32VoltFbk, 0.0f, DAB_VOLT_MAX);
    float f32VoltRef;
    float f32VoltInit;
    float f32Verr;
    float f32AbsVerr;
    float f32Gain;
    float f32Bias;
    float f32Dis, f32Chg;
    float f32Pos, f32Neg;
    float f32ChgAbs;
    Uint16 u16Seal;

    if (u16IsCold == 1U)
    {
        /* 冷启：复用 PRECHECK 判向当拍落位的有效给定（同拍口径，不重算，
         * 保证判向、装载、1ms 爬坡三处单一规则） */
        f32Dis = RunState.f32ColdDis;
        f32Chg = RunState.f32ColdChg;
        u16Seal = RunState.curLowSealing;
    }
    else
    {
        sLimitSealSynth(&f32Dis, &f32Chg, &u16Seal);
        RunState.curLowSealing = u16Seal;
    }

    f32VoltRef = sClampF((float)CfgParam.BusVolSet * 0.1f, DAB_VOLT_MIN, DAB_VOLT_MAX);

    /* 压差增益：0V→0，≥40V→1，中间线性 */
    f32Verr    = f32VoltRef - f32VoltAct;
    f32AbsVerr = fabsf(f32Verr);
    if (f32AbsVerr >= DAB_INIT_VERR_FULL_V)
    {
        f32Gain = 1.0f;
    }
    else
    {
        f32Gain = f32AbsVerr * DAB_INIT_VERR_FULL_INV;
    }

    /* Ramp.v：朝设定预偏，不越过终点 */
    f32Bias = f32Gain * DAB_VOLT_INIT_BIAS_MAX;
    if (f32Verr > 0.0f)
    {
        f32VoltInit = __fmin(f32VoltAct + f32Bias, f32VoltRef);
    }
    else if (f32Verr < 0.0f)
    {
        f32VoltInit = __fmax(f32VoltAct - f32Bias, f32VoltRef);
    }
    else
    {
        f32VoltInit = f32VoltAct;
    }

    if (u16IsCold == 1U)
    {
        f32Pos = __fmin(f32Dis, DAB_DIS_INIT_A);   /* 放电冷启：10A */
        f32Neg = __fmax(f32Chg, -DAB_CHG_INIT_A);  /* 充电冷启：−2A */
    }
    else
    {
        f32Pos = f32Dis;                           /* 放电换向：满设定 */
        f32ChgAbs = -f32Chg;
        if (f32ChgAbs <= DAB_CHG_INIT_A)
        {
            f32Neg = f32Chg;
        }
        else
        {
            /* 充电换向：2A→满设定，与电压同一增益 */
            f32Neg = -(DAB_CHG_INIT_A + f32Gain * (f32ChgAbs - DAB_CHG_INIT_A));
        }
    }

    sRampPublish(f32VoltInit, f32Pos, f32Neg);
    RampTarget.v    = f32VoltRef;
    RampTarget.iPos = f32Dis;
    RampTarget.iNeg = f32Chg;
    sBusSeed(f32VoltInit);
    g_CtrlBusRef_V = f32VoltInit + CurShare.voltAdj;
}

/* 冷启：Ramp.v 按压差预偏，RampTarget=设定；放电 10A / 充电 −2A。 */
void Ctrl_RampInitCold(float f32VoltFbk)
{
    sRampInitCommon(f32VoltFbk, 1U);
}

/* 换向/出待机：Ramp.v 按压差预偏；放电满设定 / 充电压差增益；电压快爬。 */
void Ctrl_RampInitSwitch(float f32VoltFbk)
{
    sRampInitCommon(f32VoltFbk, 0U);
    Bus.u16RecoverRamp = 1U;
}

