/**
 * ============================================================================
 * 文件名称: App_Fault.c
 * 模块功能: 只做故障判断与置位（安克 FaultDetect 角色）。
 *   状态机 / ISR 直接读 stFault；软起计时在 SM，满则调 Fault_SetSoftTo。
 *   本模块是检测层，不改状态流转，不写 RunState。
 * ----------------------------------------------------------------------------
 * 阅读顺序：§1 类型/变量 → §2 RAM → §3 状态判据 → §4 去抖 → §5 慢保护
 *           → §6 支撑 → §7 快恢复 → §8 对外接口
 * ============================================================================
 */
#include "Includes.h"          /* 全局头文件：含 stADC、RunState、CfgParam、Bus 等声明 */
#include "Interrupt_CtrlDab.h" /* ISR 发布的充电发波状态（PwmOn 判据使用） */

/* ==========================================================================
 * §1 本文件类型与变量（不对外；CCS 观察 stFaultCnt）
 * ========================================================================== */

typedef struct
{
    Uint16 hits;
    Uint16 win;
} RetryLock_t;

typedef struct
{
    struct
    {
        Uint16 batUv;
        Uint16 vbusOv;
        Uint16 ibusOv;
        Uint16 powerOv;
        Uint16 vbusUn;
        Uint16 busShort;
        Uint16 sci;
        Uint16 offCurrHit;
        Uint16 fastOvHit;
        Uint16 fastI1Hit;
        Uint16 fastI2Hit;
        Uint16 fastImbHit;
        volatile Uint16 fastRec[FAST_BIT_COUNT];
        Uint16 cmpssVRec;
        Uint16 cmpssI1Rec;
        Uint16 cmpssI2Rec;
        Uint16 u16LockPrev;
    } run;
    struct
    {
        RetryLock_t shortLock;
        RetryLock_t cmpssLock;
    } retry;
} FaultCnt_t;

volatile FaultData_t stFault;
FaultCnt_t           stFaultCnt;

/* ==========================================================================
 * §2 RAM 执行函数（ISR 热路径）
 *   带 #pragma CODE_SECTION(..., ".TI.ramfunc") 的函数全部集中在本节：
 *   链接到 RAM 执行，避免 Flash 等待周期与在线升级期间取指失败。
 * ========================================================================== */

/**
 * @brief 浮点绝对值（三目写法在任何优化级别均只生成内联指令）
 */
#pragma CODE_SECTION(Fault_Abs, ".TI.ramfunc");
static inline float Fault_Abs(float x)
{
    return (x >= 0.0f) ? x : -x;
}

/**
 * @brief 连续 N 拍确认（ISR / 5ms 共用）
 * @return 1 = 已连续满足 u16N 拍
 */
#pragma CODE_SECTION(Fault_Confirm, ".TI.ramfunc");
static inline Uint16 Fault_Confirm(volatile Uint16 *pu16Cnt, Uint16 u16Hit, Uint16 u16N)
{
    if (u16Hit == 0U)
    {
        *pu16Cnt = 0U;
        return 0U;
    }
    if (*pu16Cnt < u16N)
    {
        (*pu16Cnt)++;
    }
    return (Uint16)(*pu16Cnt >= u16N);
}

/**
 * @brief 两通道电流不平衡判据（ISR 触发与 5ms 恢复共用）
 *        |I0+I1| > FAULT_CH_IMB_A 且 |I0-I1| > max(FAULT_CH_IMB_A, 0.3×|I0+I1|)
 */
#pragma CODE_SECTION(Fault_ImbHit, ".TI.ramfunc");
static inline Uint16 Fault_ImbHit(float i0, float i1)
{
    float f32Sum  = Fault_Abs(i0 + i1);
    float f32Diff = Fault_Abs(i0 - i1);
    float f32Lim  = FAULT_CH_IMB_RATIO * f32Sum;

    if (f32Lim < FAULT_CH_IMB_A)
    {
        f32Lim = FAULT_CH_IMB_A;
    }

    return (Uint16)((f32Sum > FAULT_CH_IMB_A) && (f32Diff > f32Lim));
}

/**
 * @brief 快保护（ISR）：只置位不恢复；母线过压只看继电器 VM 瞬时值（入参由 ISR 传
 *        VmBusVolt.f32IsrResult）；5ms 恢复用同一通道的 10ms 滤波值；开路测试不屏蔽。
 *        依赖 C28x 对 16 位全局字的 OR/AND 单指令原子读改写与 5ms 恢复共享 stFault.fast。
 */
#pragma CODE_SECTION(Fault_FastProtect, ".TI.ramfunc");
void Fault_FastProtect(float f32BusCurr0, float f32BusCurr1,float f32Vbus)
{

    if (Fault_Confirm(&stFaultCnt.run.fastOvHit,
                      (Uint16)(f32Vbus > FAULT_FAST_OV),
                      FAULT_FAST_OV_N) != 0U)
    {
        stFault.fast |= FAST_MASK_VOUT_OV;
        stFaultCnt.run.fastRec[FAST_BIT_VOUT_OV] = 0U;
    }
    if (Fault_Confirm(&stFaultCnt.run.fastI1Hit,
                      (Uint16)(Fault_Abs(f32BusCurr0) > FAULT_CH_OC),
                      FAULT_FAST_OC_N) != 0U)
    {
        stFault.fast |= FAST_MASK_I1_OV;
        stFaultCnt.run.fastRec[FAST_BIT_I1_OV] = 0U;
    }
    if (Fault_Confirm(&stFaultCnt.run.fastI2Hit,
                      (Uint16)(Fault_Abs(f32BusCurr1) > FAULT_CH_OC),
                      FAULT_FAST_OC_N) != 0U)
    {
        stFault.fast |= FAST_MASK_I2_OV;
        stFaultCnt.run.fastRec[FAST_BIT_I2_OV] = 0U;
    }
    if (Fault_Confirm(&stFaultCnt.run.fastImbHit,
                      Fault_ImbHit(f32BusCurr0, f32BusCurr1),
                      FAULT_FAST_IMB_N) != 0U)
    {
        stFault.fast |= FAST_MASK_IBALANCE;
        stFaultCnt.run.fastRec[FAST_BIT_IBALANCE] = 0U;
    }
}

/* ==========================================================================
 * §3 状态判据
 * ========================================================================== */

/**
 * @brief 判断是否处于“工作正常、实际可发波”（仅母线欠压使用）
 * @return 1 = 闭环且该向未禁，且充电侧 ISR 确认有脉冲能力；0 = 未发波
 */
static Uint16 Fault_PwmOn(void)
{
    Uint16 u16Seal = RunState.curLowSealing;

    if ((RunState.state != SM_STATE_RAMP_CLOSE) &&
        (RunState.state != SM_STATE_RUN))
    {
        return 0U;
    }
    if (RunState.workMode == (Uint16)WORK_MODE_CHARGE)
    {
        return (Uint16)(((u16Seal & SM_SEAL_NO_CHARGE) == 0U) &&
                        (g_stDabIsrState.u16ChgPwmActive != 0U));
    }
    if (RunState.workMode == (Uint16)WORK_MODE_DISCHARGE)
    {
        return (Uint16)((u16Seal & SM_SEAL_NO_DISCHARGE) == 0U);
    }
    return 0U;
}

/* ==========================================================================
 * §4 通用去抖
 * ========================================================================== */

/**
 * @brief 迟滞置位/清除（单计数器异或翻转）：
 *        未置位时 hit 连续 tripN 拍置位；已置位时 !hit && recHit 连续 recN 拍清位。
 */
static void Fault_Level(Uint16 *pu16Cnt, Uint16 u16Bit,
                        Uint16 u16Hit, Uint16 u16TripN,
                        Uint16 u16RecHit, Uint16 u16RecN)
{
    Uint32 u32Mask = FAULT_MASK(u16Bit);
    Uint16 u16Set  = (Uint16)((stFault.slow & u32Mask) != 0UL);
    Uint16 u16Act  = (u16Set == 0U) ? u16Hit : (Uint16)((u16Hit == 0U) && (u16RecHit != 0U));
    Uint16 u16Lim  = (u16Set == 0U) ? u16TripN : u16RecN;

    if (u16Act == 0U)
    {
        *pu16Cnt = 0U;
    }
    else if (++(*pu16Cnt) >= u16Lim)
    {
        *pu16Cnt = 0U;
        stFault.slow ^= u32Mask;
    }
}

/**
 * @brief 固定窗口重复触发锁定：上升沿计次，达 hitMax 立即置 alm；窗口到期未满则清零重来。
 */
static void Fault_RetryLockTick(RetryLock_t *pLock, Uint16 u16Rising,
                                Uint16 u16HitMax, Uint16 u16WinMax,
                                Uint16 u16AlmMask)
{
    if (u16Rising != 0U)
    {
        pLock->hits++;
    }

    if (pLock->hits >= u16HitMax)
    {
        stFault.alm |= u16AlmMask;
        pLock->hits = 0U;
        pLock->win  = 0U;
    }
    else if ((pLock->hits != 0U) && (++pLock->win >= u16WinMax))
    {
        pLock->hits = 0U;
        pLock->win  = 0U;
    }
}

/**
 * @brief 通用"连续 N 拍满足才恢复"计数（快保护 / CMPSS 恢复共用）
 */
static Uint16 Fault_RecCntTick(volatile Uint16 *pu16Cnt, Uint16 u16BitSet,
                               Uint16 u16Ok, Uint16 u16RecN)
{
    if (Fault_Confirm(pu16Cnt, (Uint16)((u16BitSet != 0U) && (u16Ok != 0U)), u16RecN) == 0U)
    {
        return 0U;
    }
    *pu16Cnt = 0U;
    return 1U;
}

/* ==========================================================================
 * §5 慢保护判据（5ms）
 * ========================================================================== */

/**
 * @brief 慢保护（5ms 拍，基于 10ms/100ms 滤波值）
 *        顺序敏感：④ 母线欠压依赖前序故障字的最终态。
 */
static void Fault_SlowProtect5ms(float f32Vbus, float f32I0, float f32I1, Uint16 u16OpenTest)
{
    float  f32Iabs     = Fault_Abs(f32I0 + f32I1);
    float  f32Islow    = Fault_Abs(stADC.BusCurr1.Filt.f32Filt100ms +
                                   stADC.BusCurr2.Filt.f32Filt100ms);
    float  f32Lim      = __fmin(__fmax(Fault_Abs(Bus.f32Ref) - 2.0f, FAULT_I_SHORT_LO),
                                FAULT_I_SHORT_HI);
    Uint16 u16ShortHit = (Uint16)((SM_IsClosedLoopActive() != 0U) &&
                                  (f32Iabs >= f32Lim) && (f32Vbus < FAULT_VBUS_SHORT));
    Uint16 u16OtherFault;
    Uint32 u32IdleMs   = UartProto_CmdIdleMs();
    Uint16 u16SciHit   = (Uint16)((u16OpenTest == 0U) &&
                                  (u32IdleMs >= ((Uint32)FAULT_SCI_TIMEOUT_N * 5UL)));

    /* 电池欠压（开路测试不判；100ms 滤波） */
    Fault_Level(&stFaultCnt.run.batUv, FAULT_BIT_BAT_UV,
                (Uint16)((u16OpenTest == 0U) &&
                         (stADC.BatVolt.Filt.f32Filt100ms < FAULT_BAT_UV_V)),
                FAULT_N_BAT_UV,
                (Uint16)(stADC.BatVolt.Filt.f32Filt100ms > FAULT_BAT_UV_REC_V),
                FAULT_N_BAT_UV_REC);

    /* ① 母线过压 */
    Fault_Level(&stFaultCnt.run.vbusOv, FAULT_BIT_VBUS_OV,
                (Uint16)(f32Vbus > FAULT_VBUS_OV), FAULT_N_VBUS_OV,
                (Uint16)(f32Vbus <= FAULT_VBUS_OV_REC), FAULT_N_VBUS_OV_REC);

    /* ② 母线过流 */
    Fault_Level(&stFaultCnt.run.ibusOv, FAULT_BIT_IBUS_OV,
                (Uint16)(f32Iabs > FAULT_IBUS_OV), FAULT_N_IBUS_OV,
                (Uint16)(f32Iabs <= FAULT_IBUS_OV_REC), FAULT_N_IBUS_REC);

    /* ③ 功率过限 */
    Fault_Level(&stFaultCnt.run.powerOv, FAULT_BIT_POWER_OV,
                (Uint16)(stADC.f32BusPowerFilt > FAULT_POWER_OV), FAULT_N_POWER,
                (Uint16)(stADC.f32BusPowerFilt <= FAULT_POWER_REC), FAULT_N_POWER_REC);

    /* ④ 母线欠压：实际发波中 + 非开路测试 + 无其它故障 + VM<350V；无其它故障时定时恢复 */
    u16OtherFault = (Uint16)(((stFault.slow & ~(FAULT_MASK(FAULT_BIT_VBUS_UN))) != 0UL) ||
                             (stFault.fast != 0U) ||
                             (stFault.alm != 0U));
    Fault_Level(&stFaultCnt.run.vbusUn, FAULT_BIT_VBUS_UN,
                (Uint16)((Fault_PwmOn() != 0U) &&
                         (u16OpenTest == 0U) &&
                         (u16OtherFault == 0U) &&
                         (f32Vbus < FAULT_VBUS_UV)),
                FAULT_N_VBUS_UV,
                (Uint16)(u16OtherFault == 0U),
                FAULT_N_VBUS_UV_REC);

    /* ⑤ 母线短路（仅闭环态判据，防停机 0V 下零漂误报） */
    Fault_Level(&stFaultCnt.run.busShort, FAULT_BIT_SHORT,
                u16ShortHit, FAULT_N_SHORT,
                (Uint16)(u16ShortHit == 0U), FAULT_N_SHORT_REC);

    /* ⑥ 关断电流异常（停在 INIT 态后仍有电流；锁存，仅确认置位） */
    if (Fault_Confirm(&stFaultCnt.run.offCurrHit,
                      (Uint16)((RunState.state == SM_STATE_INIT) &&
                               (u16OpenTest == 0U) &&
                               (f32Islow >= FAULT_I_STUCK)),
                      FAULT_N_STUCK) != 0U)
    {
        stFault.alm |= ALM_MASK_OFF_CURR;
    }

    /* ⑦ SCI 通讯超时 */
    Fault_Level(&stFaultCnt.run.sci, FAULT_BIT_SCI,
                u16SciHit, FAULT_N_SCI,
                (Uint16)(u32IdleMs < ((Uint32)FAULT_SCI_TIMEOUT_N * 5UL)),
                FAULT_N_SCI_REC);
}

/* ==========================================================================
 * §6 支撑
 * ========================================================================== */

/**
 * @brief 单路 CMPSS soft 位恢复：条件连续 10s → 先清本路硬件锁存并复核实时状态，确认归 0 才清软件位
 */
static void Fault_CmpssRecTick(Uint32 u32Bit, Uint16 u16Ok, Uint16 *pu16Rec,
                               Uint16 u16LatchSrc)
{
    if (Fault_RecCntTick(pu16Rec,
                         (Uint16)((stFault.slow & u32Bit) != 0UL),
                         u16Ok, FAULT_N_CMPSS_REC) != 0U)
    {
        DabCmpss_ClearFilterLatchSrc(u16LatchSrc);
        if ((DabCmpss_ReadTripSrc() & u16LatchSrc) == 0U)
        {
            stFault.slow &= ~u32Bit;
        }
    }
}

/**
 * @brief 按 CMPSS 源位图置 slow.cmpss*（5ms 登记与起机复核共用）
 */
static Uint16 Fault_CmpssApplySrc(Uint16 u16Src)
{
    if ((u16Src & DAB_CMPSS_SRC_VBUS_OVP) != 0U)
    {
        stFault.slow |= FAULT_MASK(FAULT_BIT_CMPSS_VBUS_OVP);
    }
    if ((u16Src & (DAB_CMPSS_SRC_IDD1_POS | DAB_CMPSS_SRC_IDD1_NEG)) != 0U)
    {
        stFault.slow |= FAULT_MASK(FAULT_BIT_CMPSS_IDD1_OCP);
    }
    if ((u16Src & (DAB_CMPSS_SRC_IDD2_POS | DAB_CMPSS_SRC_IDD2_NEG)) != 0U)
    {
        stFault.slow |= FAULT_MASK(FAULT_BIT_CMPSS_IDD2_OCP);
    }
    return u16Src;
}

/**
 * @brief CMPSS → slow.cmpss*；按跳闸源分路恢复（V / I0 / I1），满 10s 清位
 */
static void Fault_CmpssSrc5ms(float f32Vbus, float f32I0, float f32I1)
{
    (void)Fault_CmpssApplySrc(DabCmpss_ReadTripSrc());

    Fault_CmpssRecTick(FAULT_MASK(FAULT_BIT_CMPSS_VBUS_OVP),
                       (Uint16)(f32Vbus < FAULT_CMPSS_V_REC),
                       &stFaultCnt.run.cmpssVRec,
                       DAB_CMPSS_SRC_VBUS_OVP);
    Fault_CmpssRecTick(FAULT_MASK(FAULT_BIT_CMPSS_IDD1_OCP),
                       (Uint16)(Fault_Abs(f32I0) < FAULT_CMPSS_I_REC),
                       &stFaultCnt.run.cmpssI1Rec,
                       (Uint16)(DAB_CMPSS_SRC_IDD1_POS | DAB_CMPSS_SRC_IDD1_NEG));
    Fault_CmpssRecTick(FAULT_MASK(FAULT_BIT_CMPSS_IDD2_OCP),
                       (Uint16)(Fault_Abs(f32I1) < FAULT_CMPSS_I_REC),
                       &stFaultCnt.run.cmpssI2Rec,
                       (Uint16)(DAB_CMPSS_SRC_IDD2_POS | DAB_CMPSS_SRC_IDD2_NEG));
}

/**
 * @brief 清全部故障字、全部计数（含 run 与 retry）及 CMPSS 滤波锁存；
 *        不清 ePWM OST（硬封波留到起机提交）。
 */
static void Fault_ClearAll(void)
{
    memset((void *)&stFault, 0, sizeof(stFault));
    memset(&stFaultCnt, 0, sizeof(stFaultCnt));
    DabCmpss_ClearFilterLatch();
}

/**
 * @brief 开路测试门控：屏蔽非过类故障并清计数，保留过压/过流/功率/CMPSS/短路/锁定。
 */
static void Fault_OpenTestGate5ms(Uint16 u16OpenTest)
{
    if (u16OpenTest == 0U)
    {
        return;
    }

    stFault.slow &= ~(FAULT_MASK(FAULT_BIT_VBUS_UN) |
                      FAULT_MASK(FAULT_BIT_BAT_UV) |
                      FAULT_MASK(FAULT_BIT_RELAY) |
                      FAULT_MASK(FAULT_BIT_SCI) |
                      FAULT_MASK(FAULT_BIT_SOFT_TO));
    stFault.alm &= (Uint16)(~ALM_MASK_OFF_CURR);

    stFaultCnt.run.vbusUn     = 0U;
    stFaultCnt.run.batUv      = 0U;
    stFaultCnt.run.sci        = 0U;
    stFaultCnt.run.offCurrHit = 0U;
}

/**
 * @brief 清故障命令：非 INIT 态保留命令位等待，进入 INIT 后清全部故障并清命令位。
 *        不与开关机/重启命令耦合；ePWM OST 留到起机流程解除。
 */
static void Fault_ClearCmd5ms(void)
{
    if ((CfgParam.SysCmd.bit.clearfaultcmd == 0U) ||
        (RunState.state != SM_STATE_INIT))
    {
        return;
    }

    CfgParam.SysCmd.bit.clearfaultcmd = 0U;
    Fault_ClearAll();
}

/**
 * @brief 清 OST 直前复核 CMPSS（须紧挨 DAB_PWM_Start，勿提前到装载前）
 * @return 1 = 可 TZCLR.OST；0 = 有硬件锁存，已置 slow，调用方不得解 OST
 */
Uint16 Fault_PreStartHwOk(void)
{
    return (Uint16)(Fault_CmpssApplySrc(DabCmpss_ReadTripSrc()) == 0U);
}

/* ==========================================================================
 * §7 快保护恢复（5ms 任务级；ISR 置位在 §2 Fault_FastProtect）
 * ========================================================================== */

/**
 * @brief 快保护恢复（5ms；各位独立，连续 FAULT_FAST_REC_N 拍安全才清位）
 */
static void Fault_FastRecover5ms(float f32Vbus, float f32I0, float f32I1)
{
    Uint16 u16OkMask = (Uint16)(
        ((f32Vbus <= FAULT_FAST_OV_REC)        ? FAST_MASK_VOUT_OV  : 0U) |
        ((Fault_Abs(f32I0) <= FAULT_CH_OC_REC) ? FAST_MASK_I1_OV    : 0U) |
        ((Fault_Abs(f32I1) <= FAULT_CH_OC_REC) ? FAST_MASK_I2_OV    : 0U) |
        ((Fault_ImbHit(f32I0, f32I1) == 0U)    ? FAST_MASK_IBALANCE : 0U));
    Uint16 u16Idx;
    Uint16 u16Mask;

    for (u16Idx = 0U; u16Idx < (Uint16)FAST_BIT_COUNT; u16Idx++)
    {
        u16Mask = FAULT_MASK16(u16Idx);
        if (Fault_RecCntTick(&stFaultCnt.run.fastRec[u16Idx],
                             (Uint16)((stFault.fast & u16Mask) != 0U),
                             (Uint16)((u16OkMask & u16Mask) != 0U),
                             FAULT_FAST_REC_N) != 0U)
        {
            stFault.fast &= (Uint16)(~u16Mask);
        }
    }
}

/* ==========================================================================
 * §8 对外接口
 * ========================================================================== */

/**
 * @brief 故障模块初始化（仅上电调用一次）
 */
void Fault_Init(void)
{
    /* Fault_ClearAll() 已清 CMPSS 滤波锁存，保留上电 ePWM OST 硬封波到起机再解 */
    Fault_ClearAll();
}

/** @brief 置位继电器故障位（slow.relay） */
void Fault_SetRelay(void)
{
    stFault.slow |= FAULT_MASK(FAULT_BIT_RELAY);
}

/** @brief 清除继电器故障位（slow.relay） */
void Fault_ClearRelay(void)
{
    stFault.slow &= ~FAULT_MASK(FAULT_BIT_RELAY);
}

/** @brief 置位软起超时（由 SM_SoftStartTick5ms 调用；开路测试期间旁路） */
void Fault_SetSoftTo(void)
{
    if (!FAULT_OPEN_TEST())
    {
        stFault.slow |= FAULT_MASK(FAULT_BIT_SOFT_TO);
    }
}

/** @brief 清除软起超时位 */
void Fault_ClearSoftTo(void)
{
    stFault.slow &= ~FAULT_MASK(FAULT_BIT_SOFT_TO);
}

/**
 * @brief 5ms 故障主流程（顺序敏感，见说明文档 §2.2；软起计时在 SM）
 */
void Fault_Detect5ms(void)
{
    float  f32Vbus     = stADC.VmBusVolt.Filt.f32Filt10ms;
    float  f32I0       = stADC.BusCurr1.Filt.f32Filt10ms;
    float  f32I1       = stADC.BusCurr2.Filt.f32Filt10ms;
    Uint16 u16OpenTest = (Uint16)FAULT_OPEN_TEST();
    Uint16 u16LockCur;

    Fault_OpenTestGate5ms(u16OpenTest);
    Fault_ClearCmd5ms();
    Fault_CmpssSrc5ms(f32Vbus, f32I0, f32I1);
    Fault_SlowProtect5ms(f32Vbus, f32I0, f32I1, u16OpenTest);

    /* 上升沿计次 → shortLock / cmpssLock（过流两位整组，不含 bit7） */
    u16LockCur = (Uint16)(stFault.slow & (FAULT_MASK(FAULT_BIT_SHORT) | CMPSS_OCP_MASK));
    Fault_RetryLockTick(&stFaultCnt.retry.shortLock,
                        (Uint16)(((stFaultCnt.run.u16LockPrev & FAULT_MASK16(FAULT_BIT_SHORT)) == 0U) &&
                                 ((u16LockCur & FAULT_MASK16(FAULT_BIT_SHORT)) != 0U)),
                        FAULT_N_LOCK_HIT, FAULT_N_SHORT_LOCK_WIN,
                        ALM_MASK_LOCK);
    Fault_RetryLockTick(&stFaultCnt.retry.cmpssLock,
                        (Uint16)(((stFaultCnt.run.u16LockPrev & (Uint16)CMPSS_OCP_MASK) == 0U) &&
                                 ((u16LockCur & (Uint16)CMPSS_OCP_MASK) != 0U)),
                        FAULT_N_CMPSS_LOCK_HIT, FAULT_N_CMPSS_LOCK_WIN,
                        ALM_MASK_HARDALM);
    stFaultCnt.run.u16LockPrev = u16LockCur;

    Fault_FastRecover5ms(f32Vbus, f32I0, f32I1);
}
