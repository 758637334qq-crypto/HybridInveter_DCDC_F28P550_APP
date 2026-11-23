/**
 * ============================================================================
 * 文件名称: App_Fault.c
 * 模块功能: 只做故障判断与置位（安克 FaultDetect 角色）。
 *   状态机 / ISR 直接读 stFault；软起计时在 SM，满则调 Fault_SetSoftTo。
 *   本模块是检测层，不改状态流转，不写 RunState。
 * ----------------------------------------------------------------------------
 * 阅读顺序：§1 类型/变量 → §2 RAM → §3 快照 → §4 去抖 → §5 判据
 *           → §6 支撑 → §7 快恢复 → §8 对外接口
 * ============================================================================
 */
#include "Includes.h"          /* 全局头文件：含 stADC、RunState、SysCmd、Bus 等外部声明 */
#include "Interrupt_CtrlDab.h"       /* ISR 发布的充电发波状态（PwmOn 判据使用） */
#include "App_DabModel.h"      /* DAB 数学模型头文件：含 uMdlAddMes 等模型输出 */

/* ==========================================================================
 * §1 本文件类型与变量（不对外；CCS 观察 stFaultCnt）
 * ========================================================================== */

typedef struct
{
    Uint16 trip;
    Uint16 rec;
} FaultTick_t;

typedef struct
{
    Uint16 hits;
    Uint16 win;
} RetryLock_t;

typedef struct
{
    float  f32Vbus;
    float  f32Iabs;
    float  f32Islow;
    float  f32I0;
    float  f32I1;
    Uint16 u16PwmOn;
    Uint16 u16OpenTest;
} FaultSnap_t;

typedef struct
{
    FaultTick_t batUv;
    FaultTick_t vbusOv;
    FaultTick_t ibusOv;
    FaultTick_t powerOv;
    FaultTick_t vbusUn;
    FaultTick_t busShort;
    FaultTick_t sci;
    Uint16      offCurrHit;
    Uint16      fastOvHit;
    Uint16      fastI1Hit;
    Uint16      fastI2Hit;
    Uint16      fastImbHit;
    Uint16      fastRec[FAST_BIT_COUNT];
    Uint16      cmpssVRec;
    Uint16      cmpssI1Rec;
    Uint16      cmpssI2Rec;
    Uint16      u16ShortPrev;
    Uint16      u16CmpssPrev;
} FaultRunCnt_t;

typedef struct
{
    RetryLock_t shortLock;
    RetryLock_t cmpssLock;
} FaultRetryCnt_t;

typedef struct
{
    FaultRunCnt_t   run;
    FaultRetryCnt_t retry;
} FaultCnt_t;

volatile FaultData_t stFault;
FaultCnt_t           stFaultCnt;

/* ==========================================================================
 * §2 RAM 执行函数（ISR 热路径）
 *   带 #pragma CODE_SECTION(..., ".TI.ramfunc") 的函数全部集中在本节：
 *   链接到 RAM 执行，避免 Flash 等待周期与在线升级期间取指失败。
 *   新增 ISR 调用函数请放本节并加本 pragma，不要散落到其它节；
 *   Fault_SetBit / Fault_ClearBit 仅在 5ms 上下文使用，无 pragma，也顺带放此。
 * ========================================================================== */

/**
 * @brief 浮点绝对值
 * @param x  输入浮点数
 * @return   |x|
 *
 * 注：本函数被 ISR 调用，随 Fault_FastProtect 一起放入 RAM 执行，
 *     避免运行期间或在线升级时从 Flash 取指。
 *     这里用比较取反而不用 fabsf：fabsf 依赖 --fp_mode=relaxed 才内联成
 *     ABSF32，Debug / -O0 构建下可能退化为 Flash 中的库调用；三目写法在
 *     任何优化级别都只生成内联指令，ISR 里更可控。
 */
#pragma CODE_SECTION(Fault_Abs, ".TI.ramfunc");
static inline float Fault_Abs(float x)
{
    return (x >= 0.0f) ? x : -x;
}

/** @brief 故障字置位（32 位；slow，5ms 上下文） */
static void Fault_SetBit(volatile Uint32 *pAll, Uint32 u32Mask)
{
    *pAll |= u32Mask;
}

/** @brief 故障字清除（32 位） */
static void Fault_ClearBit(volatile Uint32 *pAll, Uint32 u32Mask)
{
    *pAll &= ~u32Mask;
}

/** @brief 故障字置位（16 位；fast / alm） */
static void Fault_SetBit16(volatile Uint16 *pAll, Uint16 u16Mask)
{
    *pAll |= u16Mask;
}

/** @brief 故障字清除（16 位） */
static void Fault_ClearBit16(volatile Uint16 *pAll, Uint16 u16Mask)
{
    *pAll &= (Uint16)(~u16Mask);
}

/**
 * @brief 连续 N 拍确认（ISR 用）
 * @param pu16Cnt  连续命中计数器
 * @param u16Hit   本拍条件是否成立
 * @param u16N     确认拍数
 * @return 1 = 已连续满足 N 拍
 *
 * 注：ISR 调用，放 RAM 执行（不在 .TI.ramfunc 就会跳回 Flash 取指）。
 */
#pragma CODE_SECTION(Fault_Confirm, ".TI.ramfunc");
static inline Uint16 Fault_Confirm(Uint16 *pu16Cnt, Uint16 u16Hit, Uint16 u16N)
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
 * @brief 两通道电流不平衡判据（供 ISR 触发与 5ms 恢复共用，避免两处公式漂移）
 * @return 1 = 不平衡命中
 *
 * 判据：|I0+I1| > FAULT_CH_IMB_A 且 |I0-I1| > max(FAULT_CH_IMB_A, 0.3×|I0+I1|)
 * 注：ISR 与 5ms 共用，同样放 RAM 执行
 */
#pragma CODE_SECTION(Fault_ImbHit, ".TI.ramfunc");
static inline Uint16 Fault_ImbHit(float i0, float i1)
{
    float f32Sum  = Fault_Abs(i0 + i1);        /* |I0+I1|：总电流 */
    float f32Diff = Fault_Abs(i0 - i1);        /* |I0-I1|：不平衡量 */
    float f32Lim  = FAULT_CH_IMB_RATIO * f32Sum; /* 比例阈值 */

    /* 绝对下限用三目取大，避免调用 fmaxf 库函数（ISR 内只允许内联指令）*/
    if (f32Lim < FAULT_CH_IMB_A)
    {
        f32Lim = FAULT_CH_IMB_A;
    }

    return (Uint16)((f32Sum > FAULT_CH_IMB_A) && (f32Diff > f32Lim));
}

/**
 * @brief 快保护（ISR）：只置位不恢复；电压取 VM，与 5ms 同源；开路测试不屏蔽。
 *        CMPSS 硬件跳闸由 5ms Fault_CmpssSrc5ms 登记。
 */
#pragma CODE_SECTION(Fault_FastProtect, ".TI.ramfunc");
void Fault_FastProtect(float f32BusCurr0, float f32BusCurr1)
{
    float f32Vbus = stADC.VmBusVolt.f32IsrResult;

    if (Fault_Confirm(&stFaultCnt.run.fastOvHit,
                      (Uint16)(f32Vbus > DAB_DISCHARGE_BUS_OVP_VOLT),
                      FAULT_FAST_OV_N) != 0U)
    {
        stFault.fast |= FAST_MASK_VOUT_OV;         /* 锁存过压故障 */
    }

    /* ---- ② 通道 0 过流软件快保护 ---- */
    if (Fault_Confirm(&stFaultCnt.run.fastI1Hit,
                      (Uint16)(Fault_Abs(f32BusCurr0) > FAULT_CH_OC),
                      FAULT_FAST_OC_N) != 0U)
    {
        stFault.fast |= FAST_MASK_I1_OV;           /* 锁存通道 0 过流 */
    }

    /* ---- ③ 通道 1 过流软件快保护 ---- */
    if (Fault_Confirm(&stFaultCnt.run.fastI2Hit,
                      (Uint16)(Fault_Abs(f32BusCurr1) > FAULT_CH_OC),
                      FAULT_FAST_OC_N) != 0U)
    {
        stFault.fast |= FAST_MASK_I2_OV;           /* 锁存通道 1 过流 */
    }

    /* ---- ④ 两通道电流不平衡（判据与 5ms 恢复共用 Fault_ImbHit）---- */
    if (Fault_Confirm(&stFaultCnt.run.fastImbHit,
                      Fault_ImbHit(f32BusCurr0, f32BusCurr1),
                      FAULT_FAST_IMB_N) != 0U)
    {
        stFault.fast |= FAST_MASK_IBALANCE;        /* 锁存不平衡故障 */
    }
}

/* ==========================================================================
 * §3 状态快照
 * ========================================================================== */

/**
 * @brief 当前工作方向是否被单向禁波（ISR 会 FORCE_LOW）
 */
static Uint16 Fault_DirSealed(void)
{
    if ((RunState.workMode == (Uint16)WORK_MODE_DISCHARGE) &&
        ((RunState.curLowSealing & SM_SEAL_NO_DISCHARGE) != 0U))
    {
        return 1U;
    }
    if ((RunState.workMode == (Uint16)WORK_MODE_CHARGE) &&
        ((RunState.curLowSealing & SM_SEAL_NO_CHARGE) != 0U))
    {
        return 1U;
    }
    return 0U;
}

/**
 * @brief 判断是否处于“工作正常、实际可发波”
 * @return 1 = 闭环且该向未禁，且充电侧 ISR 确认有脉冲能力；0 = 未发波
 *
 * 欠压/短路用此口径：仅状态机 RUN/CLOSE 不够——充电压差不足、打嗝停波时
 * g_stDabIsrState.u16ChgPwmActive=0，不算工作中，不累计欠压。
 */
static Uint16 Fault_PwmOn(void)
{
    if ((RunState.state != SM_STATE_RAMP_CLOSE) &&
        (RunState.state != SM_STATE_RUN))
    {
        return 0U;
    }
    if (Fault_DirSealed() != 0U)
    {
        return 0U;
    }
    /* 充电：须 ISR 实际允许开通（压差/打嗝门通过）；放电无此门，状态即可 */
    if (RunState.workMode == (Uint16)WORK_MODE_CHARGE)
    {
        return (g_stDabIsrState.u16ChgPwmActive != 0U) ? 1U : 0U;
    }
    if (RunState.workMode == (Uint16)WORK_MODE_DISCHARGE)
    {
        return 1U;
    }
    return 0U;
}

/**
 * @brief 判断是否处于"开路测试"模式
 * @return 1 = 开路测试中，0 = 正常模式
 *
 * 说明：运行时命令 SysCmd 与配置参数 CfgParam.SysCmd 任一置位即生效。
 */
static Uint16 Fault_OpenTest(void)
{
    return (Uint16)((SysCmd.bit.openTest != 0U) ||            /* 运行时命令 */
                    (CfgParam.SysCmd.bit.openTest != 0U));    /* 配置参数 */
}

/**
 * @brief 采集本拍 ADC 特征值与状态标志（一次性读取，保证同拍口径一致）
 * @note  VM/I0/I1 用 10ms 滤波（5ms 慢保护口径）；关断电流仍用 100ms
 */
static void Fault_TakeSnap(FaultSnap_t *p)
{
    float f32Vbus = stADC.VmBusVolt.Filt.f32Filt10ms;
    float f32I0   = stADC.BusCurr1.Filt.f32Filt10ms;
    float f32I1   = stADC.BusCurr2.Filt.f32Filt10ms;

    p->f32Vbus  = f32Vbus;
    p->f32I0    = f32I0;
    p->f32I1    = f32I1;
    p->f32Iabs  = Fault_Abs(f32I0 + f32I1);
    p->f32Islow = Fault_Abs(
        stADC.BusCurr1.Filt.f32Filt100ms + stADC.BusCurr2.Filt.f32Filt100ms);

    p->u16PwmOn    = Fault_PwmOn();
    p->u16OpenTest = Fault_OpenTest();
}

/* ==========================================================================
 * §4 通用去抖
 * ========================================================================== */

/**
 * @brief 迟滞置位/清除：hit 连续 tripN 置位；hit=0 且 recHit 连续 recN 清位（recN=0 不自动恢复）。
 *        tripN 须 ≥1；hit 成立时不同拍做恢复。细则见故障模块说明 §2.3。
 */
static void Fault_Level(volatile Uint32 *pAll, FaultTick_t *pTick, Uint16 u16Bit,
                        Uint16 u16Hit, Uint16 u16TripN,
                        Uint16 u16RecHit, Uint16 u16RecN)
{
    Uint32 u32Mask = FAULT_MASK(u16Bit);

    /* ---- 触发方向：连续成立才累计 ---- */
    if (u16Hit != 0U)
    {
        pTick->rec = 0U;                   /* 触发中不累计恢复，须重新连续计时 */
        if (pTick->trip < u16TripN)
        {
            pTick->trip++;
        }
        if (pTick->trip >= u16TripN)
        {
            pTick->trip = u16TripN;        /* 钳位，防止溢出 */
            Fault_SetBit(pAll, u32Mask);
        }
    }
    else
    {
        pTick->trip = 0U;                  /* 立即清零（要求连续） */
    }

    /* ---- 恢复方向 ---- */
    /* hit 成立时不做恢复判定：避免触发/恢复阈值交界处同拍置位又清除 */
    if ((u16Hit == 0U) && (u16RecN != 0U) && (u16RecHit != 0U) &&
        ((*pAll & u32Mask) != 0UL))
    {
        if (pTick->rec < u16RecN)
        {
            pTick->rec++;
        }
        if (pTick->rec >= u16RecN)
        {
            pTick->rec = 0U;
            Fault_ClearBit(pAll, u32Mask);
        }
    }
    else
    {
        pTick->rec = 0U;
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
        pLock->hits++;                     /* 达门限即清零，最大只记 u16HitMax */
    }

    if (pLock->hits >= u16HitMax)          /* 达门限立即锁定，不等窗口结束 */
    {
        Fault_SetBit16(&stFault.alm, u16AlmMask);
        pLock->hits = 0U;
        pLock->win  = 0U;
    }
    else if ((pLock->hits != 0U) && (++pLock->win >= u16WinMax))
    {
        pLock->hits = 0U;                  /* 窗口到期 → 重新计数 */
        pLock->win  = 0U;
    }
}

/**
 * @brief 通用"连续 N 拍满足才恢复"计数（快保护 / CMPSS 恢复共用）
 *
 * @param pu16Cnt   恢复计数器
 * @param u16BitSet 是否在恢复太 (1=已置位)
 * @param u16Ok     恢复条件是否成立 (1=安全, 0=仍危险)
 * @param u16RecN   恢复时间
 * @return 1 = 已连续满足 u16RecN 拍，调用方执行清位动作
 *
 * 三段式语义：位未置位或条件不满足 → 计数清零、保持锁存；
 * 连续满足 u16RecN 拍 → 返回 1。清位方式（16/32 位故障字、附加动作）
 * 由调用方决定，本函数只做计数。
 */
static Uint16 Fault_RecCntTick(Uint16 *pu16Cnt, Uint16 u16BitSet,
                               Uint16 u16Ok, Uint16 u16RecN)
{
    if ((u16BitSet == 0U) || (u16Ok == 0U))
    {
        *pu16Cnt = 0U;                     /* 无需恢复或仍危险：重新计时，保持锁存 */
        return 0U;
    }
    if (*pu16Cnt < u16RecN)
    {
        (*pu16Cnt)++;
    }
    if (*pu16Cnt >= u16RecN)
    {
        *pu16Cnt = 0U;
        return 1U;
    }
    return 0U;
}

/* ==========================================================================
 * §5 专项判据
 * ========================================================================== */

/**
 * @brief 母线短路：|I|≥lim 且 V<SHORT；lim=|Iref|-2 钳位 [LO,HI]；不绑 PwmOn。
 *        重试锁定在 Detect5ms 按上升沿统计。
 */
static void Fault_Short(float f32IAbs, float f32Vbus)
{
    float  f32Lim = Fault_Abs(Bus.f32Ref) - 2.0f;
    Uint16 u16Hit;

    if (f32Lim > FAULT_I_SHORT_HI)
    {
        f32Lim = FAULT_I_SHORT_HI;
    }
    if (f32Lim < FAULT_I_SHORT_LO)
    {
        f32Lim = FAULT_I_SHORT_LO;
    }

    u16Hit = (Uint16)((f32IAbs >= f32Lim) && (f32Vbus < FAULT_VBUS_SHORT));
    Fault_Level(&stFault.slow, &stFaultCnt.run.busShort, FAULT_BIT_SHORT,
                u16Hit, FAULT_N_SHORT,
                (Uint16)(u16Hit == 0U), FAULT_N_SHORT_REC);
}

/**
 * @brief 慢保护 ①~⑦（5ms 拍，全部基于入口快照，互不重复采样）
 * @param p  本拍快照
 *
 * 注：顺序敏感——④ 依赖 ①~③ 的置位结果。
 */
static void Fault_SlowProtect5ms(const FaultSnap_t *p)
{
    Uint16 u16OtherFault;               /* ④ 判据：除欠压外的其它故障 */

    /* 电池欠压：开路测试不判（门控每拍清位兜底）；用 100ms 滤波值。 */
    Fault_Level(&stFault.slow, &stFaultCnt.run.batUv, FAULT_BIT_BAT_UV,
                (Uint16)((p->u16OpenTest == 0U) &&
                         (stADC.BatVolt.Filt.f32Filt100ms < FAULT_BAT_UV_V)),
                FAULT_N_BAT_UV,
                (Uint16)(stADC.BatVolt.Filt.f32Filt100ms > FAULT_BAT_UV_REC_V),
                FAULT_N_BAT_UV_REC);

    /* ---- ① 母线过压 ---- */
    Fault_Level(&stFault.slow, &stFaultCnt.run.vbusOv, FAULT_BIT_VBUS_OV,
                (Uint16)(p->f32Vbus > FAULT_VBUS_OV),        /* hit: 过压 */
                FAULT_N_VBUS_OV,                              /* 触发拍数 */
                (Uint16)(p->f32Vbus <= FAULT_VBUS_OV_REC),    /* recHit: 安全 */
                FAULT_N_VBUS_OV_REC);                         /* 恢复拍数 */

    /* ---- ② 母线过流 ---- */
    Fault_Level(&stFault.slow, &stFaultCnt.run.ibusOv, FAULT_BIT_IBUS_OV,
                (Uint16)(p->f32Iabs > FAULT_IBUS_OV),
                FAULT_N_IBUS_OV,
                (Uint16)(p->f32Iabs <= FAULT_IBUS_OV_REC),
                FAULT_N_IBUS_REC);

    /* ---- ③ 功率过限 ---- */
    Fault_Level(&stFault.slow, &stFaultCnt.run.powerOv, FAULT_BIT_POWER_OV,
                (Uint16)(stADC.f32BusPowerFilt > FAULT_POWER_OV),
                FAULT_N_POWER,
                (Uint16)(stADC.f32BusPowerFilt <= FAULT_POWER_REC),
                FAULT_N_POWER_REC);

    /* ---- ④ 母线欠压 ---
     * 触发：实际发波中（Fault_PwmOn）+ 非开路测试 + 无其它故障 + Vbus < 350V，连续 15s。
     *   充电压差不足/打嗝停波：PwmOn=0，不累计。
     * 恢复：无其它故障时无条件定时清位（recHit=无其它故障，无电压回差），连续 15s。
     *   有其它故障在位时不恢复欠压，避免与其它保护抢清位。*/
    u16OtherFault = (Uint16)(((stFault.slow &
                               ~(FAULT_MASK(FAULT_BIT_VBUS_UN))) != 0UL) ||
                             (stFault.fast != 0U) ||
                             (stFault.alm != 0U));

    Fault_Level(&stFault.slow, &stFaultCnt.run.vbusUn, FAULT_BIT_VBUS_UN,
                (Uint16)((p->u16PwmOn != 0U) &&
                         (p->u16OpenTest == 0U) &&
                         (u16OtherFault == 0U) &&
                         (p->f32Vbus < FAULT_VBUS_UV)),
                FAULT_N_VBUS_UV,
                (Uint16)(u16OtherFault == 0U),                 /* 无其它故障 → 无条件计时恢复 */
                FAULT_N_VBUS_UV_REC);

    /* ---- ⑤ 母线短路：仅 I+V；PwmOn 只给欠压用 ---- */
    Fault_Short(p->f32Iabs, p->f32Vbus);

    /* ---- ⑥ 关断电流异常（停机后仍有电流；锁存，仅确认置位）---- */
    if (Fault_Confirm(&stFaultCnt.run.offCurrHit,
                      (Uint16)((FAULT_RUN_REQ() == 0U) &&
                               (p->f32Islow >= FAULT_I_STUCK)),
                      FAULT_N_STUCK) != 0U)
    {
        Fault_SetBit16(&stFault.alm, ALM_MASK_OFF_CURR);
    }

    /* ---- ⑦ SCI：开路测试不判；空闲 ≥ FAULT_SCI_TIMEOUT_N×5ms 判掉线 ---- */
    {
        Uint16 u16IdleMs = UartProto_CmdIdleMs();
        Uint16 u16SciHit = (Uint16)((p->u16OpenTest == 0U) &&
                                    (u16IdleMs >= (FAULT_SCI_TIMEOUT_N * 5U)));
        Fault_Level(&stFault.slow, &stFaultCnt.run.sci, FAULT_BIT_SCI,
                    u16SciHit, FAULT_N_SCI,
                    (Uint16)(u16IdleMs < (FAULT_SCI_TIMEOUT_N * 5U)),
                    FAULT_N_SCI_REC);
    }
}

/* ==========================================================================
 * §6 支撑
 * ========================================================================== */

/**
 * @brief 单路 CMPSS soft 位恢复：条件连续 10s → 清该位 + 仅清本路锁存
 * @param u16LatchSrc  DAB_CMPSS_SRC_*；本路恢复不得清他路（防漏记）
 */
static void Fault_CmpssRecTick(Uint32 u32Bit, Uint16 u16Ok, Uint16 *pu16Rec,
                               Uint16 u16LatchSrc)
{
    if (Fault_RecCntTick(pu16Rec,
                         (Uint16)((stFault.slow & u32Bit) != 0UL),
                         u16Ok, FAULT_N_CMPSS_REC) != 0U)
    {
        Fault_ClearBit(&stFault.slow, u32Bit);
        /* 只清本路比较器锁存，保留 ePWM OST；他路锁存不动 */
        DabCmpss_ClearFilterLatchSrc(u16LatchSrc);
    }
}

/**
 * @brief 按 CMPSS 源位图置 slow.cmpss*（5ms 登记与起机复核共用）
 * @return 源位图（便于调用方判断是否有跳闸）
 */
static Uint16 Fault_CmpssApplySrc(Uint16 u16Src)
{
    if ((u16Src & DAB_CMPSS_SRC_VBUS_OVP) != 0U)
    {
        Fault_SetBit(&stFault.slow, FAULT_MASK(FAULT_BIT_CMPSS_VBUS_OVP));
    }
    if ((u16Src & (DAB_CMPSS_SRC_IDD1_POS | DAB_CMPSS_SRC_IDD1_NEG)) != 0U)
    {
        Fault_SetBit(&stFault.slow, FAULT_MASK(FAULT_BIT_CMPSS_IDD1_OCP));
    }
    if ((u16Src & (DAB_CMPSS_SRC_IDD2_POS | DAB_CMPSS_SRC_IDD2_NEG)) != 0U)
    {
        Fault_SetBit(&stFault.slow, FAULT_MASK(FAULT_BIT_CMPSS_IDD2_OCP));
    }
    return u16Src;
}

/**
 * @brief CMPSS → slow.cmpss*；按跳闸源分路恢复（V / I0 / I1），满 10s 清位
 */
static void Fault_CmpssSrc5ms(const FaultSnap_t *p)
{
    (void)Fault_CmpssApplySrc(DabCmpss_ReadTripSrc());

    /* CMPSS 位按自身条件恢复；与 alm.hardalm 解耦（锁定只管拦起机） */
    Fault_CmpssRecTick(FAULT_MASK(FAULT_BIT_CMPSS_VBUS_OVP),
                       (Uint16)(p->f32Vbus < FAULT_CMPSS_V_REC),
                       &stFaultCnt.run.cmpssVRec,
                       DAB_CMPSS_SRC_VBUS_OVP);
    Fault_CmpssRecTick(FAULT_MASK(FAULT_BIT_CMPSS_IDD1_OCP),
                       (Uint16)(Fault_Abs(p->f32I0) < FAULT_CMPSS_I_REC),
                       &stFaultCnt.run.cmpssI1Rec,
                       (Uint16)(DAB_CMPSS_SRC_IDD1_POS | DAB_CMPSS_SRC_IDD1_NEG));
    Fault_CmpssRecTick(FAULT_MASK(FAULT_BIT_CMPSS_IDD2_OCP),
                       (Uint16)(Fault_Abs(p->f32I1) < FAULT_CMPSS_I_REC),
                       &stFaultCnt.run.cmpssI2Rec,
                       (Uint16)(DAB_CMPSS_SRC_IDD2_POS | DAB_CMPSS_SRC_IDD2_NEG));
}

/**
 * @brief 清故障字与 CMPSS 滤波锁存；不清 ePWM OST（硬封波留到起机提交）。
 *        retry 组刻意保留，仅 Fault_Init 清零。
 */
static void Fault_ClearAll(void)
{
    memset((void *)&stFault, 0, sizeof(stFault));
    memset(&stFaultCnt.run, 0, sizeof(stFaultCnt.run));
    DabCmpss_ClearFilterLatch();
}

/**
 * @brief 开路测试门控：屏蔽 run / 保留 over。
 *
 * 母线欠压、电池欠压、继电器、SCI、软起超时属"非过"类，测试期间清位并清计数；
 * 关断电流（alm 字）测试期间同样放行；
 * over（过压/过流/过功率/CMPSS/短路等）与 alm.lock/hardalm 一律不动。
 */
static void Fault_OpenTestGate5ms(const FaultSnap_t *p)
{
    if (p->u16OpenTest == 0U)
    {
        return;
    }

    Fault_ClearBit(&stFault.slow,
                   FAULT_MASK(FAULT_BIT_VBUS_UN) |
                   FAULT_MASK(FAULT_BIT_BAT_UV) |
                   FAULT_MASK(FAULT_BIT_RELAY) |
                   FAULT_MASK(FAULT_BIT_SCI) |
                   FAULT_MASK(FAULT_BIT_SOFT_TO));
    Fault_ClearBit16(&stFault.alm, ALM_MASK_OFF_CURR);

    stFaultCnt.run.vbusUn.trip  = 0U;
    stFaultCnt.run.vbusUn.rec   = 0U;
    stFaultCnt.run.batUv.trip   = 0U;
    stFaultCnt.run.batUv.rec    = 0U;
    stFaultCnt.run.sci.trip     = 0U;
    stFaultCnt.run.sci.rec      = 0U;
    stFaultCnt.run.offCurrHit   = 0U;
}

/**
 * @brief 清故障命令：仅停机确认后执行（关机请求 + 已在 INIT）。
 *
 * 运行中或开机请求仍为 1 时丢弃命令，避免清 soft/lock/hardalm 后被 ISR/状态机
 * 按旧 RUN 继续控，或清完立刻自动起机。OST 不在此解除。
 *
 * 运行请求直接读 CfgParam（与 clearfaultcmd 同源），口径同 SM_SyncSysCmd 的
 * shutdown 合成式；不回调状态机同步，避免故障模块反向依赖 SM。
 * SysCmd 镜像仍由 UART/CAN 写命令时与 2ms UpdateShutdownFlag 负责刷新。
 */
static void Fault_ClearCmd5ms(void)
{
    Uint16 u16RunReq;

    if (CfgParam.SysCmd.bit.clearfaultcmd == 0U)
    {
        return;
    }

    /* 消费命令位：通信写 CfgParam.SysCmd 与本函数同在主循环超循环内顺序执行，
     * 不存在抢占，故无需关中断 */
    CfgParam.SysCmd.bit.clearfaultcmd = 0U;

    /* 与 SM_SyncSysCmd：shutdown = onoffcmd || openTest */
    u16RunReq = (Uint16)((CfgParam.SysCmd.bit.onoffcmd != 0U) ||
                         (CfgParam.SysCmd.bit.openTest != 0U));

    /* 停机门槛：无运行请求，且状态机已回 INIT */
    if ((u16RunReq != 0U) || (RunState.state != SM_STATE_INIT))
    {
        return;
    }

    if ((stFault.slow != 0UL) || (stFault.fast != 0U) ||
        (stFault.alm != 0U))
    {
        Fault_ClearAll();
    }
}

/**
 * @brief 清 OST 直前复核 CMPSS（须紧挨 DAB_PWM_Start，勿提前到装载前）
 * @return 1 = 可 TZCLR.OST；0 = 有硬件锁存，已置 soft，调用方不得解 OST
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
    /* 各位按自身安全条件恢复；与 alm.lock / hardalm 解耦（锁定只管拦起机） */
    if (Fault_RecCntTick(&stFaultCnt.run.fastRec[FAST_BIT_VOUT_OV],
                         (Uint16)((stFault.fast & FAST_MASK_VOUT_OV) != 0U),
                         (Uint16)(f32Vbus <= FAULT_FAST_OV_REC),
                         FAULT_FAST_REC_N) != 0U)
    {
        stFault.fast &= (Uint16)(~FAST_MASK_VOUT_OV);
    }
    if (Fault_RecCntTick(&stFaultCnt.run.fastRec[FAST_BIT_I1_OV],
                         (Uint16)((stFault.fast & FAST_MASK_I1_OV) != 0U),
                         (Uint16)(Fault_Abs(f32I0) <= FAULT_CH_OC_REC),
                         FAULT_FAST_REC_N) != 0U)
    {
        stFault.fast &= (Uint16)(~FAST_MASK_I1_OV);
    }
    if (Fault_RecCntTick(&stFaultCnt.run.fastRec[FAST_BIT_I2_OV],
                         (Uint16)((stFault.fast & FAST_MASK_I2_OV) != 0U),
                         (Uint16)(Fault_Abs(f32I1) <= FAULT_CH_OC_REC),
                         FAULT_FAST_REC_N) != 0U)
    {
        stFault.fast &= (Uint16)(~FAST_MASK_I2_OV);
    }
    if (Fault_RecCntTick(&stFaultCnt.run.fastRec[FAST_BIT_IBALANCE],
                         (Uint16)((stFault.fast & FAST_MASK_IBALANCE) != 0U),
                         (Uint16)(Fault_ImbHit(f32I0, f32I1) == 0U),
                         FAULT_FAST_REC_N) != 0U)
    {
        stFault.fast &= (Uint16)(~FAST_MASK_IBALANCE);
    }
}

/* ==========================================================================
 * §8 对外接口
 * ========================================================================== */

/**
 * @brief 故障模块初始化（仅上电调用一次）
 *
 * 经 Fault_ClearAll 清三整字故障、run 计数与 CMPSS 滤波锁存，另行清零
 * retry 锁定组，并清 ePWM Trip 状态解除硬件封波。
 * 与运行期清故障（Fault_ClearAll）的区别：只有上电才允许清 retry 组与
 * ePWM OST —— 运行期"反复跳闸 → 清故障 → 再跳闸"无法绕过锁定保护，
 * 而上电时系统尚未起机，解除 OST 安全。
 */
void Fault_Init(void)
{
    Fault_ClearAll();                                            /* 清 stFault + stFaultCnt.run + CMPSS 滤波锁存 */
    memset(&stFaultCnt.retry, 0, sizeof(stFaultCnt.retry));      /* retry 组仅上电清零 */
    DabCmpss_ClearTrip();   /* 仅上电清 ePWM TZ；运行期清故障保留 OST 到起机提交 */
}

/**
 * @brief 置位继电器故障位（slow.relay，幂等锁存）
 *
 * 继电器的检测与异常判定在继电器控制模块内完成，异常时经本接口上报；
 * 本模块不自动恢复该位，清位走 Fault_ClearRelay 或清故障命令。
 */
void Fault_SetRelay(void)
{
    Fault_SetBit(&stFault.slow, FAULT_MASK(FAULT_BIT_RELAY));
}

/**
 * @brief 清除继电器故障位（slow.relay），与 Fault_SetRelay 配对使用
 */
void Fault_ClearRelay(void)
{
    Fault_ClearBit(&stFault.slow, FAULT_MASK(FAULT_BIT_RELAY));
}

/** @brief 置位软起超时（由 SM_SoftStartTick5ms 调用） */
void Fault_SetSoftTo(void)
{
    Fault_SetBit(&stFault.slow, FAULT_MASK(FAULT_BIT_SOFT_TO));
}

/** @brief 清除软起超时位 */
void Fault_ClearSoftTo(void)
{
    Fault_ClearBit(&stFault.slow, FAULT_MASK(FAULT_BIT_SOFT_TO));
}

/**
 * @brief 5ms 故障主流程（顺序敏感，见说明文档 §2.2；软起计时在 SM）
 */
void Fault_Detect5ms(void)
{
    FaultSnap_t snap;
    Uint16      u16ShortEvent;
    Uint16      u16CmpssEvent;

    Fault_TakeSnap(&snap);
    Fault_OpenTestGate5ms(&snap);
    Fault_ClearCmd5ms();
    Fault_CmpssSrc5ms(&snap);
    Fault_SlowProtect5ms(&snap);

    /* 上升沿计次 → shortLock / cmpssLock（过流两位，不含 bit7） */
    u16ShortEvent = (Uint16)((stFault.slow & FAULT_MASK(FAULT_BIT_SHORT)) != 0UL);
    u16CmpssEvent = (Uint16)((stFault.slow & CMPSS_OCP_MASK) != 0UL);
    Fault_RetryLockTick(&stFaultCnt.retry.shortLock,
                        (Uint16)((stFaultCnt.run.u16ShortPrev == 0U) &&
                                 (u16ShortEvent != 0U)),
                        FAULT_N_LOCK_HIT, FAULT_N_SHORT_LOCK_WIN,
                        ALM_MASK_LOCK);
    Fault_RetryLockTick(&stFaultCnt.retry.cmpssLock,
                        (Uint16)((stFaultCnt.run.u16CmpssPrev == 0U) &&
                                 (u16CmpssEvent != 0U)),
                        FAULT_N_CMPSS_LOCK_HIT, FAULT_N_CMPSS_LOCK_WIN,
                        ALM_MASK_HARDALM);
    stFaultCnt.run.u16ShortPrev = u16ShortEvent;
    stFaultCnt.run.u16CmpssPrev = u16CmpssEvent;

    Fault_FastRecover5ms(snap.f32Vbus, snap.f32I0, snap.f32I1);
}
