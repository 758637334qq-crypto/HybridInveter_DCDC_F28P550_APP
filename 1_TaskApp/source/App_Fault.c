/**
 * ============================================================================
 * 文件名称: App_Fault.c
 * 模块功能: 只做故障判断与置位（安克 FaultDetect 角色）。
 *   状态机 / ISR 直接读 stFault；软起超时读写 RunState.softStart* 置 softTo。
 *   本模块是检测层，不改状态流转。
 * ----------------------------------------------------------------------------
 * 阅读顺序（本文件按此顺序分节）：
 *   §1 变量         stFault / stFaultCnt
 *   §2 RAM 函数     Abs / Confirm / ImbHit / FastProtect —— 带 .TI.ramfunc，
 *                   随 ISR 进 RAM 执行，集中一处便于核对 RAM 运行区占用
 *   §3 状态快照     Fault_PwmOn / Fault_OpenTest / Fault_TakeSnap
 *   §4 通用去抖     Fault_Level / Fault_RetryLockTick / Fault_FastRecTick
 *   §5 专项判据     Fault_Short / SlowProtect5ms（①~⑦）
 *   §6 支撑         CMPSS 采集 / 清故障 / 入口前置处理
 *   §7 快保护恢复   Fault_FastRecover5ms（5ms，5s 安全延时）
 *   §8 对外接口     Fault_Init / Fault_Detect5ms（含软起超时）/ Fault_*Relay
 * ============================================================================
 */
#include "Includes.h"          /* 全局头文件：含 stADC、RunState、SysCmd、Bus 等外部声明 */
#include "Interrupt_CtrlDab.h"       /* ISR 发布的充电发波状态（PwmOn 判据使用） */
#include "App_DabModel.h"      /* DAB 数学模型头文件：含 uMdlAddMes 等模型输出 */

/* ==========================================================================
 * §1 变量（类型见 App_Fault.h）
 * ========================================================================== */

volatile FaultData_t stFault;     /* 三整字故障字：slow / fast / alm（位定义见 App_Fault.h §2） */
FaultCnt_t           stFaultCnt;  /* 去抖与锁定计数组（字段说明见 App_Fault.h §4） */

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

/**
 * @brief 故障字幂等置位（32 位；slow 在 5ms 上下文写）
 */
static void Fault_SetBit(volatile Uint32 *pAll, Uint32 u32Mask)
{
    if ((*pAll & u32Mask) == 0UL)
    {
        *pAll |= u32Mask;
    }
}

/**
 * @brief 故障字幂等清除，理由同 Fault_SetBit
 */
static void Fault_ClearBit(volatile Uint32 *pAll, Uint32 u32Mask)
{
    if ((*pAll & u32Mask) != 0UL)
    {
        *pAll &= ~u32Mask;
    }
}

/**
 * @brief 16 位故障字幂等置位（fast / alm）
 */
static void Fault_SetBit16(volatile Uint16 *pAll, Uint16 u16Mask)
{
    if ((*pAll & u16Mask) == 0U)
    {
        *pAll |= u16Mask;
    }
}

/**
 * @brief 16 位故障字幂等清除
 */
static void Fault_ClearBit16(volatile Uint16 *pAll, Uint16 u16Mask)
{
    if ((*pAll & u16Mask) != 0U)
    {
        *pAll &= (Uint16)(~u16Mask);
    }
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
 * @brief 快速硬件保护检测（ISR 入口）
 *
 * @param f32BusCurr0    通道 0 电流采样值 (A)
 * @param f32BusCurr1    通道 1 电流采样值 (A)
 *
 * 母线电压在函数内取继电器 VM，与 5ms 恢复/慢保护/电压环同源，
 * 避免触发与恢复使用不同测点导致"仍过压却已清故障"。
 *
 * ── 设计原则 ──────────────────────────────────────────
 *   ① 只置位、不恢复：快保护一旦触发即锁存，
 *      恢复由 5ms 任务 Fault_FastRecover5ms() 负责（lock 置位后不恢复）。
 *   ② 连续 N 拍确认：防止 ADC 单次毛刺误触发。
 *   ③ 开路测试不屏蔽本函数：软件过压/过流/不平衡必须全程有效，
 *      OpenTest 只在 5ms 慢保护里屏蔽欠压等非过类判据。
 *
 * 注：CMPSS 硬件跳闸位（slow.cmpss*）不在本函数处理——CMPSS 触发时
 *     硬件已直接封波（OST/DCAEVT），不依赖固件，固件只需在 5ms 任务里
 *     记录锁存源并置位上报，不占用 ISR 时间。见 §6 Fault_CmpssSrc5ms。
 * ─────────────────────────────────────────────────────
 */
#pragma CODE_SECTION(Fault_FastProtect, ".TI.ramfunc");
void Fault_FastProtect(float f32BusCurr0, float f32BusCurr1)
{
    float f32Vbus;                      /* 继电器 VM 母线电压 (V) */

    /* ---- ① 母线过压软件快保护（ADC，与 CMPSS 互补）----
     * 电压源与慢保护/快恢复/电压环同一路（VM）；VBUS1/2 仅供功率计算 */
    f32Vbus = stADC.VmBusVolt.f32IsrResult;

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
 * @param p  输出快照
 */
static void Fault_TakeSnap(FaultSnap_t *p)
{
    float f32Vbus = stADC.VmBusVolt.f32IsrResult;    /* 母线唯一控制/保护电压源 */
    float f32I0 = stADC.BusCurr1.f32IsrResult;
    float f32I1 = stADC.BusCurr2.f32IsrResult;

    p->f32Vbus  = f32Vbus;                           /* 继电器 VM 母线电压 */
    p->f32I0    = f32I0;
    p->f32I1    = f32I1;
    p->f32Iabs  = Fault_Abs(f32I0 + f32I1);          /* |Ibus1 + Ibus2| */
    p->f32Islow = Fault_Abs(                         /* 100ms 滤波后的总电流绝对值 */
        stADC.BusCurr1.Filt.f32Filt100ms + stADC.BusCurr2.Filt.f32Filt100ms);

    p->u16PwmOn    = Fault_PwmOn();
    p->u16OpenTest = Fault_OpenTest();
}

/* ==========================================================================
 * §4 通用去抖
 * ========================================================================== */

/**
 * @brief 通用故障位 置位/清除 函数（迟滞计数法）
 *
 * @param pWord    指向目标故障字（如 &stFault.slow）
 * @param pTick    指向该故障项的计数器
 * @param u16Bit   故障位在字中的位号 (0~31)
 * @param u16Hit   当前周期触发条件是否成立 (1/0)
 * @param u16TripN 触发所需连续成立周期数
 * @param u16RecHit 当前周期恢复条件是否成立 (1/0)
 * @param u16RecN  恢复所需连续成立周期数（0 = 不自动恢复）
 *
 * ── 原理 ──────────────────────────────────────────────
 *   触发路径：
 *     每个控制周期若 hit=1，trip 计数 +1；
 *     当 trip >= tripN 时，将 word 的第 bit 位置 1（故障置位）。
 *     若 hit=0，trip 立即清零（要求连续成立）。
 *
 *   恢复路径（仅 recN > 0 时有效）：
 *     故障已置位且 recHit=1 时，rec 计数 +1；
 *     当 rec >= recN 时，清除该位并重置 rec；
 *     若 recHit=0 或故障未置位，rec 立即清零。
 *
 *   作用：防止信号毛刺导致误触发/误恢复，提高抗扰性。
 *
 * ── 注意 ──────────────────────────────────────────────
 *   tripN 必须 >= 1。若传 0，则 "trip >= tripN" 恒成立，hit=1 的每一拍
 *   都会置位（等于无确认）；确需"单拍即触发"请传 1。
 * ─────────────────────────────────────────────────────
 */
static void Fault_Level(volatile Uint32 *pAll, FaultTick_t *pTick, Uint16 u16Bit,
                        Uint16 u16Hit, Uint16 u16TripN,
                        Uint16 u16RecHit, Uint16 u16RecN)
{
    Uint32 u32Mask = FAULT_MASK(u16Bit);   /* 例：bit=3 → mask=0x08 */

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
 * @brief 固定窗口重复触发锁定推进（短路 / 快保护 / CMPSS 共用）
 *
 * 采用"固定窗口（自首次触发起算）"，不是滑动窗口：
 * 窗口从第一次上升沿开始计时，u32WinMax 拍后若累计次数未达 u16HitMax，
 * 则计数清零并重新等待下一次触发；窗口内上升沿一旦达到 u16HitMax
 * 立即置位 u16AlmMask 对应 alm 位（不等窗口结束）。
 */
static void Fault_RetryLockTick(RetryLock_t *pLock, Uint16 u16Rising,
                                Uint16 u16HitMax, Uint32 u32WinMax,
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
        pLock->win  = 0UL;
    }
    else if ((pLock->hits != 0U) && (++pLock->win >= u32WinMax))
    {
        pLock->hits = 0U;                  /* 窗口到期 → 重新计数 */
        pLock->win  = 0UL;
    }
}

/**
 * @brief 快保护单项恢复判断（直接改故障字，无返回值）
 *
 * @param pWord    快保护故障字指针（stFault.fast）
 * @param u16Mask  本项对应的故障位掩码
 * @param u16Ok    当前周期安全条件是否成立 (1=安全, 0=仍危险)
 * @param pu16Cnt  指向该故障项的恢复计数器
 */
static void Fault_FastRecTick(volatile Uint16 *pWord, Uint16 u16Mask,
                              Uint16 u16Ok, Uint16 *pu16Cnt)
{
    /* ① 故障位 = 0：无需恢复，清计数 */
    if ((*pWord & u16Mask) == 0U)
    {
        *pu16Cnt = 0U;
        return;
    }

    /* ② 故障位 = 1 但条件不满足：重新计时，保持锁存 */
    if (u16Ok == 0U)
    {
        *pu16Cnt = 0U;
        return;
    }

    /* ③ 条件连续满足 FAULT_FAST_REC_N 拍：清故障位 */
    if (*pu16Cnt < FAULT_FAST_REC_N)
    {
        (*pu16Cnt)++;
    }
    if (*pu16Cnt >= FAULT_FAST_REC_N)
    {
        *pu16Cnt = 0U;
        *pWord &= (Uint16)(~u16Mask);
    }
}

/* ==========================================================================
 * §5 专项判据
 * ========================================================================== */

/**
 * @brief 母线短路故障检测（含固定窗口锁定）
 *
 * @param f32IAbs   母线总电流绝对值 |Ibus| (A)
 * @param f32Vbus   母线电压 (V)，取继电器 VM 实测值
 *
 * ── 短路判据（只看电量，不绑发波门）────────────────────
 *   同时满足：
 *     ① |Ibus| >= lim          （电流超过动态阈值）
 *     ② Vbus  < FAULT_VBUS_SHORT （母线电压跌落）
 *   不看 Fault_PwmOn：充电压差停波会清 g_stDabIsrState.u16ChgPwmActive，
 *   若再要求 PwmOn，母线跌落+大电流反而不计短路；欠压仍用 PwmOn。
 *
 *   动态阈值 lim = |I_ref| - 2.0，钳位到 [FAULT_I_SHORT_LO, FAULT_I_SHORT_HI]
 *
 * ── 触发/恢复 ────────────────────────────────────────
 *   触发：连续 FAULT_N_SHORT 次命中 → 置位 SHORT
 *   恢复：连续 FAULT_N_SHORT_REC 次未命中 → 清除 SHORT
 * ─────────────────────────────────────────────────────
 */
static void Fault_Short(float f32IAbs, float f32Vbus)
{
    float  f32ISet = Fault_Abs(Bus.f32Ref);      /* |I_ref| */
    float  f32Lim  = f32ISet - 2.0f;             /* 动态阈值 = |I_ref| - 2 A */
    Uint16 u16Hit;

    /* 上限钳位 */
    if (f32Lim > FAULT_I_SHORT_HI)
    {
        f32Lim = FAULT_I_SHORT_HI;
    }
    /* 下限钳位：限流值不得低于最灵敏的 LO，保证短路保护始终有效 */
    if (f32Lim < FAULT_I_SHORT_LO)
    {
        f32Lim = FAULT_I_SHORT_LO;
    }

    /* 短路命中：电流超限 && 电压跌落（与是否发波无关） */
    u16Hit = (Uint16)((f32IAbs >= f32Lim) && (f32Vbus < FAULT_VBUS_SHORT));

    /* 置位/恢复沿用通用迟滞计数 */
    Fault_Level(&stFault.slow, &stFaultCnt.run.busShort, FAULT_BIT_SHORT,
                u16Hit, FAULT_N_SHORT,
                (Uint16)(u16Hit == 0U), FAULT_N_SHORT_REC);

    /* 短路重试锁定不在本函数计数——统一由 Detect5ms 第 6 步按上升沿
     * 在 30s 窗口内统计（30s 窗口覆盖本处原 5s 窗口的全部场景），
     * 避免两处窗口口径不一。 */
}

/**
 * @brief 慢保护 ①~⑦（5ms 拍，全部基于入口快照，互不重复采样）
 * @param p  本拍快照
 *
 * 注：顺序敏感——④ 依赖 ①~③ 的置位结果。
 */
static void Fault_SlowProtect5ms(const FaultSnap_t *p)
{
    const float f32Vbat = stADC.BatVolt.f32IsrResult;
    Uint16 u16OtherFault;               /* ④ 判据：除欠压外的其它故障 */

    /* 原始电池电压可直接判断欠压，滤波值有下限。 */
    Fault_Level(&stFault.slow, &stFaultCnt.run.batUv, FAULT_BIT_BAT_UV,
                (Uint16)(f32Vbat < FAULT_BAT_UV_V), FAULT_N_BAT_UV,
                (Uint16)(f32Vbat > FAULT_BAT_UV_REC_V), FAULT_N_BAT_UV_REC);

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
     * 恢复：Vbus ≥ 360V 连续 15s 主动清位（不再无条件定时清）。*/
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
                1U,                                            /* 恢复无条件，仅由时间决定 */
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

    /* ---- ⑦ SCI：开路测试不判（空闲会虚高；门控每拍清位兜底）---- */
    {
        Uint16 u16SciHit = (Uint16)((p->u16OpenTest == 0U) &&
                                    ((g_UartProto.u16RxIdleCnt >= FAULT_SCI_TIMEOUT_N) ||
                                     (g_UartProto.u16RunCmdIdleCnt >= FAULT_SCI_TIMEOUT_N)));
        Fault_Level(&stFault.slow, &stFaultCnt.run.sci, FAULT_BIT_SCI,
                    u16SciHit, FAULT_N_SCI,
                    (Uint16)((g_UartProto.u16RxIdleCnt < FAULT_SCI_TIMEOUT_N) &&
                             (g_UartProto.u16RunCmdIdleCnt < FAULT_SCI_TIMEOUT_N)),
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
    if ((stFault.slow & u32Bit) == 0UL)   /* ① 位未置位：无需恢复，清计数 */
    {
        *pu16Rec = 0U;
        return;
    }
    if (u16Ok == 0U)                      /* ② 安全条件不成立：重新计时，保持锁存 */
    {
        *pu16Rec = 0U;
        return;
    }
    if (*pu16Rec < FAULT_N_CMPSS_REC)
    {
        (*pu16Rec)++;
    }
    if (*pu16Rec >= FAULT_N_CMPSS_REC)
    {
        Fault_ClearBit(&stFault.slow, u32Bit);
        /* 只清本路比较器锁存，保留 ePWM OST；他路锁存不动 */
        DabCmpss_ClearFilterLatchSrc(u16LatchSrc);
        *pu16Rec = 0U;
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

    if ((stFault.alm & ALM_MASK_HARDALM) != 0U)
    {
        stFaultCnt.run.cmpssVRec  = 0U;
        stFaultCnt.run.cmpssI1Rec = 0U;
        stFaultCnt.run.cmpssI2Rec = 0U;
        return;
    }

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
 * 欠压、继电器、SCI 属于"非过"类，测试期间必然异常，直接清位并清计数；
 * 关断电流（alm 字）测试期间同样放行，单独清 alm 对应位；
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
                   FAULT_MASK(FAULT_BIT_RELAY) |
                   FAULT_MASK(FAULT_BIT_SCI) |
                   FAULT_MASK(FAULT_BIT_SOFT_TO));
    Fault_ClearBit16(&stFault.alm, ALM_MASK_OFF_CURR);

    stFaultCnt.run.vbusUn.trip  = 0U;
    stFaultCnt.run.vbusUn.rec   = 0U;
    stFaultCnt.run.sci.trip     = 0U;
    stFaultCnt.run.sci.rec      = 0U;
    stFaultCnt.run.offCurrHit   = 0U;
}

/**
 * @brief 清故障命令：仅停机确认后执行（关机请求 + 已在 INIT）。
 *
 * 运行中或开机请求仍为 1 时丢弃命令，避免清 soft/lock/hardalm 后被 ISR/状态机
 * 按旧 RUN 继续控，或清完立刻自动起机。OST 不在此解除。
 */
static void Fault_ClearCmd5ms(void)
{
    if (CfgParam.SysCmd.bit.clearfaultcmd == 0U)
    {
        return;
    }

    /* 消费命令位：通信写 CfgParam.SysCmd 与本函数同在主循环超循环内顺序执行，
     * 不存在抢占，故无需关中断 */
        CfgParam.SysCmd.bit.clearfaultcmd = 0U;

    /* CAN 0x20 只改 CfgParam，这里先同步再判停机门槛 */
    SM_SyncSysCmd();

    /* 停机门槛：无运行请求，且状态机已回 INIT */
    if ((FAULT_RUN_REQ() != 0U) || (RunState.state != SM_STATE_INIT))
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
 * @brief 5ms 周期调用的快保护恢复任务（各位独立判断，5s 安全延时）
 *
 * @param f32Vbus  本拍继电器 VM 母线电压
 * @param f32I0    通道 0 电流
 * @param f32I1    通道 1 电流
 */
static void Fault_FastRecover5ms(float f32Vbus, float f32I0, float f32I1)
{
    Uint16 u16Ok[FAST_BIT_COUNT];
    Uint16 u16Idx;

    /* lock / hardalm 任一置位：快保护进入锁定态，不再自动恢复，
     * 各恢复计数保持清零，解除锁定后须重新连续满足 5s 才允许恢复 */
    if ((stFault.alm & (ALM_MASK_LOCK | ALM_MASK_HARDALM)) != 0U)
    {
        for (u16Idx = 0U; u16Idx < FAST_BIT_COUNT; u16Idx++)
        {
            stFaultCnt.run.fastRec[u16Idx] = 0U;
        }
        return;
    }

    /* ---- 各故障位对应的安全条件（迟滞阈值见 App_Fault.h §1）---- */
    u16Ok[FAST_BIT_VOUT_OV]  = (Uint16)(f32Vbus <= FAULT_FAST_OV_REC);
    u16Ok[FAST_BIT_I1_OV]    = (Uint16)(Fault_Abs(f32I0) <= FAULT_CH_OC_REC);
    u16Ok[FAST_BIT_I2_OV]    = (Uint16)(Fault_Abs(f32I1) <= FAULT_CH_OC_REC);
    u16Ok[FAST_BIT_IBALANCE] = (Uint16)(Fault_ImbHit(f32I0, f32I1) == 0U);

    for (u16Idx = 0U; u16Idx < FAST_BIT_COUNT; u16Idx++)
    {
        Fault_FastRecTick(&stFault.fast, FAULT_MASK16(u16Idx),
                          u16Ok[u16Idx], &stFaultCnt.run.fastRec[u16Idx]);
    }
}

/* ==========================================================================
 * §8 对外接口
 * ========================================================================== */

/**
 * @brief 故障模块初始化（仅上电调用一次）
 *
 * 全量清零三整字故障与全部计数（含 retry 锁定组），并清 CMPSS 滤波锁存、
 * 清 ePWM Trip 状态解除硬件封波。
 * 与运行期清故障（Fault_ClearAll）的区别：只有上电才允许清 retry 组与
 * ePWM OST —— 运行期"反复跳闸 → 清故障 → 再跳闸"无法绕过锁定保护，
 * 而上电时系统尚未起机，解除 OST 安全。
 */
void Fault_Init(void)
{
    memset(&stFaultCnt, 0, sizeof(stFaultCnt));
    Fault_ClearAll();
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

/**
 * @brief 置位软起超时（slow.softTo）；仅本文件调用
 */
static void Fault_SetSoftTo(void)
{
    Fault_SetBit(&stFault.slow, FAULT_MASK(FAULT_BIT_SOFT_TO));
}

/**
 * @brief 清除软起超时位
 */
void Fault_ClearSoftTo(void)
{
    Fault_ClearBit(&stFault.slow, FAULT_MASK(FAULT_BIT_SOFT_TO));
}

/**
 * @brief 软起超时（5ms）：armed 且本态 softStartTick 连续满 50s → softTo
 * @note  tick 随 EnterState 清零；armed 由进 RUN 当场清（在线换向不再监视）
 */
static void Fault_SoftStartTick5ms(void)
{
    if (RunState.softStartArmed == 0U)
    {
        return;
    }
    if (RunState.state == SM_STATE_INIT)
    {
        return;
    }
    if (RunState.softStartTick < FAULT_N_SOFT_TO)
    {
        RunState.softStartTick++;
    }
    if (RunState.softStartTick >= FAULT_N_SOFT_TO)
    {
        Fault_SetSoftTo();
    }
}

/**
 * @brief 5ms 周期故障检测主函数
 *
 * 调用时机：由 5ms 定时任务调度器周期调用。
 *
 * ── 执行流程（顺序敏感，勿随意调换）──────────────────
 *   1. 本拍快照（ADC / PWM / 开路测试标志）
 *   2. 开路测试门控：屏蔽 run / 保留 over
 *   3. clearfaultcmd：当拍消费
 *   4. CMPSS 硬件锁存源 → slow.cmpss*
 *   5. 慢保护 ①~⑦
 *   6. 软起超时（冷启武装且本态连续 50s）
 *   7. 重复触发锁定（短路 / 快保护 → lock；CMPSS → hardalm）
 *   8. 快保护恢复（5s 安全延时）
 * ─────────────────────────────────────────────────────
 */
void Fault_Detect5ms(void)
{
    FaultSnap_t snap;
    Uint16      u16ShortEvent;           /* 短路保护事件（30s / 3 次 → lock） */
    Uint32      u32CmpssEvent;           /* CMPSS 慢保护事件字（5min / 5 次 → hardalm） */

    Fault_TakeSnap(&snap);                                  /* 1. 本拍快照 */
    Fault_OpenTestGate5ms(&snap);                            /* 2. 开路测试门控 */
    Fault_ClearCmd5ms();                                     /* 3. 清故障命令 */
    Fault_CmpssSrc5ms(&snap);                                /* 4. CMPSS → slow */

    Fault_SlowProtect5ms(&snap);                             /* 5. 慢保护 ①~⑦ */
    Fault_SoftStartTick5ms();                                /* 6. 软起超时 */

    /* ---- 6. 重复触发锁定（固定窗口计数，机制见 Fault_RetryLockTick）----
     * 事件按"上升沿"计次：上一拍为 0 且本拍非 0 才计 1 次，
     * 持续存在的故障不重复计次；上一拍存于 run.u16ShortPrev / u32CmpssPrev。
     *   shortLock ：事件 = 短路保护发生（slow 的 busShort 位上升沿）
     *               30s 窗口内满 FAULT_N_LOCK_HIT(3) 次 → alm.lock
     *               （窗口须明显大于"5s 恢复 + 重启"一轮耗时；原 5s 短窗口
     *                的突发短路场景是本窗口的子集，统一合并到此）
     *   cmpssLock ：事件 = slow 中 CMPSS 位
     *               5min 窗口内满 FAULT_N_CMPSS_LOCK_HIT(5) 次 → alm.hardalm
     * lock/hardalm：快保护不再自动恢复（见 §7），只能清故障或重新上电；
     * retry 组不受清故障影响，防止"反复跳→清→再跳"绕过保护。 */
    u16ShortEvent = (Uint16)((stFault.slow & FAULT_MASK(FAULT_BIT_SHORT)) != 0UL);
    u32CmpssEvent = stFault.slow & CMPSS_MASK;
    Fault_RetryLockTick(&stFaultCnt.retry.shortLock,
                        (Uint16)((stFaultCnt.run.u16ShortPrev == 0U) &&
                                 (u16ShortEvent != 0U)),
                        FAULT_N_LOCK_HIT, FAULT_N_SHORT_LOCK_WIN,
                        ALM_MASK_LOCK);
    Fault_RetryLockTick(&stFaultCnt.retry.cmpssLock,
                        (Uint16)((stFaultCnt.run.u32CmpssPrev == 0UL) &&
                                 (u32CmpssEvent != 0UL)),
                        FAULT_N_CMPSS_LOCK_HIT, FAULT_N_CMPSS_LOCK_WIN,
                        ALM_MASK_HARDALM);
    stFaultCnt.run.u16ShortPrev  = u16ShortEvent;
    stFaultCnt.run.u32CmpssPrev  = u32CmpssEvent;

    Fault_FastRecover5ms(snap.f32Vbus, snap.f32I0, snap.f32I1);  /* 8. 快保护恢复 */
}
