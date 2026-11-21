/**
 * ============================================================================
 * 文件名称: App_StateMachine.c
 * 模块功能: DAB (双有源全桥变换器) 主状态机源码实现文件
 * 控制核心: 充放电全工况统一闭环控制高压直流母线电压 (Vbus) 与母线电流 (Ibus)
 * 调度周期: 2ms 主步进；STANDBY / 换向裁决在 100us
 * 版权说明: 最终量产定版 (Production Ready)
 * ============================================================================
 *
 * 【任务上下文分布】(谁在什么节拍里跑什么)
 *   Task_2ms   : SM_Step_2ms()      —— 状态推进；关机/故障裁决拥有最高优先级
 *   Task_100us : SM_Standby_100us() —— 换向仲裁(只计票) + STANDBY 驻留/出态提交
 *   EPWM10_ISR   : 只读 RunState.state / workMode / curLowSealing，不回写状态机变量
 *
 * 【状态流转总图】
 *   冷启:  INIT → RELAY → GPIO → PRECHECK → RAMP_CLOSE → RUN
 *   换向:  强制/CAN: RAMP_CLOSE/RUN 均可提交(驻留5拍)
 *          自动: 仅 RUN 计票(双票确认+驻留1拍), RAMP_CLOSE 抑制自动换向 --> STANDBY
 *          STANDBY(驻留到期, 以最新方向重装载) → RAMP_CLOSE → RUN
 *   停机:  任意态 --(故障 | 上位机清故障 | 协议关机)--> INIT (OstStop 硬件封波)
 *   注: 起机跳过 STANDBY (STANDBY 仅作换向中转), STANDBY 占用原 RAMP_OPEN=4 槽位。
 *
 * 【封波三层防线(软 → 硬, 与 Interrupt_CtrlDab.c 封波门对齐)】
 *   ① ISR 封波门: state ∉ {RAMP_CLOSE, RUN} → 每拍 DAB_PWM_FORCE_LOW() 且跳过
 *      Dab_Ctrl_Loop()。STANDBY/PRECHECK/INIT 等所有非闭环态天然被此门拦住。
 *   ② 方向封波位: curLowSealing(禁充/禁放)在 Dab_Ctrl_Loop 之后、Dab_Pwm_*Update 之前强制低
 *      —— 允许外环反向计算给定, 仅在"当前工况=被禁方向"时封锁输出。
 *   ③ OST 硬件锁存: DAB_PWM_OstStop() 触发 TZ 单向 Trip; 仅提交点在
 *      Fault_PreStartHwOk() 通过后 DAB_PWM_Start() 清除。故障停机用此层。
 *
 * 【跨上下文共享契约(重难点)】
 *   - RunState.state / workMode: 16 位单字读写天然原子; 任务写 ISR 读; volatile。
 *   - stateTick: "当前态自管"计时字段, 按态分节拍复用:
 *       RELAY/GPIO/PRECHECK → 2ms 句柄内自增; RAMP_CLOSE → 2ms 调度器尾部自增;
 *       STANDBY → 100µs 任务自增(驻留计拍)。任意时刻仅一个上下文持有它,
 *       进态统一清零(SM_EnterState), 因此无需跨上下文互斥。
 *   - irefDebounce / voltDebounce / standbyDwellN: 仅 100µs 任务写(单一写者, 免锁)。
 *   - curLowSealing: 1ms Ctrl_RampCfg 重算写, 100µs 仲裁与 ISR 读, 位图单字原子。
 *   - Bus.f32Ref: ISR 写、100µs 仲裁读, 允许读到 1 拍旧值(仅判向用, 不参与闭环)。
 * ============================================================================
 */

#include "App_StateMachine.h"
#include <string.h>
#include <math.h>

/* ==========================================================================
 * 全局对象
 *
 * SysCmd   : 运行侧命令镜像(2ms/串口/CAN/清故障写入, ISR 只读)
 * RunState : 状态机运行上下文(任务读写, ISR 只读 state/workMode/curLowSealing)
 * 两者均跨上下文共享, 必须 volatile; 所有单字段均为 16 位, 读写天然原子,
 * 单变量无需关中断保护(复合装载序列除外, 见 SM_StartClosedLoop)。
 * ========================================================================== */
volatile SysCmd_t    SysCmd;    /* 命令控制字: 任务写 / ISR 读 */
volatile RunState_t  RunState;  /* 运行上下文: 状态机本体 */

/**
 * @brief  进态原语: 写入新状态并清零通用阶段计时
 *
 * 【原理】stateTick 是按态复用的计时字段(RELAY 超时 / GPIO 缓冲 / PRECHECK 滑窗 /
 *         RAMP_CLOSE 驻留 / STANDBY 驻留共用), 若进新态不清零, 下一态会拿到
 *         上一态遗留的计时基数, 导致超时/窗口判断整体漂移。故统一在进态点清零,
 *         各态句柄无需重复处理。
 *
 * 【原子性】state 为 16 位单字写, ISR 侧以 u16StateTmp 快照读取, 无撕裂风险;
 *          static inline 让编译器就地展开到 8 个调用点, 无函数调用开销,
 *          且不会像非 static 全局函数那样在头文件被包含时产生多重定义。
 */
static inline void SM_EnterState(SmState_e eNext)
{
    RunState.state        = eNext;
    RunState.stateTick    = 0U;
    RunState.softStartTick = 0U;   /* 本态连续 50s 口径：进态清零 */
}

/**
 * @brief  清空换向防抖票箱(电压票 + 电流给定票)
 *
 * 【重难点】两个防抖计数器不含方向位, 只表达"反向条件连续成立的时长"。
 * 计票资格失效或进入 STANDBY 时清票；条件持续成立但未满计时保留。
 */
static void SM_ClearModeDebounce(void)
{
    RunState.irefDebounce = 0U;
    RunState.voltDebounce = 0U;
}

/** 外部指令判向：强制优先于 CAN；仅本文件使用 */
static Uint16 SM_ResolveForcedMode(Uint16 u16ForceMode)
{
    Uint16 u16CanCmd = (Uint16)RunState.canCmd;

    if (u16ForceMode == 1U)
    {
        return (Uint16)WORK_MODE_DISCHARGE;
    }
    if (u16ForceMode == 2U)
    {
        return (Uint16)WORK_MODE_CHARGE;
    }
    if (u16CanCmd == (Uint16)CAN_CMD_DISCHARGE)
    {
        return (Uint16)WORK_MODE_DISCHARGE;
    }
    if (u16CanCmd == (Uint16)CAN_CMD_CHARGE)
    {
        return (Uint16)WORK_MODE_CHARGE;
    }
    return (Uint16)WORK_MODE_STANDBY;
}

/* ==========================================================================
 * CfgParam.SysCmd → SysCmd 命令镜像
 *
 * 【为什么要镜像】CfgParam 是协议层参数区, 串口 0x00 / CAN 0x20 / 清故障等多个
 * 上下文都可能改写; SysCmd 是运行侧的稳定快照。镜像后状态机只依赖单一数据源,
 * 避免直接读协议区时遇到"位域写了一半"的组合。调用时机: 2ms UpdateShutdownFlag
 * 开头 + 各通讯命令入口, 保证任一入口改完 CfgParam 后最迟 2ms 对状态机生效。
 * ========================================================================== */
void SM_SyncSysCmd(void)
{
    SysCmd.bit.forceMode = CfgParam.SysCmd.bit.ReCharForce;  /* 强制方向(0自适应/1强放/2强充) */
    SysCmd.bit.openTest  = CfgParam.SysCmd.bit.openTest;     /* 开环测试模式镜像 */
    SysCmd.bit.shutdown  = (CfgParam.SysCmd.bit.onoffcmd || CfgParam.SysCmd.bit.openTest) ? 1U : 0U;
    /* shutdown = 开机指令 且 非开环测试。开环测试强制视为"要求停机重走起机流程",
     * 以便在 INIT→PRECHECK 链路中以测试模式起机(继电器压差检查被旁路)。 */
    /* standby 不镜像到 SysCmd：协议 standby 语义已由 SM_STATE_STANDBY 状态承担,
     * 运行侧只认 RunState.state 单一事实源, 避免位域与状态双源打架。 */
}

/**
 * @brief  进入 STANDBY(换向中转): 封波 + 记录目标方向/驻留 + 清控制辅助量
 *
 * 【函数说明】换向的唯一提交点(100µs 任务调用)。u16Target: 换向后的目标工况;
 * u16DwellN: 本趟 STANDBY 驻留拍数(0 视为非法, 兜底为强制驻留 5 拍=500µs)。
 *
 * 【流程】
 *   ① SM_EnterState(STANDBY) —— state 退出 ISR 封波门白名单{CLOSE,RUN},
 *      下一拍起 ISR 每拍 FORCE_LOW 且跳过控制步;
 *   ② DAB_PWM_Stop() —— AQCSFRC 软件强制低, 不等下一拍立即生效, 与①双保险;
 *   ③ workMode ← 目标方向 —— 出态时 SM_StartClosedLoop 以此装载;
 *   ④ standbyDwellN ← 驻留拍数;
 *   ⑤ 清票箱 + 清 CAN 指令(已消费) + Ctrl_StandbyClear()(仅清均流辅助量;
 *      Bus/PI/Duty 留给出态关中断窗内 Ctrl_SetMode+RampInit 统一清零 —— 此刻
 *      ISR 已不跑 Dab_Ctrl_Loop, 残留数据不会被消费, 无需在此抢清)。
 *
 * 【重难点·残留时序窗口】①与②之间存在在飞 ISR: 它锁存的是切换前的旧 state
 * (仍属白名单), 会完整跑完本拍 Dab_Ctrl_Loop+Dab_Pwm_*Update, 并因 Dab_Pwm_*Update 清 AQCSFRC 而顶掉
 * ②刚写的强制低 —— 最坏漏出一拍(≈16.7µs)切换前占空比。这是既有设计权衡:
 * 换向命令本身即封波前奏, 一拍旧波幅值有限; 若要根除需将①②包进关中断窗。
 */
static void SM_EnterStandby(Uint16 u16Target, Uint16 u16DwellN)
{
    SM_EnterState(SM_STATE_STANDBY);   /* ① 切状态: ISR 封波门下一拍生效 */
    DAB_PWM_Stop();                    /* ② 软强制低: 立即生效, 双保险 */
    RunState.workMode = u16Target;     /* ③ 记录目标方向, 出态时装载 */
    RunState.standbyDwellN = (u16DwellN == 0U) ? SM_T_STANDBY_FORCE : u16DwellN;
    SM_ClearModeDebounce();            /* ⑤a 清换向票箱(进待机=一次完整换向事务) */
    RunState.canCmd = CAN_CMD_NONE;    /* ⑤b CAN 指令已消费, 防重复解析 */
    Ctrl_StandbyClear();               /* ⑤c 仅清均流; Bus/PI/Duty 出态再清 */
}

/**
 * @brief  闭环启动/换向装载: 关中断窗内原子完成"清环→播种→解封→公布状态"
 *
 * 【函数说明】冷启(PRECHECK 判向完成)与换向(STANDBY 驻留到期)共用的唯一闭环
 * 提交点。u16Mode: 目标工况; u16IsSwitch: 0=冷启播种 / 1=换向播种(额外清方向
 * 封波位, 换向后新方向才允许发波)。
 *
 * 【流程】
 *   ① f32Vbus: 双路母线陷波均值, 斜坡/外环播种初值;
 *   ② 关中断窗(顺序不可调换):
 *      a. SetMode / Ctrl_SetMode / Dab_Ctrl_SessionReset / RampInit*: 装载波、环参与会话状态;
 *      b. Fault_PreStartHwOk(): **清 OST 直前**复核 CMPSS。失败 → OstStop + INIT,
 *         本窗不解 OST —— 关掉「复核通过→装载中短过流→TZCLR 把新跳闸一并清掉」窗口;
 *      c. DAB_PWM_Start(): 仅复核通过才 TZCLR.OST;
 *      d. Enter(RAMP_CLOSE): 最后公布状态;
 *   ③ restore 中断。
 *
 * 【重难点·两类 OST 同源解锁】
 *   · 正常换向: EnterStandby 用 AQCSFRC 软停, 通常未强制 OST; Start 清 OST 是兜底,
 *     若装载窗内 CMPSS 已跳闸则 b 挡住, 不会误开。
 *   · 故障后恢复: OstStop 硬锁 OST, soft 位可先恢复; 仍须 b 复核通过才解锁。
 *   两类共用「复核紧贴 TZCLR」, 不拆两条清 OST API。
 */
static void SM_StartClosedLoop(Uint16 u16Mode, Uint16 u16IsSwitch)
{
    float    f32Vbus;
    uint16_t intm;
    Uint16   u16HwOk;

    f32Vbus = stADC.VmBusVolt.f32Notch;   /* 继电器 VM 侧陷波值（控制主反馈） */

    intm = __disable_interrupts();
    DAB_PWM_SetMode(u16Mode);
    Ctrl_SetMode(u16Mode);
    Dab_Ctrl_SessionReset();   /* 清上一会话的充电迟滞锁存，避免“轻载休眠=1”跨起机残留 */
    if (u16IsSwitch != 0U)
    {
        Ctrl_RampInitSwitch(f32Vbus);
    }
    else
    {
        Ctrl_RampInitCold(f32Vbus);
    }

    /* 装载后核 → Start → 清后再核（挡 TZCLR 窄缝）；失败共用 OstStop→INIT */
    u16HwOk = Fault_PreStartHwOk();
    if (u16HwOk != 0U)
    {
        DAB_PWM_Start();
        u16HwOk = Fault_PreStartHwOk();
    }
    if (u16HwOk == 0U)
    {
        DAB_PWM_OstStop();
        SM_EnterState(SM_STATE_INIT);
        __restore_interrupts(intm);
        return;
    }
    SM_EnterState(SM_STATE_RAMP_CLOSE);
    __restore_interrupts(intm);
}

/**
 * @brief  100µs 自动换向计票(只计票, 不改 workMode; 提交统一在 SM_Standby_100us)
 *
 * 【函数说明】自动换向的"感知层"。两条独立确认通道, 任一满计即认为"要反向":
 *   ① 电压通道(仅 RUN 计票, 10 拍≈1ms): 母线电压越出设定带 ±20V;
 *      RAMP_CLOSE 期间软起尚未稳定, 自动换向被抑制, 不进本函数计票;
 *   ② 电流给定通道(仅 RUN 计票, 4 拍≈400µs): 外环给定已翻到反向符号。
 * 本函数绝不直接修改 workMode —— 计票与提交分离, 提交点唯一(SM_Standby_100us),
 * 避免 100µs 内多处改向造成竞态。
 *
 * 【原理·方向怎么翻】CLOSE/RUN 时 workMode 只能是充或放, 合法候选只有"反方向"。
 * 计数器不含方向位, 只表达"反向条件连续成立的时长"; 目标方向由提交点按当前
 * workMode 取反得到(充↔放)。
 *
 * 调用方已确定反向目标并检查禁向，本函数只累计电压和电流给定条件。
 *
 * 【防抖语义】连续成立才累加, 任一拍不成立立即清零(非滑窗平均) —— 抗毛刺,
 * 且"条件消失"的响应与"条件出现"一样快。
 *
 * 【数据来源】Bus.f32Ref 为 ISR 产物, 任务侧读到 1 拍旧值属正常, 判向用途可容忍。
 */
static void SM_ModeArbitrate_100us(void)
{
    float f32Vbus;
    float f32Iact;
    float f32Iref;
    Uint16 u16WantRev;

    /* ---------- ① 电压通道: 只看"相对当前方向的反向条件" ---------- */
    f32Vbus = stADC.VmBusVolt.f32Notch;    /* 电压环同一个值: 继电器 VM 侧陷波 */
    f32Iact = stADC.BusCurr1.f32IsrResult + stADC.BusCurr2.f32IsrResult;  /* 两路母线电流和 */
    u16WantRev = 0U;

    if (RunState.workMode == (Uint16)WORK_MODE_CHARGE)
    {
        if ((f32Vbus < (RampTarget.v - SM_V_MODE_BAND)) &&
            (f32Iact > -SM_EPS_CURR) &&
            ((RunState.curLowSealing & SM_SEAL_NO_DISCHARGE) == 0U))
        {
            u16WantRev = 1U;
        }
    }
    else if (RunState.workMode == (Uint16)WORK_MODE_DISCHARGE)
    {
        if ((f32Vbus > (RampTarget.v + SM_V_MODE_BAND)) &&
            (f32Iact < SM_EPS_CURR) &&
            ((RunState.curLowSealing & SM_SEAL_NO_CHARGE) == 0U))
        {
            u16WantRev = 1U;
        }
    }

    if (u16WantRev != 0U)
    {
        if (RunState.voltDebounce < SM_N_VOLT_DEBOUNCE)   /* 未满计则+1, 满计封顶 */
        {
            RunState.voltDebounce++;
        }
    }
    else
    {
        RunState.voltDebounce = 0U;                       /* 条件断即清零(抗毛刺) */
    }

    /* ---------- ② 电流给定通道：仅 RUN；同样只计「要反向」 ---------- */
    if (RunState.state != SM_STATE_RUN)
    {
        RunState.irefDebounce = 0U;   /* 非 RUN(CLOSE 软起中)电流票无效并清零 */
        return;
    }

    f32Iref = Bus.f32Ref;             /* 外环输出=整机电流给定(带符号) */
    u16WantRev = 0U;

    if (RunState.workMode == (Uint16)WORK_MODE_CHARGE)
    {
        if ((f32Iref > SM_EPS_CURR) &&
            ((RunState.curLowSealing & SM_SEAL_NO_DISCHARGE) == 0U))
        {
            u16WantRev = 1U;
        }
    }
    else if (RunState.workMode == (Uint16)WORK_MODE_DISCHARGE)
    {
        if ((f32Iref < -SM_EPS_CURR) &&
            ((RunState.curLowSealing & SM_SEAL_NO_CHARGE) == 0U))
        {
            u16WantRev = 1U;
        }
    }

    if (u16WantRev != 0U)
    {
        if (RunState.irefDebounce < SM_N_IREF_DEBOUNCE)   /* 未满计则+1, 满计封顶 */
        {
            RunState.irefDebounce++;
        }
    }
    else
    {
        RunState.irefDebounce = 0U;                       /* 条件断即清零(抗毛刺) */
    }
}

/**
 * @brief  100µs 换向总入口: 仲裁计票 + STANDBY 驻留/出态提交
 *
 * 【函数说明】状态机唯一在 100µs 节拍运行的函数, 承担两件事:
 *   A. state==STANDBY: 驻留计时, 到期按最新方向重装载闭环(出态);
 *   B. state∈{CLOSE,RUN}: 判定是否需要换向, 需要则提交进 STANDBY。
 *
 * 【流程】
 *   A 出态路径: ++stateTick 计驻留(进态已清零) → 未满 standbyDwellN 直接返回 →
 *     故障/关机复查(命中则不出态, 保持封波等 2ms 裁决收敛回 INIT) →
 *     到期重读强制方向(待机期间用户改向立即生效; 无外部指令沿用入态目标) →
 *     SM_StartClosedLoop(mode, isSwitch=1) 换向播种路径回 RAMP_CLOSE。
 *   B 入口判定: 非 CLOSE/RUN → 清票箱返回(其他阶段无换向概念) →
 *     ① 强制/CAN 指令存在: dwell=5(500µs 裕量), 直接以指令方向为目标;
 *     ② 无外部指令: RAMP_CLOSE 直接清票返回(软起期抑制自动换向);
 *        RUN 取反向目标，计票前查一次禁向；被禁则清票返回。
 *        允许计票后，任一计数满 → dwell=1;
 *     ③ 目标==当前向(如强制指令与当前一致): 清票箱+消费 CAN 指令返回;
 *     ④ SM_EnterStandby(目标, dwell) 提交换向。
 *
 * 【重难点·驻留拍数差异】强制/CAN=5 拍(500µs): 方向由人/上位机给出, 无任何
 * 事前确认, 留足裕量让软强制/均流复位彻底完成; 自动=1 拍(100µs): 方向已经
 * 双通道防抖确认(≈400µs/≈1ms), 条件早已成熟, 一拍封波即可重装载。
 *
 * 【重难点·"到期以最新强制方向为准"】STANDBY 期间若用户把强放改成强充,
 * 出态直接按强充起机 —— 用户最后意图优先, 避免换到一半又被再次换向。
 */
void SM_Standby_100us(void)
{
    Uint16 u16Next;
    Uint16 u16DwellN;
    Uint16 u16SealBit;

    /* ================= A. STANDBY 驻留/出态 ================= */
    if (RunState.state == SM_STATE_STANDBY)
    {
        if (++RunState.stateTick < RunState.standbyDwellN)  /* 100µs 计驻留, 未满即返回 */
        {
            return;
        }
        /* 出态前复查故障/关机: 驻留期间新置故障或协议关机时不得解 OST 重新发波,
         * 保持 STANDBY 封波, 由 2ms UpdateShutdownFlag 收敛回 INIT。 */
        if (FAULT_ANY() || CfgParam.SysCmd.bit.faultcmd ||
            (SysCmd.bit.shutdown == 0U))
        {
            return;
        }
        /* 到期以最新强制方向为准；无外部指令则沿用入态目标。 */
        u16Next = SM_ResolveForcedMode((Uint16)SysCmd.bit.forceMode);
        if (u16Next != (Uint16)WORK_MODE_STANDBY)   /* 有外部指令: 覆写入态目标 */
        {
            RunState.workMode = u16Next;
        }
        SM_StartClosedLoop(RunState.workMode, 1U);  /* 换向播种路径回 RAMP_CLOSE */
        return;
    }

    /* ================= B. 换向入口判定 ================= */
    if ((RunState.state != SM_STATE_RAMP_CLOSE) && (RunState.state != SM_STATE_RUN))
    {
        SM_ClearModeDebounce();   /* INIT/RELAY/GPIO/PRECHECK: 无换向概念, 清票防残留 */
        return;
    }

    /* 换向入口：①强制 ②CAN → 驻留 5 拍；③自动满计 → 驻留 1 拍
     * 真正改方向：SM_EnterStandby(u16Next) 写入 workMode=目标，封波；
     * 出待机再 SM_StartClosedLoop(workMode) 按新方向重新闭环。 */
    u16DwellN = SM_T_STANDBY_FORCE;                               /* 默认按强制档驻留 */
    u16Next = SM_ResolveForcedMode((Uint16)SysCmd.bit.forceMode); /* 解析外部指令 */
    if (u16Next == (Uint16)WORK_MODE_STANDBY)                     /* 无强制/CAN → 自动仲裁 */
    {
        /* 软起期抑制自动换向: RAMP_CLOSE 内母线还在跟随斜坡, 判向条件未成熟,
         * 计票会被起机落差反复触发, 出现刚换完向又换回去的来回切换。
         * 强制/CAN 是外部意图, 不受此抑制。 */
        if (RunState.state == SM_STATE_RAMP_CLOSE)
        {
            SM_ClearModeDebounce();
            return;
        }

        /* 先定反向目标；两个计数器共用一次禁向检查。 */
        if (RunState.workMode == (Uint16)WORK_MODE_CHARGE)
        {
            u16Next = (Uint16)WORK_MODE_DISCHARGE;   /* 充→放 */
            u16SealBit = SM_SEAL_NO_DISCHARGE;       /* 目标方向对应的封波位 */
        }
        else if (RunState.workMode == (Uint16)WORK_MODE_DISCHARGE)
        {
            u16Next = (Uint16)WORK_MODE_CHARGE;      /* 放→充 */
            u16SealBit = SM_SEAL_NO_CHARGE;
        }
        else
        {
            SM_ClearModeDebounce();       /* workMode 异常(STANDBY/非法值): 票作废 */
            return;
        }

        if ((RunState.curLowSealing & u16SealBit) != 0U)
        {
            SM_ClearModeDebounce();
            return;
        }

        SM_ModeArbitrate_100us();
        if ((RunState.voltDebounce < SM_N_VOLT_DEBOUNCE) &&
            (RunState.irefDebounce < SM_N_IREF_DEBOUNCE))
        {
            return;
        }
        u16DwellN = SM_T_STANDBY_AUTO;
    }

    if (u16Next == RunState.workMode)
    {
        SM_ClearModeDebounce();           /* 同向返回 */
        RunState.canCmd = CAN_CMD_NONE;
        return;
    }

    SM_EnterStandby(u16Next, u16DwellN);  /* 进待机内清票 */
}

/* ==========================================================================
 * 关机 / 故障 —— 每 2ms 拍最先执行(最高优先级)
 * ========================================================================== */

/**
 * @brief  关机/故障裁决: 任意态收敛回 INIT + 故障快照上报 + 过程性故障自动复位
 *
 * 【流程】
 *   ① SM_SyncSysCmd(): 先刷新命令镜像, 本拍后续判断全部基于最新指令;
 *   ② 停机三触发(任一成立即整备停机):
 *      a. FAULT_ANY()  —— 任意快/慢/alm 故障字非零;
 *      b. faultcmd     —— 上位机请求清故障: 清零动作仅允许在停机态执行
 *                         (Fault_ClearCmd5ms 的前置条件), 故先回 INIT;
 *      c. shutdown==0  —— 协议层要求关机(onoffcmd=0 或 openTest=1);
 *   ③ 故障快照: fast/slow/alm 各自仅在非零时写入 SaveAlarm —— 持续快照"最后
 *      一次非零故障", 不会被后续零值覆盖, 供上位机事后追溯停机原因;
 *   ④ DAB_PWM_OstStop(): OST 硬件锁存封波(仅 state!=INIT 时执行, 防重复写);
 *   ⑤ SM_EnterState(INIT): 状态收敛, 由 Handle_Init 逐拍复位安全面;
 *   ⑥ shutdown==0(主动关机)时: Fault_ClearRelay + Fault_ClearSoftTo ——
 *      继电器超时/软起超时属"过程性故障", 停机即视为复位完成, 下次起机不被
 *      历史阻塞(与硬故障的严格复位条件相区分)。
 *
 * 【重难点】为什么停机用 OstStop 而非 DAB_PWM_Stop: Stop 是 AQCSFRC 软强制,
 * 任何后续代码误清 AQCSFRC 都会重新发波; OST 是 TZ 单向锁存, 只有显式
 * TZCLR(且仅在起机提交点)才解除 —— 硬件层兜底, 软件漏写也不出波。
 */
static void UpdateShutdownFlag(void)
{
    SM_SyncSysCmd();                                        /* ① 刷新命令镜像 */

    if (FAULT_ANY() || CfgParam.SysCmd.bit.faultcmd ||      /* ② 停机三触发 */
        (SysCmd.bit.shutdown == 0U)||g_BootProto.u16FlashUpdateFlag)
    {
        if (stFault.fast != 0U)                             /* ③ 故障快照(仅非零写) */
        {
            g_CanMon.SaveAlarm.savefastfault = stFault.fast;
        }
        if (stFault.slow != 0U)
        {
            g_CanMon.SaveAlarm.saveslowfault1 = stFault.slow;
        }
        if (stFault.alm != 0U)
        {
            g_CanMon.SaveAlarm.savealarm = stFault.alm;
        }

        if (RunState.state != SM_STATE_INIT)                /* ④ 非 INIT 才补 OST 封波 */
        {
            DAB_PWM_OstStop();
        }
        RunState.softStartArmed = 0U;
        SM_EnterState(SM_STATE_INIT);                       /* Enter 已清 softStartTick */
    }

    if (SysCmd.bit.shutdown == 0U)                          /* ⑥ 主动关机: 过程性故障复位 */
    {
        Fault_ClearRelay();                                 /* 继电器超时故障允许复位 */
        Fault_ClearSoftTo();                                /* 软起超时故障允许复位 */
    }
}

/* ==========================================================================
 * 各状态处理(2ms) —— 每态一个句柄, 由 SM_Step_2ms 分发
 * ========================================================================== */

/**
 * @brief  [状态0] INIT: 安全面复位 + 起机放行 + 继电器初始吸合
 *
 * 【流程】每 2ms 重复执行, 直到满足起机放行条件:
 *   ① 从机角色: workMode 强制 STANDBY —— 从机不自主判向, 方向由主机调度
 *      (当前调度链路未接入, 此处仅固定角色语义);
 *   ② 每拍复位安全面: 清 CAN 指令 / DAB_PWM_Stop 软强制兜底 / PowerSafeOff
 *      全部功率 IO 进安全态 —— INIT 停留多久, 安全面就保持多久;
 *   ③ 起机放行三条件(同时成立): 无故障 + 无清故障请求 + shutdown==1(协议要求运行);
 *   ④ 放行后一次性动作: Ctrl_Init 控制模块冷复位 → relayStable/票箱/禁向位
 *      清零 → 吸合软起继电器与主继电器 → 进 RELAY 等待压差收敛。
 *
 * 【重难点·软起与主继电器同拍吸合】两行紧邻同拍执行, 隐含前提是软起支路已完成
 * 预充(VM 侧已有电, 由外部上电时序保证)。若硬件上 SOFT_RLY 串预充电阻, 则主
 * 继电器同拍闭合会短路限流电阻, 需改错拍吸合 —— 属硬件原理图确认项(遗留)。
 */
static void Handle_Init(void)
{
    if (RunState.isMaster == 0U)                 /* ① 从机角色: 方向不由本机判 */
    {
        RunState.workMode = WORK_MODE_STANDBY;
    }

    RunState.canCmd = CAN_CMD_NONE;              /* ② 清外部指令残留 */
    DAB_PWM_Stop();                              /*    软强制兜底(与 OST 双保险) */
    AppGpio_PowerSafeOff();                      /*    全部功率 IO 安全态 */

    if ((FAULT_ANY() == 0U) && (CfgParam.SysCmd.bit.faultcmd == 0U) &&   /* ③ 放行三条件 */
        (SysCmd.bit.shutdown != 0U))
    {
        Ctrl_Init();                             /* ④ 控制模块冷复位(斜坡/补偿/环路态) */

        RunState.relayStable = 0U;               /*    RELAY 平稳计数清零 */
        SM_ClearModeDebounce();                  /*    换向票箱清零 */
        RunState.curLowSealing = 0U;             /*    禁向位清零(双向可用) */
        RunState.softStartArmed = 1U;            /*    开机监视；Enter(RELAY) 已清 tick */
        AppGpio_SoftRlySet(1U);
        AppGpio_RelayBusSet(1U);                 /*    主继电器吸合(同拍, 见重难点) */
        SM_EnterState(SM_STATE_RELAY);           /*    → RELAY 等压差收敛 */
    }
}

/**
 * @brief  [状态1] PRECHECK: 40ms 滑窗母线观测 + 三级优先判向 + 闭环冷启提交
 *
 * 【流程】
 *   ① 窗内(前 19 拍): 每拍统计"母线(VM 陷波) < 设定电压"命中次数(precheckLowCnt),
 *      窗未满直接返回 —— 40ms 滑窗给电压采样/陷波足够的建立时间;
 *   ② 窗满(第 20 拍)判向, 三级优先:
 *      a. 强制/CAN 外部指令(SM_ResolveForcedMode): 人/上位机意图最高优先;
 *      b. 禁向评估: 充/放电电流设定 ≤0.5A 判该方向禁用 —— 单边禁用直接定
 *         另一向(双禁/双可用落入 c 项电压多数票);
 *      c. 电压多数票: 窗内 ≥15/20 拍欠压 → 放电(母线缺电需电池顶上), 否则充电;
 *   ③ 提交: 写 workMode / 清 CAN 指令 / 清票数 → SM_StartClosedLoop(mode, 0)
 *      冷启播种路径 → RAMP_CLOSE。
 *
 * 【重难点·判向基准】预检以"母线(VM 陷波) < 设定值"计放电票(无死区带), 与在线电压换向
 * 的"RampTarget.v ± 20V"带判据用途不同, 不能互套:
 *   · 预检是一次性判向, 没有"当前方向"可回差, 不存在来回横跳, 故不需要滞回带;
 *     抗噪靠 40ms 窗内 15/20 多数票(能容忍毛刺), 等效门限强于单点死区。
 *   · 在线换向每秒判 1 万次, 必须用 ±20V 带做迟滞, 否则会在两方向间乒乓。
 * 判向倾向 = "持续欠压才放电, 否则充电": 充电从母线取电、放电向母线馈电, 母线
 * 持续低于设定说明缺电需顶起, 才判放电; 票数不足(母线接近设定)说明母线已有
 * 能量, 判充电只是吸收能量, 不会继续顶高母线, 判错代价更小。
 *
 * 【重难点·计数语义】precheckLowCnt 是"命中次数"而非连续计数 —— 与换向防抖
 * 的"断即清零"语义不同, 允许窗内毛刺, 由多数票门限兜底。
 */
static void Handle_Precheck(void)
{
    Uint16 workMode = WORK_MODE_DISCHARGE;               /* 判向结果(初值仅为消警告, ②必覆盖) */
    float vBus = stADC.VmBusVolt.f32Notch;               /* 继电器 VM 陷波值(与电压环同源) */
    float f32Vset = (float)CfgParam.BusVolSet * 0.1f;    /* 设定电压(0.1V 定标→V) */

    /* 窗内统计: 母线 < 设定值即计放电票(无死区带, 见重难点·判向基准) */
    if (vBus < (f32Vset))
    {
        RunState.precheckLowCnt++;
    }

    if (++RunState.stateTick < SM_N_PRECHECK_WIN)        /* ① 40ms 滑窗未满 */
    {
        return;
    }

    workMode = SM_ResolveForcedMode((Uint16)SysCmd.bit.forceMode);  /* ②a 外部指令优先 */
    if (workMode == (Uint16)WORK_MODE_STANDBY)                      /* 无外部指令 */
    {
        Uint16 u16NoDis = (((float)CfgParam.BusDischgCurSet * 0.1f) <= SM_IBUS_DISCHAR_STANDBY_MIN) ? 1U : 0U;
        Uint16 u16NoChg = (((float)CfgParam.BusChgCurSet * 0.1f) <= SM_IBUS_CHAR_STANDBY_MIN) ? 1U : 0U;

        if ((u16NoDis != 0U) && (u16NoChg == 0U))        /* ②b 仅放电被禁 → 充电 */
        {
            workMode = (Uint16)WORK_MODE_CHARGE;
        }
        else if ((u16NoChg != 0U) && (u16NoDis == 0U))   /* 仅充电被禁 → 放电 */
        {
            workMode = (Uint16)WORK_MODE_DISCHARGE;
        }
        else                                             /* ②c 电压多数票判向 */
        {
            workMode = (RunState.precheckLowCnt >= SM_N_PRECHECK_TH)
                     ? (Uint16)WORK_MODE_DISCHARGE       /* ≥15/20 拍欠压 → 放电 */
                     : (Uint16)WORK_MODE_CHARGE;         /* 否则充电 */
        }
    }

    RunState.workMode = workMode;                        /* ③ 判向结果落位 */
    RunState.canCmd = CAN_CMD_NONE;                      /*    CAN 指令已消费 */
    RunState.precheckLowCnt = 0U;                        /*    票数清零(下一趟从零起) */

    /* 起机跳过 STANDBY：RampInitCold → RAMP_CLOSE */
    SM_StartClosedLoop(workMode, 0U);                    /* 冷启播种路径(不清禁向位) */
}

/**
 * @brief  [状态2] RELAY: 继电器吸合确认 —— 高低压侧压差收敛判定
 *
 * 【流程】每拍两条并行时间线:
 *   超时线: ++stateTick ≥750 拍(1500ms)仍未收敛 → Fault_SetRelay(slow 故障);
 *   收敛线: vDiff=|VM 陷波 − PCS 100ms 滤波| 压差 → ≥30V 视为未吸合/弹跳, relayStable
 *   清零重计 → 连续 250 拍(500ms) <30V → 判定吸合完毕 → GPIO。
 *
 * 【重难点·双计数器并行】超时用 stateTick(进态清零后单调递增), 收敛用
 * relayStable(可反复清零) —— 二者语义相反(一个必须单调、一个允许回退),
 * 不能合并进一个计数器, 否则每次弹跳会同时重置超时线, 继电器卡死将永不报故障。
 *
 * 【openTest 旁路】台架无高压源时两侧压差恒超限, 开环测试强制视为已闭合,
 * 跳过压差检查 —— 仅工程测试有效, 量产运行不会置位该标志。
 */
static void Handle_Relay(void)
{
	float vDiff;

    if (++RunState.stateTick >= SM_T_RELAY_TIMEOUT)   /* 超时线: 1500ms 未收敛 */
    {
        Fault_SetRelay();                             /* → slow.relay 故障停机 */
        return;
    }

    vDiff = fabsf(stADC.VmBusVolt.f32Notch - stADC.PcsBusVolt.Filt.f32Filt100ms);  /* VM 陷波 vs PCS 100ms 滤波 */
    if (CfgParam.SysCmd.bit.openTest == 1U)
    {
        vDiff = 0.0f;                                 /* openTest: 旁路压差检查 */
    }

    if (vDiff >= SM_V_BUS_DIFF_MAX)                   /* 压差仍大: 未吸合/触点弹跳 */
    {
        RunState.relayStable = 0U;                    /* 收敛计数回退重计 */
        return;
    }

    if (++RunState.relayStable < SM_T_RELAY_STABLE)   /* 收敛线: 未满 500ms */
    {
        return;
    }

    RunState.relayStable = 0U;                        /* 归零, 供下一趟起机复用 */
    SM_EnterState(SM_STATE_GPIO);                     /* → GPIO 缓冲 */
}

/**
 * @brief  [状态3] GPIO: 继电器动作后的硬件电平建立缓冲 + 使能门极驱动
 *
 * 【流程】固定驻留 25 拍(50ms) —— 给继电器触点弹跳余波/驱动电源轨建立留时间;
 *   到期后 DrvEn 置 1(解除 DAB 门极驱动封锁) → 清预检票数 → PRECHECK。
 *
 * 【原理】驱动使能必须晚于继电器吸合确认: 若母线尚未接通就使能驱动, 软起阶段
 * 变压器会带空载/半载母线发波, 产生不可控的电流冲击。
 */
static void Handle_Gpio(void)
{
    if (++RunState.stateTick < SM_T_GPIO_DELAY)   /* 50ms 缓冲未满 */
    {
        return;
    }

    AppGpio_DrvEnSet(1U);                         /* 使能 DAB 门极驱动 */
    RunState.precheckLowCnt = 0U;                 /* 预检票数清零 */
    SM_EnterState(SM_STATE_PRECHECK);             /* → PRECHECK 判向 */
}

/**
 * @brief  [状态5] RAMP_CLOSE: 斜坡到位判定(闭环本体在 1ms/ISR)
 *
 * 【流程】
 *   ① workMode 合法性防御: 非充非放则本拍不判完成;
 *   ② 最短驻留 10ms(stateTick，进态清零): 防零拍直穿 RUN;
 *   ③ Ctrl_RampDone() → RUN。
 * 软起：本态连续 50s（tick 进态清）；进 RUN 当场清 softStartArmed。
 *
 * 【重难点·计时归属】本态 stateTick 由 SM_Step_2ms 尾部自增，仅服务最短驻留。
 */
static void Handle_RampClose(void)
{
    Uint16 workMode;

    workMode = RunState.workMode;
    if ((workMode != (Uint16)WORK_MODE_CHARGE) &&
        (workMode != (Uint16)WORK_MODE_DISCHARGE))
    {
        return;                                   /* 工况非法: 不判完成(防御分支) */
    }

    if (RunState.stateTick < SM_T_RAMP_CLOSE_MIN)  /* ② 最短驻留 10ms 未满 */
    {
        return;
    }

    if (Ctrl_RampDone() == 0U)                     /* ③ 斜坡未到位(电压/限值任一差) */
    {
        return;
    }

    RunState.softStartArmed = 0U;                  /* 唯一 RUN 入口：当场结束冷启监视 */
    SM_EnterState(SM_STATE_RUN);
}

/* ==========================================================================
 * 对外 API
 * ========================================================================== */

/**
 * @brief  状态机冷态初始化(上电调用一次)
 *
 * 【流程】SysCmd/RunState 全零 → 初始态 INIT → 状态位图对齐 → 默认主机角色。
 * 从机角色由通讯层运行时改写 RunState.isMaster。
 */
void SM_Init(void)
{
    memset((void *)&SysCmd, 0, sizeof(SysCmd));       /* 命令镜像清零(全默认) */
    memset((void *)&RunState, 0, sizeof(RunState));   /* 运行上下文清零 */
    RunState.state = SM_STATE_INIT;                   /* 初始态: INIT 等待放行 */
    RunState.staFlag.all = (Uint16)(1U << (Uint16)SM_STATE_INIT);  /* 位图对齐 */
    RunState.isMaster = 1U;                           /* 默认主机角色 */
}

/**
 * @brief  2ms 主步进: 关机/故障裁决 → 状态分发 → 观测发布 → 计时维护
 *
 * 【流程】
 *   ① UpdateShutdownFlag(): 必须最先执行 —— 故障/关机响应延迟上界=2ms, 且保证
 *      本拍后续句柄看到的是"已处理停机"的状态(触发即回 INIT);
 *   ② switch 分发: 各态句柄见上方逐一注释;
 *      - STANDBY/RUN 无 2ms 句柄: STANDBY 驻留/出态在 100µs 任务; RUN 稳态
 *        全部闭环动作在 ISR, 状态机无事可做;
 *      - default: OstStop + 回 INIT —— 任何跑飞的状态值(位翻转等)无条件收敛
 *        到硬件封波停机(fail-safe 设计);
 *   ③ staFlag 位图发布: 每拍按当前态重建(1U<<state), 上位机观测专用;
 *   ④ RAMP_CLOSE 最短驻留: 尾部 stateTick++（进态清零，与软起超时无关）;
 *   ⑤ openTestTime: 开环测试或 RUN 发波时长观测。
 * 软起超时在 Fault_Detect5ms（Fault_SoftStartTick5ms），不在此调用。
 *
 * 【重难点·执行顺序】①与②的先后不可调换: 若先分发后裁决, 故障拍内旧态句柄
 * 仍会执行一次(如 RAMP_CLOSE 判完成进 RUN), 虽然下一拍会被裁决拉回, 但多出
 * 一拍无谓动作; 先裁决保证"故障/关机拍 = 立即停机拍"。
 */
void SM_Step_2ms(void)
{
    UpdateShutdownFlag();                          /* ① 最高优先级: 关机/故障裁决 */

    switch (RunState.state)                        /* ② 状态分发 */
    {
        case SM_STATE_INIT:         Handle_Init();       break;
        case SM_STATE_RELAY:        Handle_Relay();      break;
        case SM_STATE_GPIO:         Handle_Gpio();       break;
        case SM_STATE_PRECHECK:     Handle_Precheck();   break;
        case SM_STATE_STANDBY:      break; /* 100us SM_Standby_100us */
        case SM_STATE_RAMP_CLOSE:   Handle_RampClose();  break;
        case SM_STATE_RUN:          break;
        default:
            DAB_PWM_OstStop();
            RunState.softStartArmed = 0U;
            SM_EnterState(SM_STATE_INIT);
            break;
    }

    RunState.staFlag.all = (Uint16)(1U << (Uint16)RunState.state);   /* ③ 观测位图 */

    if (RunState.state == SM_STATE_RAMP_CLOSE)     /* ④ 最短驻留计时 */
    {
        RunState.stateTick++;
    }

    if ((CfgParam.SysCmd.bit.openTest != 0U) || (RunState.state == SM_STATE_RUN))
    {
        RunState.openTestTime++;                   /* ⑤ 发波时长观测计数 */
    }
}
