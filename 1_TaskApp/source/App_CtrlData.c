/*************************************************************************************************
 * App_CtrlData.c — DAB 控制数据、有效给定合成与 1ms 斜坡（不依赖 RunState / CurShare）
 *   ↑ 一句话：本文件只把上位机给的数变成能用的给定，不判断充还是放，也不算环路；状态机说走哪条路，这里只把数值摆好、限好、慢慢爬过去。
 *
 * 职责分层：
 *   1. 对象宿主机与复位 — Ctrl_Init / Ctrl_SetMode（管 stDabParam / Bus / stDabPi0..1 / Duty）
 *   ① 对象宿主机与复位：这几个全局变量（stDabParam / Bus / stDabPi0..1 / Duty）的家，别人用 extern 引，初始化也在这里做。
 *   2. 有效给定合成     — Ctrl_CalcGiven（协议换算 → 充电功率降额 → 硬上限 → ≤0.5A 禁向）
 *   ② 有效给定合成：四道关口一道比一道严，任何一道把数压成 0，这一向就发不出去。
 *   3. 斜坡步进与播种   — Ctrl_RampStep1ms / Ctrl_RampDone / Ctrl_RampInitCold / Switch
 *   ③ 斜坡步进与播种：播种=给斜坡一个合理起点；步进=每 1ms 往前挪一点；Done=判断挪到位没有。
 *
 * 外部编排职责：
 *   - 状态机决定何时冷启/换向、写 RunState.curLowSealing、清均流（CurShare_ResetComp）；
 *   决定权不在本文件：何时冷启/换向、封什么波、清均流都由状态机决定，本文件只被调用。
 *   - Task_1ms 按「闭环判据 → Ctrl_RampStep1ms → CurShare_Update1ms → 发布 g_CtrlBusRef_V」装配；
 *   Task_1ms 的装配顺序是「闭环判据 → Ctrl_RampStep1ms → CurShare_Update1ms → 发布 g_CtrlBusRef_V」，所以最终发布值是 Ramp.v 再加均流微调；本文件只给 Ramp.v。
 *   - 电压外环和电流内环计算在 Interrupt_CtrlDab.c。
 *   本文件一个 PI 都不算，只装系数、清记忆；电压外环和电流内环都在 Interrupt_CtrlDab.c。
 *************************************************************************************************/
#include "Includes.h"
// 工程总头：带类型定义（Uint16 等）和全局对象声明。
#include "App_DabModel.h"
// DAB 模型宏：匝比、功率上限、电池电流上限、内环增益、放电输出上限等。

/*==============================================================================
 * 斜坡范围、步长与充电降额边界
 *   这一段全是可调旋钮：行为想变基本只改这里，不动逻辑。
 *============================================================================*/
// 常态下 1ms 只挪 1V。例：给定 400V、当前 380V，差 20V → 20ms 走完。
#define DAB_VOLT_RAMP_STEP      (1.0f)     /* 常态电压步长：1V/ms */
// 换向快爬时，剩余压差缩到 4V 以内就退回 1V/ms，别冲过头。
#define DAB_VOLT_FAST_EXIT      (4.0f)     /* 换向快爬退出偏差(V) */
// 高门槛 40V：压差超过它走最快一档。
#define DAB_VOLT_FAST_BAND_HI   (40.0f)    /* 快爬分档压差门槛(V) */
#define DAB_VOLT_FAST_BAND_MID  (25.0f)
// 中门槛 25V。
#define DAB_VOLT_FAST_BAND_LO   (12.0f)
// 低门槛 12V。
// 高门槛对应步长 10V/ms。
#define DAB_VOLT_FAST_STEP_HI   (10.0f)    /* 对应分档步长(V/ms) */
#define DAB_VOLT_FAST_STEP_MID  (6.0f)
// 中门槛对应 6V/ms。
#define DAB_VOLT_FAST_STEP_LO   (3.0f)
// 低门槛对应 3V/ms。
#define DAB_VOLT_FAST_STEP_MIN  (2.0f)
// 最慢快爬档 2V/ms（压差 4~12V 时用），原则是差得远走得快、快到了踩刹车。

// 只对放宽限速：2A → 20A 要 9ms；收紧不受限速，当拍立刻到位。
#define DAB_CURR_RAMP_STEP      (2.0f)     /* 电流限值放宽步长：2A/ms */
// 上电待机时正负各 ±2A，反正没发波，只是让变量有个合法值。
#define DAB_IDLE_CURR_A         (2.0f)     /* INIT 复位占位电流限值幅值(A) */
// 冷启进充电不从 −20A 起步，先给 −2A，再每 ms 放宽 2A（9ms 到 −20A）。
#define DAB_CHG_INIT_A          (2.0f)     /* 充电冷启：Ramp.iNeg 起步幅值(A) */
// 冷启进放电先给 +10A，目标 20A 的话 5ms 后放宽到 20A。
#define DAB_DIS_INIT_A          (10.0f)    /* 放电冷启：Ramp.iPos 起步幅值(A) */
// 压差达到 40V 及以上，预偏增益满幅 = 1。
#define DAB_INIT_VERR_FULL_V    (40.0f)    /* 压差≥此值：初值增益=1（满幅） */
#define DAB_INIT_VERR_FULL_INV  (1.0f / DAB_INIT_VERR_FULL_V)
// 预先存好 1/40，运行时少做一次除法：gain = min(|Vset−Vact|/40, 1)。
// 播种时先把给定往目标方向「骗」一段（最多 10V），让环路一开始就有劲。
#define DAB_VOLT_INIT_BIAS_MAX  (10.0f)    /* Ramp.v 初值朝设定最多预偏(V) */
// 电压到位容差：浮点数几乎不会精确相等，所以用容差，不用 ==。
#define DAB_RAMP_VOLT_ERR       (0.5f)     /* 电压到位判据(V) */
// 电流限值到位容差 0.05A。
#define DAB_RAMP_CURR_ERR       (0.05f)    /* 电流到位判据(A) */

// 降额只在 0 < Vbus < 500 之间有效，注意不含端点。
#define DAB_CHG_VBUS_MIN_V      (0.0f)     /* 充电功率降额有效母线电压下界(V，不含) */
// 上界同样不含：Vbus 恰好 500V 时降额直接给 0，等于禁止充电。
#define DAB_CHG_VBUS_MAX_V      (500.0f)   /* 充电功率降额有效母线电压上界(V，不含) */

/*==============================================================================
 * 控制对象
 *============================================================================*/
volatile ST_DAB_PARAM stDabParam;
// volatile：ISR 也要读，编译器不许把它缓存在寄存器里。
volatile ST_DAB_BUS   Bus;
// 电压外环运行态：误差、积分、输出、总电流给定，只在 ISR 里算。
volatile ST_DAB_PI    stDabPi0, stDabPi1;
// 两路桥各承担总电流的一半，所以一人一份内环。
volatile ST_DAB_DUTY  stDabDuty0, stDabDuty1;
// 两路桥各自的发波记忆：占空比、同步整流占空比、SR 使能/计数。

volatile ST_DAB_RAMP  Ramp;
// 「现在在哪」：当前电压给定、放电上限 iPos(≥0)、充电下限 iNeg(≤0)。ISR 只读。
ST_DAB_RAMP           RampTarget;
// 「要去哪」：终点值。到位判断和均流都用它，所以不能只留一个 Ramp。
volatile float        g_CtrlBusRef_V;
// 给 ISR 的电压给定。本文件里先贴初值，Task_1ms 再加均流微调后发布。

/* 换向快爬标志：仅本模块 1ms 斜坡读/清、换向播种置位，不放进 ISR 外环结构体。 */
static Uint16 s_u16RecoverRamp;
// 1 = 正在换向快爬；0 = 常态 1V/ms。static 是告诉别的文件别碰它。

/*==============================================================================
 * 基础原语（限幅 / 单步跟踪 / 写入）
 *============================================================================*/
static inline float sClampF(float f32V, float f32Lo, float f32Hi)
// 【函数解析】把 f32V 夹在 [Lo, Hi] 之间。inline：每个 1ms 要叫好几次。
{
    return __fmin(__fmax(f32V, f32Lo), f32Hi);
    // 先抬到不低于下限，再压到不高于上限；两个都是编译器浮点内建，比库函数快。
    // 顺序不能反（先 fmax 后 fmin），否则下限大于上限时会给出错值；本工程常量保证不会。
}

/* 偏差大于一步只挪一步，不足一步直接落到目标。 */
static inline float sRampTrack(float f32Value, float f32Target, float f32Step)
// 【函数解析】单步跟踪：所有斜坡的「发动机」。
// 公式：delta = 目标 − 当前；|delta| > step 只走 step，否则一步到位。
{
    float f32Delta = f32Target - f32Value;
    // 正数=要往上走，负数=要往下走。

    if (f32Delta > f32Step)
    // 还差得远（往上方向）：
    {
        return f32Value + f32Step;
        // 本拍只加一个步长。例：380→400，step=1 → 381。
    }
    if (f32Delta < -f32Step)
    // 还差得远（往下方向）：
    {
        return f32Value - f32Step;
        // 本拍只减一个步长。
    }
    return f32Target;
    // 剩下的差距不足一步，直接等于目标：保证斜坡一定收敛，不会永远差一点。
}

/* 统一写当前斜坡与终点：调用方保证电流限值已在有效域内。 */
static inline void sRampWrite(float f32V, float f32Pos, float f32Neg,
                              float f32TgtV, float f32TgtPos, float f32TgtNeg)
                              // 【函数解析】一次把「当前值」和「终点」成对写进去。
                              // 为什么要成对：只写 Ramp 不写 RampTarget，Ctrl_RampDone 会立刻误判到位；只写 RampTarget 不写 Ramp，斜坡会从旧位置重新爬。
{
    RampTarget.v    = f32TgtV;
    // 先写终点（「应该到哪」），三个一起。
    RampTarget.iPos = f32TgtPos;
    RampTarget.iNeg = f32TgtNeg;
    Ramp.v          = f32V;
    // 再写当前值（「现在在哪」）。
    Ramp.iPos       = f32Pos;
    Ramp.iNeg       = f32Neg;
}

/*==============================================================================
 * 环路与发波复位
 *============================================================================*/
static void sBridgeResetOne(volatile ST_DAB_PI *pstPi, volatile ST_DAB_DUTY *pstDuty,
                            float f32K1, float f32K2)
                            // 【函数解析】复位一路桥：清内环记忆 + 清发波记忆 + 装增益，封波时调用。
                            // 增量式 PI：Δu = K1·e(k) − K2·e(k−1)，K1 = Kp + Ki·Ts，K2 = Kp。
{
    pstPi->f32Ref     = 0.0f;
    // 电流给定清 0：封波期间不许留任何电流指令。
    pstPi->f32Err1    = 0.0f;
    // 上一拍误差清 0：不清的话第一拍 Δu 会带进旧的 e(k−1)，产生跳变。
    pstPi->f32Out     = 0.0f;
    // 内环输出（累计的占空比校正量）清 0。
    pstPi->f32K1      = f32K1;
    // 按当前工况装增益：充电装 0，放电装 Kp 那套。
    pstPi->f32K2      = f32K2;
    pstPi->f32DutyFf  = 0.0f;
    // 占空比前馈清 0：防止上一段的模型反解值直接拿去发波。
    pstPi->u16DutySat = 0U;
    // 抗饱和标志清 0 =「没顶上限」，外环据此决定要不要停积分。

    pstDuty->f32Duty     = 0.0f;
    // 实际发给 ePWM 的占空比清 0（等于封波）。
    pstDuty->f32SrDuty   = 0.0f;
    // 充电同步整流的占空比清 0。
    pstDuty->u16SrEnable = 0U;
    // 同步整流使能关闭：SR 一开等于给体二极管并一条低阻通路，必须先关。
    pstDuty->u16SrOnCnt  = 0U;
    // 开通确认计数清 0：重新开 SR 要再连续确认 N 拍，不能接着上次数。
    pstDuty->u16SrSoft   = 0U;
    // 软起计数清 0：下次开 SR 的占空比从 0 慢慢放开，不是一上来满幅。
}

static void sBridgeResetBoth(float f32K1, float f32K2)
// 【函数解析】两路桥一起复位：必须同参数、同拍清，否则两桥出力不对称。
{
    sBridgeResetOne(&stDabPi0, &stDabDuty0, f32K1, f32K2);
    sBridgeResetOne(&stDabPi1, &stDabDuty1, f32K1, f32K2);
}

// 【函数解析】电压外环「重新做人」：Vref 直接给新值，其余动态量全清。
/* 清外环动态量并把 Vref 贴到初值（避免 INIT 阶段残留上一会话旧给定）。 */
static void sBusSeed(float f32Vref)
{
    Bus.f32Vref  = f32Vref;
    // 给定贴新值，不是慢慢爬：这里就是要它立刻等于新起点。
    Bus.f32Err   = 0.0f;
    // 误差和上一拍误差一起清：外环的差分项和预测项都以它为基准。
    Bus.f32Err1  = 0.0f;
    Bus.f32Integ = 0.0f;
    // 积分清 0 = 没有历史电流记忆，避免上次运行积出来的电流一下子放出去。
    Bus.f32Out   = 0.0f;
    // 外环输出（总电流给定）清 0。
    Bus.f32Ref   = 0.0f;
    // 发给两路内环的总给定清 0。
    Bus.u16Tick  = 0U;
    // ISR 相位计数归零，让外环从固定节拍重新开始。
}

void Ctrl_Init(void)
// 【函数解析】上电初始化统一入口：斜坡摆到最低电压、限值 ±2A、外环清空、两路内环和发波全关、增益装 0。此时整机是「沉默」的。
{
    s_u16RecoverRamp = 0U;
    // 快爬标志清 0：初始化绝不快爬。
    // 当前值 = 终点值 = 360V/±2A，所以 Ctrl_RampDone 立刻返回「到位」，状态机可以马上往下走；DAB_VOLT_MIN=360V 是「再低会倒灌」的下限。
    sRampWrite(DAB_VOLT_MIN, DAB_IDLE_CURR_A, -DAB_IDLE_CURR_A,
               DAB_VOLT_MIN, DAB_IDLE_CURR_A, -DAB_IDLE_CURR_A);
    sBusSeed(DAB_VOLT_MIN);
    // 外环 Vref 贴 360V，其余清零。
    g_CtrlBusRef_V = DAB_VOLT_MIN;
    // 先直接贴一个安全值给 ISR，不等 Task_1ms 发布。
    sBridgeResetBoth(0.0f, 0.0f);
    // 两路增益装 0：初始化阶段内环不产生任何校正量。
}

/* 封波关中断窗：装工况与内环 PI 系数，并清两路内环/发波记忆。 */
void Ctrl_SetMode(Uint16 workMode)
// 【函数解析】切工况专用。必须在「已封波且关了中断」的窗口里调：要同时改 u16IsCharge 和两路增益，跑到一半被 ISR 打断会给出半套参数。
{
    if (workMode == (Uint16)WORK_MODE_CHARGE)
    {
        stDabParam.u16IsCharge = 1U;
        // 标记充电：ISR 据此走充电分支（符号、占空比算法都变）。
        sBridgeResetBoth(0.0f, 0.0f);
        // 充电段 K1=K2=0：内环不做增量 PI，占空比由前馈（开方）直接给，所以「清 0」不是漏配，是故意让 PI 通道闭嘴。
    }
    else
    {
        stDabParam.u16IsCharge = 0U;
        // 其它任何值都当放电处理（只有 CHARGE 是特例）。
        sBridgeResetBoth(DAB_DISCHARGE_KP + DAB_DISCHARGE_KI_TS, DAB_DISCHARGE_KP);
        // 放电装增量式 PI：K1 = Kp + Ki·Ts = 0.002 + 0.00333 ≈ 0.00533，K2 = Kp = 0.002。
        // 展开就是 Δu = Kp·(e(k)−e(k−1)) + Ki·Ts·e(k)，标准增量式 PI。
    }
}

/*==============================================================================
 * 有效电流限值与禁向合成（全工程唯一口径）
 * 顺序：协议换算 → 充电功率降额 → 整机硬上限 → ≤0.5A 禁向。
 *   顺序不能换：降额必须在硬上限之前，否则降额算出的 30A 会绕过 20A 硬顶。
 *============================================================================*/
static float sChargePowerLimitA(void)
// 【函数解析】把「充电功率 / 电池电流」限制折算成「母线侧电流上限」（负数）。
// 原理：电池侧最多吃 P = min(Pmax, Ibat_max·Vbat) 瓦；这些功率全从母线来，母线电压 Vbus，则母线电流 = P / Vbus；充电方向为负，所以取负号。
// 例：Vbat=50V、60A → 3000W（小于 3200W，取 3000W）；Vbus=400V → 3000/400 = 7.5A → 返回 −7.5A。
// 反例：Vbat=58V → 3480W > 3200W，取 3200W；400V → −8A。
{
    float f32Vbus = stADC.VmBusVolt.f32Notch;
    // 母线电压取陷波后的值：滤掉工频纹波，避免降额限值跟着纹波每拍抖。
    float f32Vbat = stADC.BatVolt.Filt.f32Filt10ms;
    // 电池电压取 10ms 滤波值：电池电压本身就慢，用慢值防止限值抖。

    // 四个条件全满足才算数：母线 0<Vbus<500（不含端点）、电池 30≤Vbat<60。
    // 任何一条不满足（采样没上来、电池没接）就判定「不能充电」。
    if ((f32Vbus > DAB_CHG_VBUS_MIN_V) && (f32Vbus < DAB_CHG_VBUS_MAX_V) &&
        (f32Vbat >= DAB_BAT_VOLT_MIN) && (f32Vbat < DAB_BAT_OVP_VOLT))
    {
        // 内层 fmin：3200W 和「电池能吃的 60A×Vbat」取小的那个，谁先到顶听谁的。
        // __divf32 是编译器浮点除法，比走标准库快；负号 = 充电电流为负，与全工程「正放负充」一致。
        return -__divf32(__fmin(DAB_CHARGE_POWER_MAX,
                                DAB_CHARGE_BAT_CUR_MAX * f32Vbat), f32Vbus);
    }
    return 0.0f;
    // 条件不满足返回 0 = 充电限值不许低于 0 = 等于禁止充电。
}

Uint16 Ctrl_CalcGiven(float *pf32Dis, float *pf32Chg)
// 【函数解析】算出本拍允许的放电上限和充电下限，并返回禁向位图（bit0=禁充、bit1=禁放）。
// 纯函数：不写全局，只通过两个指针把结果带出去，任何地方都能调。
{
    float  f32Dis;
    float  f32Chg;
    Uint16 u16Seal = 0U;
    // 位图初值 0 = 两向都允许，后面逐条往上加禁止位。

    f32Dis = (float)CfgParam.BusDischgCurSet * 0.1f;
    // 上位机放电设定是 0.1A/LSB，乘 0.1 换成 A。例：200 → 20.0A。
    f32Chg = -(float)CfgParam.BusChgCurSet * 0.1f;
    // 充电同样 0.1A/LSB，但加负号：充电流记负值，全工程一个符号体系。例：200 → −20.0A。
    f32Chg = __fmax(f32Chg, sChargePowerLimitA());
    // 两个都是负数，取较大者 = 取绝对值较小者 = 更严的那个。
    // 例：设定 −20A、降额给 −7.5A → 取 −7.5A；反例：降额条件不成立返回 0，而 0 比 −20 大 → 取 0 → 充电被关死。

    f32Dis = sClampF(f32Dis, 0.0f, DAB_CURR_LIMIT);
    // 放电夹到 0~20A：配置被写成负数或超 20A 都在这里收口。
    f32Chg = sClampF(f32Chg, -DAB_CURR_LIMIT, 0.0f);
    // 充电夹到 −20~0A：上限 0 表示「不允许往正方向充」，别把符号搞混。

    if (f32Dis <= SM_IBUS_DISCHAR_STANDBY_MIN)
    // 门槛 0.5A，是 ≤ 不是 <：设定 0.5A 这一拍就禁放。
    {
        f32Dis = 0.0f;
        // 限值直接归 0：不只是标记，数值上也不留 0.4A 的余量给环路钻。
        u16Seal |= SM_SEAL_NO_DISCHARGE;
        // 置 bit1（值 2）：本拍禁放，调用方据此封波。
    }
    if (f32Chg >= -SM_IBUS_CHAR_STANDBY_MIN)
    // 充电是负数，所以「幅值 ≤0.5A」写成 chg ≥ −0.5；等于 −0.5A 也禁。
    {
        f32Chg = 0.0f;
        // 充电限值归 0。
        u16Seal |= SM_SEAL_NO_CHARGE;
        // 置 bit0（值 1）：本拍禁充。
    }

    *pf32Dis = f32Dis;
    // 结果通过指针带出去，两个值都已在有效域内。
    *pf32Chg = f32Chg;
    return u16Seal;
}
// 【边界数字】放电设定 5(0.5A) → 禁放；6(0.6A) → 允许，限值 0.6A。充电设定 5 → chg=−0.5 → 禁充；6 → −0.6 < −0.5 → 允许。

/*==============================================================================
 * 1ms 斜坡步进与到位判定
 *============================================================================*/
/* 常态 1V/ms；s_u16RecoverRamp=1 时按剩余压差分档加快，进入 4V 以内回常态。 */
static float sVoltStepSelect(float f32Diff)
// 【函数解析】选本拍电压步长。入参是「目标−当前」（带符号，内部取绝对值）。
// 只有换向播种把 s_u16RecoverRamp 置 1 才快爬，其它时候一律 1V/ms。
{
    if (s_u16RecoverRamp == 0U)
    {
        return DAB_VOLT_RAMP_STEP;
        // 常态 1V/ms：没换向就别快爬，快了会冲击母线。
    }

    f32Diff = fabsf(f32Diff);
    // 快爬只看压差大小，不管往上还是往下。
    if (f32Diff <= DAB_VOLT_FAST_EXIT)    { return DAB_VOLT_RAMP_STEP;     }
    // ≤4V → 1V/ms。
    if (f32Diff > DAB_VOLT_FAST_BAND_HI)  { return DAB_VOLT_FAST_STEP_HI;  }
    // >40V → 10V/ms。
    if (f32Diff > DAB_VOLT_FAST_BAND_MID) { return DAB_VOLT_FAST_STEP_MID; }
    // 25~40V → 6V/ms。
    if (f32Diff > DAB_VOLT_FAST_BAND_LO)  { return DAB_VOLT_FAST_STEP_LO;  }
    // 12~25V → 3V/ms。
    return DAB_VOLT_FAST_STEP_MIN;
    // 4~12V → 2V/ms。
}
// 【分档数字】压差 60V：60/10 = 6ms 走完；20V：20/3 ≈ 7ms；8V：8/2 = 4ms。

/* 电压限速前进；电流限值放宽走 2A/ms 斜坡，收紧立即生效。返回本拍禁向位图。 */
Uint16 Ctrl_RampStep1ms(void)
// 【函数解析】1ms 主步进，只在闭环时被 Task_1ms 调，做四件事：
// ① 读电压设定并夹紧；② 重算两向电流限值；③ 各自往前挪一步；④ 写回并判断快爬是否该退出。
// 返回禁向位图给调用方封波用。
{
    float  f32VoltRef;
    // 局部变量先声明（老 C 编译器要求），都是本拍中间量。
    float  f32VoltStep;
    float  f32VoltNext;
    float  f32Dis;
    float  f32Chg;
    float  f32CurV;
    float  f32PosNext;
    float  f32NegNext;
    Uint16 u16Seal;

    f32VoltRef  = sClampF((float)CfgParam.BusVolSet * 0.1f, DAB_VOLT_MIN, DAB_VOLT_MAX);
    // 电压设定 0.1V/LSB → V（4000 → 400.0V），再夹到 360~450V。
    // 每拍重新读配置：上位机改设定后下一拍就生效，不用重启。
    u16Seal     = Ctrl_CalcGiven(&f32Dis, &f32Chg);
    // 每拍重算电流限值（降额依赖实时电压，必须实时算），顺便拿到禁向位图。

    f32CurV     = Ramp.v;
    // 当前电压先存出来：函数里写过 Ramp.v 之后再读就变味了。
    f32VoltStep = sVoltStepSelect(f32VoltRef - f32CurV);
    // 按剩余压差选步长：常态 1，换向快爬按档位。
    f32VoltNext = sRampTrack(f32CurV, f32VoltRef, f32VoltStep);
    // 电压往前走一步：最多一个步长，不足一步就到位。
    f32PosNext  = __fmin(sRampTrack(Ramp.iPos, f32Dis, DAB_CURR_RAMP_STEP), f32Dis);
    // 放电上限：先按 2A/ms 放宽，再和目标取「较小值」。
    // 放宽（0→20A）：track 只给 2A，min(2,20)=2 → 被限速；收紧（20→5A）：track 给 18A，min(18,5)=5 → 立即到位。这就是「收紧立即生效」。
    f32NegNext  = __fmax(sRampTrack(Ramp.iNeg, f32Chg, DAB_CURR_RAMP_STEP), f32Chg);
    // 充电下限对称处理：放宽（−2→−20）走 2A/ms；收紧（−20→−5）取 max 立即到位。

    if (fabsf(f32VoltRef - f32VoltNext) <= DAB_VOLT_FAST_EXIT)
    // 用「走完这一步之后」的压差判断，不是用走之前的。
    // 好处：快爬最后一拍一跨进 4V 就立刻清标志，下一拍直接 1V/ms，不会多冲一拍。
    {
        s_u16RecoverRamp = 0U;
        // 退出快爬，回到常态 1V/ms。
    }

    sRampWrite(f32VoltNext, f32PosNext, f32NegNext, f32VoltRef, f32Dis, f32Chg);
    // 一次写全：当前值给下一步的结果，终点给本拍目标（目标每拍可能被配置改）。
    return u16Seal;
    // 位图交回去，调用方按 bit 决定这一向封不封波。
}

Uint16 Ctrl_RampDone(void)
// 【函数解析】到位判断：电压 + 两个电流限值，三个都在容差内才算到位。
// 用途：状态机等「斜坡爬完了」再进下一步，避免带着斜坡去切模式。
{
    // 三个条件与：电压差 ≤0.5V、放电限差 ≤0.05A、充电限差 ≤0.05A → 1（到位）。
    // 用绝对值差和容差比，不用 ==：浮点斜坡最后一步是「落到目标」，但 0.1f 累加会带尾巴。
    return ((fabsf(RampTarget.v - Ramp.v) <= DAB_RAMP_VOLT_ERR) &&
            (fabsf(RampTarget.iPos - Ramp.iPos) <= DAB_RAMP_CURR_ERR) &&
            (fabsf(RampTarget.iNeg - Ramp.iNeg) <= DAB_RAMP_CURR_ERR)) ? 1U : 0U;
}

/*==============================================================================
 * 冷启 / 换向播种（关中断窗内由状态机调用；此时均流已清零）
 *============================================================================*/
/* 压差增益 = min(|Vref−Vact|/40V, 1)，Ramp.v 朝设定最多预偏 10V 且不越终点。
 * 冷启：放电≤10A、充电≥−2A；换向：放电满额，充电在 [冷启起步, 有效限额] 同增益插值。 */
static void sRampSeed(float f32VoltFbk, float f32Dis, float f32Chg, Uint16 u16IsSwitch)
// 【函数解析】播种 = 给斜坡一个「有劲又不冒进」的起点，冷启和换向共用一套：
// ① 电压：从实测值往设定方向预偏 gain×10V（压差越大预偏越多）；
// ② 电流：冷启用小值起步；换向时放电直接给满额，充电按同一个 gain 插值。
// 为什么预偏：纯从实测值 1V/ms 爬，压差 40V 要 40ms，这期间环路没给定、没出力。
{
    float f32VoltAct;
    float f32VoltRef;
    float f32VoltInit;
    float f32Gain;
    float f32ChgFloor;
    float f32Pos;
    float f32Neg;

    f32VoltAct  = sClampF(f32VoltFbk, 0.0f, DAB_VOLT_MAX);
    // 实测电压先夹到 0~450V：传感器坏掉给出 900V 时，别把斜坡起点带飞。
    f32VoltRef  = sClampF((float)CfgParam.BusVolSet * 0.1f, DAB_VOLT_MIN, DAB_VOLT_MAX);
    // 设定值和步进里同一套换算与夹紧，保证两边算的是同一个目标。
    f32Gain     = __fmin(fabsf(f32VoltRef - f32VoltAct) * DAB_INIT_VERR_FULL_INV, 1.0f);
    // gain = min(|Vset−Vact| / 40, 1)：压差 ≥40V 满预偏；20V → 0.5；4V → 0.1。压差越小越保守。
    f32VoltInit = sRampTrack(f32VoltAct, f32VoltRef, f32Gain * DAB_VOLT_INIT_BIAS_MAX);
    // 预偏是一步到位，但步长被 gain×10V 限制，所以最多往目标方向挪 10V。
    // 例：Vact=380、Vset=400 → gain=0.5 → 步长 5V → 起点 385V，剩 15V 走 1V/ms。
    // 反例：Vact=340、Vset=400 → gain=1 → 步长 10V → 起点 350V，剩 50V 交给斜坡。

    f32ChgFloor = __fmax(f32Chg, -DAB_CHG_INIT_A);
    // 充电「地板」= max(目标, −2A)：目标 −20A 时地板 −2A；目标 −1A 时地板就是 −1A。作用是不管目标多负，起步幅度都不超过 2A。
    if (u16IsSwitch != 0U)
    // 换向分支（多半是充→放）：
    {
        f32Pos = f32Dis;
        // 放电方向直接给满额，不再从 10A 慢慢放宽：换向前电流已被均流清零，不存在从零硬拽到 20A 的冲击。
        f32Neg = f32ChgFloor + f32Gain * (f32Chg - f32ChgFloor);
        // 充电方向在 [地板, 目标] 之间按 gain 插值。
        // 例：chg=−20、地板=−2、gain=0.5 → −2 + 0.5×(−18) = −11A；压差大（gain→1）就接近目标，压差小（gain→0）就贴地板。
        s_u16RecoverRamp = (fabsf(f32VoltRef - f32VoltInit) > DAB_VOLT_FAST_EXIT) ? 1U : 0U;
        // 预偏之后还剩 4V 以上才开快爬；剩不到 4V 就没必要快，直接 1V/ms 收尾。
    }
    else
    {
        f32Pos = __fmin(f32Dis, DAB_DIS_INIT_A);
        // 冷启放电：min(目标, 10A)。目标 20A → 从 10A 起步，之后 2A/ms 放宽，5ms 到 20A。
        // 目标本来只有 3A → 直接给 3A，不会先给 10A（min 保证不越过目标）。
        f32Neg = f32ChgFloor;
        // 冷启充电：直接用 −2A 地板起步，9ms 放宽到 −20A。
        s_u16RecoverRamp = 0U;
        // 冷启不开快爬：冷启时母线还没建立，快爬没意义还危险。
    }

    sRampWrite(f32VoltInit, f32Pos, f32Neg, f32VoltRef, f32Dis, f32Chg);
    // 起点给预偏值/起步电流，终点给真实目标（不预偏）；之后 Ramp.v 会从 f32VoltInit 一路 1V/ms（或快爬）走向 f32VoltRef。
    sBusSeed(f32VoltInit);
    // 外环 Vref 就贴到 Ramp.v 起点，误差/积分/输出全清 → 环路从干净状态起步。
    g_CtrlBusRef_V = f32VoltInit;
    // 立刻同步给 ISR 的电压给定，别让 ISR 用上一段的旧值跑一拍。
}

void Ctrl_RampInitCold(float f32VoltFbk, float f32Dis, float f32Chg)
// 【函数解析】冷启播种入口：u16IsSwitch=0 走「冷启」分支。
// 调用前状态机必须已封波、关中断、清均流，否则这里改 Ramp 会和 ISR 抢。
{
    sRampSeed(f32VoltFbk, f32Dis, f32Chg, 0U);
    // 0 = 冷启：放电 ≤10A、充电 = −2A、不快爬。
}

void Ctrl_RampInitSwitch(float f32VoltFbk, float f32Dis, float f32Chg)
// 【函数解析】换向播种入口：u16IsSwitch=1 走「换向」分支。
{
    sRampSeed(f32VoltFbk, f32Dis, f32Chg, 1U);
    // 1 = 换向：放电满额、充电按压差增益插值、压差 >4V 就开快爬。
}
