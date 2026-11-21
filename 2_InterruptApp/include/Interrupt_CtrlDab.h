/*************************************************************************************************
 * Interrupt_CtrlDab.h — DAB EPWM10_ISR 对外接口与整定参数
 *
 * 本头只放三类东西：整定宏、ISR 状态类型、对外函数声明。
 * 算法与实现说明见 Interrupt_CtrlDab.c；使用本头的 .c 须先包含 Includes.h。
 *************************************************************************************************/
#ifndef ISR_CTRL_DAB_H_
#define ISR_CTRL_DAB_H_

/* 限幅：先取上限再取下限（lo > hi 时 lo 优先，与原实现一致）。
 * x 在宏内求值两次，只可传纯表达式。 */
#define DAB_SAT(x, lo, hi)   __fmax(__fmin((x), (hi)), (lo))

/*==============================================================================
 * 一、并联桥与放电前馈软起
 *============================================================================*/
#define DAB_BRIDGE_NUM          (2U)                            /* 并联桥数 */
#define DAB_CURR_PER_BRIDGE     (1.0f / (float)DAB_BRIDGE_NUM)  /* 每桥给定 = 总给定 × 0.5 */

/* 前馈软起：本路给定（总给定的一半）达到 5A 时系数到 1，即总给定 10A 时前馈满 */
#define DAB_FF_SOFTSTART_A      (5.0f)
#define DAB_FF_SOFTSTART_K      (1.0f / DAB_FF_SOFTSTART_A)

/*==============================================================================
 * 二、充电发波边界
 *============================================================================*/
#define DAB_CHG_DUTY_LV_MAX     DAB_CHG_D_LOW_MAX  /* 低压桥上限，单一事实源在 ConfigChip_EpwmInit.h */
#define DAB_CHG_DUTY_SR_MAX     (0.45f)            /* SR 独立限幅，不与 DutyMax 联动 */

/* DAB_CHG_D_TOP / DAB_CHG_D_LOW_MAX 必须 < 0.5：CMP = 2·d·TBPRD，d ≥ 0.5 时
 * 比较沿越过周期沿，桥臂恒通并与互补臂重叠直通。浮点常量不能 #if，
 * 断言在 Interrupt_CtrlDab.c 用整数源（_MILLI）做编译期检查。 */
#define DAB_LV_DUTY_HARD_MAX    (0.5f)
#define DAB_HV_DUTY_HARD_MAX    (1.0f)

/*==============================================================================
 * 三、放电开环分段映射（u ∈ [0,1] → HV / LV 占空比）
 *
 *   u < 0.03         零指令，不发波
 *   0.03 ≤ u ≤ 0.30  HV 固定 45%，LV = 10% + 0.667·u
 *   0.30 < u ≤ 0.50  HV 固定 45%，LV = 30% + 0.70·(u-0.30)
 *   0.50 < u ≤ 1.00  LV 锁 44%，  HV = 45% + slope·(u-0.50)
 *
 * 先开 LV（增大移相角），LV 到顶后再动 HV，功率从零平滑上升，避免冲击电流。
 *============================================================================*/
#define DAB_U_MIN_PACKET    (0.03f)   /* 打嗝门限：低于 3% 视为零指令（刻意无迟滞） */
#define DAB_DUTY_LV_MIN     (0.10f)   /* 再小脉宽驱动不了 */
#define DAB_DUTY_LV_MAX     (0.44f)   /* 接近 50% 就直通 */
#define DAB_DUTY_HV_BASE    (0.45f)   /* 略小于 50%，留死区 */
#define DAB_U_MID1          (0.30f)
#define DAB_U_MID2          (0.50f)
#define DAB_LV_SLOPE1       (0.66667f)
#define DAB_LV_SLOPE2       (0.70f)
#define DAB_HV_SLOPE        (DAB_DISCHARGE_HV_SLOPE)

/* 增益调度：环路增益与映射斜率成正比。以第二段斜率为基准，用“基准/当前”
 * 反向放大 PI 增量；倍率与斜率在同一分段边界切换，乘积恒为 DAB_LV_SLOPE2。 */
#define DAB_SCHED_S5_SCALE_1  (DAB_LV_SLOPE2 / DAB_LV_SLOPE1)  /* 段① 只动 S5 */
#define DAB_SCHED_S5_SCALE_2  (1.0f)                            /* 段② 基准 */
#define DAB_SCHED_S4_SCALE_3  (DAB_LV_SLOPE2 / DAB_HV_SLOPE)    /* 段③ 只动 S4 */

/*==============================================================================
 * 四、放电前馈曲线
 *============================================================================*/
#define DAB_DIS_FF_MAX          (0.70f)
#define DAB_DIS_FF_ERR_DEADBAND (5.0f)    /* V */
#define DAB_DIS_FF_KP           (0.0075f) /* 1/V */
#define DAB_DIS_FF_TRIM_MAX     (0.10f)   /* 基础前馈的 ±10% */

/*==============================================================================
 * 五、封波：强制 4 路 ePWM 全部输出低电平（灭弧）
 *
 * AQCSFRC = 0x0005 → A 路、B 路都强制低；RLDCSF = 3 → CPU 写直达活动寄存器，
 * 下一个 TBCLK 生效（不等装载事件）。发波函数反其道：先切 RLDCSF=0 解除强制，
 * 最后才武装 OSHTLD，保证装载事件看到的是完整一套新影子值。
 * 本宏依赖 DabEpwm*Regs，只可在包含 Includes.h 的 .c 中使用。
 *============================================================================*/
#define DAB_PWM_FORCE_LOW()                         \
    do {                                            \
        DabEpwm1Regs.AQSFRC.bit.RLDCSF = 3U;        \
        DabEpwm1Regs.AQCSFRC.all = 0x0005U;         \
        DabEpwm2Regs.AQSFRC.bit.RLDCSF = 3U;        \
        DabEpwm2Regs.AQCSFRC.all = 0x0005U;         \
        DabEpwm3Regs.AQSFRC.bit.RLDCSF = 3U;        \
        DabEpwm3Regs.AQCSFRC.all = 0x0005U;         \
        DabEpwm4Regs.AQSFRC.bit.RLDCSF = 3U;        \
        DabEpwm4Regs.AQCSFRC.all = 0x0005U;         \
        DabEpwm1bRegs.AQSFRC.bit.RLDCSF = 3U;       \
        DabEpwm1bRegs.AQCSFRC.all = 0x0005U;        \
        DabEpwm2bRegs.AQSFRC.bit.RLDCSF = 3U;       \
        DabEpwm2bRegs.AQCSFRC.all = 0x0005U;        \
        DabEpwm3bRegs.AQSFRC.bit.RLDCSF = 3U;       \
        DabEpwm3bRegs.AQCSFRC.all = 0x0005U;        \
        DabEpwm4bRegs.AQSFRC.bit.RLDCSF = 3U;       \
        DabEpwm4bRegs.AQCSFRC.all = 0x0005U;        \
    } while (0)

/*==============================================================================
 * 六、发波状态：每桥一个 Uint16，同时记录“上次写入的 AQ 动作值”和“当前相位”
 *
 *   bit[1:0] 充电 S1~S4 / 放电 Hv 上次写入的动作值（0=NO_ACTION，2=SET，3=作废）
 *   bit[3:2] 充电 S5~S8 / 放电 Lv 上次写入的动作值
 *   bit[5:4] 放电内部比较点（CLEAR / NO_ACTION）上次写入的动作值
 *   bit[8]   相位：0 = 已封波（下拍重写常量），1 = 发波中（跳过常量写）
 *
 * 充电与放电复用同一份字段：换向必先封波，封波时整字复位成 DAB_PWM_STATE_RESET。
 * 契约（改动前必读）：
 *   1. 只由 ISR 读写；任务侧最多重复写“强制低”，同值属安全方向；
 *   2. 释放（写发波值）只允许 Dab_Pwm_ChgUpdate / Dab_Pwm_DisUpdate 做；
 *   3. 不能读回 AQCTL：影子寄存器读回的是当前有效值，可能滞后一拍，
 *      必须用软件变量记上次写进去的值；
 *   4. 两态够用的前提是“换向必先封波”。若将来支持运行中直接换向，
 *      必须恢复三态，否则放电会沿用充电的 AQCSFRC=0，S1/S3 不常闭；
 *   5. 任何新增的 AQCSFRC 写入口都必须同步把相位置 OFF。
 *============================================================================*/
#define DAB_PWM_STATE_RESET     (0x003FU)

typedef union
{
    Uint16 all;                 /* 整字复位 / 整组比较 */
    struct
    {
        Uint16 u16S1S4Act : 2;  /* 充电 S1~S4 / 放电 Hv */
        Uint16 u16S5S8Act : 2;  /* 充电 S5~S8 / 放电 Lv */
        Uint16 u16InnerAct : 2; /* 放电内部比较点 */
        Uint16 u16Rsvd0   : 2;
        Uint16 u16Phase   : 1;  /* 0 = 已封波，1 = 发波中 */
        Uint16 u16Rsvd1   : 7;
    } bit;
} DabAqLast_t;

/*==============================================================================
 * 七、DAB ISR 运行状态（全文件唯一可变状态块，定义在 Interrupt_CtrlDab.c）
 *
 *   ① 跨节拍预算：任务侧写、ISR 只读；
 *   ② 两桥发波动作值缓存：ISR 私有；
 *   ③ 充/放电发波状态：ISR 写、任务侧只读；打嗝锁存两工况互斥，换向自动重判。
 * u16ChgBuckOn / u16BurstOff 是 ISR 内部迟滞锁存，
 * 任务侧只允许在关中断窗内通过 Dab_Ctrl_SessionReset() 复位，不要直接写。
 *============================================================================*/
typedef struct
{
    /* ① 跨节拍预算 */
    float  f32InvBatVolt;   /* 1/Vbat；0 = 不可用（BatCurr 输出 0） */
    float  f32DisFedDuty;   /* 放电前馈比例 Vset/(n·Vbat)-1，100us 预算；-1 = 零前馈 */

    /* ② 两桥 AQ 动作值缓存 */
    DabAqLast_t stChgAqBr1;
    DabAqLast_t stChgAqBr2;

    /* ③ 充/放电发波状态（两工况互斥，打嗝锁存共用） */
    Uint16 u16ChgBuckOn;    /* 充电：1 = Buck 压差足够，允许发波（迟滞） */
    Uint16 u16BurstOff;     /* 充放电共用：1 = 轻载休眠（迟滞，判决方向随工况镜像） */
    Uint16 u16ChgPwmActive; /* 充电：1 = PWM 实际在发波 */
    float  f32ChgLimitA;    /* 充电：整机有效充电电流幅值 [A] */
    float  f32CapFilt;      /* 充电：能力缓放值 [A]，下降当拍跟随，上升每拍限步 */
} ST_DAB_ISR_STATE;

extern volatile ST_DAB_ISR_STATE g_stDabIsrState;

/* EPWM10 零点中断（DAB 控制 ISR，定义在 Interrupt_CtrlDab.c，RAM 运行）
 * 向量注册：ConfigChip_DspInit.c → PieVectTable.EPWM10_INT + PIEIER3.INTx10 */
extern __interrupt void EPWM10_ISR(void);

/*==============================================================================
 * 七A、充电内环拍级参数包（Interrupt_CtrlDab.c 内部使用）
 *
 *   两路共用的本拍快照：Main 算一次填一包，两路传同一指针，
 *   收敛内环形参（9→4）。栈上聚合，不新增全局变量。
 *============================================================================*/
typedef struct
{
    float f32Vslow;     /* 陷波滤波后的母线电压(V) */
    float f32Iref;      /* 单路给定幅值(A)，恒为正 */
    float f32InvDv;     /* 1/压差，逆模型系数 */
    float f32DutyMax;   /* 本拍占空比上限 */
    float f32IcmdMax;   /* 本拍电流指令上限(A)，恒为正 */
} ST_DAB_CHG_CTX;

/*==============================================================================
 * 八、对外接口（任务侧调用）
 *============================================================================*/

/**
 * @brief  1ms 任务：刷新 BatCurr 用的 1/Vbat
 * @details 分母取 stADC.BatVolt 的 10ms 滤波值；低于 DAB_BAT_VOLT_MIN 写 0，
 *          ISR 侧据此直接输出 BatCurr = 0。须在 AdcDabFilt1ms() 之后调用。
 */
void Dab_Ctrl_InvBatVoltUpdate(void);

/**
 * @brief  100us 任务：预算放电前馈比例 Vset/(n·Vbat) - 1
 * @details 电压给定 100us 才更新一次，Vbat 取 10ms 滤波值，无需每拍做除法；
 *          低于 DAB_BAT_VOLT_MIN 写 -1，前馈经限幅后归零。
 */
void Dab_Ctrl_DisFfUpdate100us(void);

/**
 * @brief  控制会话复位：清充/放电迟滞锁存与 AQ 动作值缓存
 * @details 由 SM_StartClosedLoop 在关中断窗内、Ctrl_SetMode 之后调用。
 *          调用时 PWM 必须已封波；本函数只清软件状态，不操作 PWM 硬件。
 */
void Dab_Ctrl_SessionReset(void);

#endif /* ISR_CTRL_DAB_H_ */
