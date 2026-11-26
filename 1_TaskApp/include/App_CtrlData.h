/*************************************************************************************************
 * App_CtrlData.h — DAB 控制数据与任务侧接口
 *
 * 数据沿给定往下走：
 *   Ramp / RampTarget     1ms 斜坡当前值与终点
 *   g_CtrlBusRef_V        发布给 ISR 的电压给定 = Ramp.v + CurShare.voltAdj
 *   Bus                   共用电压外环，输出总电流给定
 *   stDabPi0 / stDabPi1   两路电流环，各取总给定的一半
 *   stDabDuty0 / 1        两路发波记忆
 *   stDabParam            当前充/放电工况；任务装载，充电 ISR 更新 f32SrRatio
 *
 * 任务接口：
 *   Ctrl_Init           INIT 复位斜坡、均流补偿和环路运行态
 *   Ctrl_StandbyClear   STANDBY 入口仅清均流；环清零在出态
 *   Ctrl_SetMode        封波关中断窗装工况并清内环/Duty
 *   Ctrl_RampCfg        每 1ms 读配置并推进一步斜坡
 *   Ctrl_RampDone       电压和两向电流限值是否都到位
 *   Ctrl_RampInitCold   冷启：清 Bus + 装 Ramp（放电10A / 充电−2A）
 *   Ctrl_RampInitSwitch 换向：清 Bus + 装 Ramp（放电满设定 / 充电压差增益）
 *   Ctrl_CalcGiven      有效给定+禁向位图（与冷启/1ms 同一口径，供 PRECHECK 判向拍）
 *
 * 环路计算在 Interrupt_CtrlDab.c。双桥 PWM 均启用。
 * 使用本头的 .c 须先包含 Includes.h。
 *************************************************************************************************/
#ifndef APP_CTRL_DATA_H
#define APP_CTRL_DATA_H

/*==============================================================================
 * 变压器与整机电流限值
 *============================================================================*/
#define DAB_TURNS_RATIO       (8.0f)      /* 匝比 n：高压 400V / 低压 50V */
#define DAB_CURR_LIMIT        (20.0f)     /* 整机母线电流给定的绝对上限(A) */

/*==============================================================================
 * 工况
 *============================================================================*/
/* 任务在封波窗口装载工况字段；充电 ISR 每拍更新 f32SrRatio。
 * 任务与 ISR 共享，必须 volatile。
 * 充电硬件占空比上限用宏 DAB_CHG_D_LOW_MAX，不再镜像到本结构体。 */
typedef struct
{
    float  f32SrRatio;                     /* 同步整流：高压 / (匝比 × 低压) */
    Uint16 u16IsCharge;                    /* 1 充电，0 放电。符号由它推导 */
} ST_DAB_PARAM;

/*==============================================================================
 * 电压外环（两路共用）
 * 正电流放电，负电流充电。1ms 发布斜坡，位置式 PI 出总电流，两路各取 1/2。
 * 无电压环电流前馈：f32Integ 单独保存，f32Out 是限速、限幅后的 I+Kp·e，f32Ref 来自 f32Out。
 * 大信号 = Ki_FAST 助推 + 连续预测退坡 + 加载向输出限步。
 *============================================================================*/
typedef struct
{
    float f32Vref;         /* 本拍电压给定：g_CtrlBusRef_V = Ramp.v + CurShare.voltAdj */
    float f32Err;          /* 滤波后电压误差，低通系数 DAB_BUS_ALPHA */
    float f32Err1;         /* 上一拍未夹位误差：差分与预测基准 */
    float f32Integ;        /* 电压外环积分(A)。输出限幅不回写这里 */
    float f32Out;          /* 限速、限幅后的 I+Kp·e，总电流给定主通道 */
    float f32Ref;          /* 总电流给定(A)：正放电、负充电 */

    Uint16 u16Tick;        /* ISR 相位：0 算外环，4 算陷波 */
    Uint16 u16RecoverRamp; /* 1：换向播种后，电压给定按偏差分档快爬 */
} ST_DAB_BUS;

/*==============================================================================
 * 电流内环（每路一份）
 * Δu = K1·e(k) − K2·e(k−1)，K1 = Kp + Ki·Ts，K2 = Kp，结果累加在 f32Out。
 * 占空比前馈叠在电流环输出上：放电为模型反解 u_ff，充电为开方 d_ff。发波占空比在 ST_DAB_DUTY。
 *============================================================================*/
typedef struct
{
    float f32Ref;                          /* 本路电流给定(A)：带符号——充电存负幅值、放电存正 */
    float f32Err1;                         /* 上一拍误差 */

    float f32Out;                          /* 充电：电流修正(A)；放电：增量 PI 校正 */
    float f32OutMax;                       /* 放电内环输出上限 */

    float f32K1;                           /* Kp + Ki·Ts */
    float f32K2;                           /* Kp */

    float f32DutyFf;                       /* 占空比前馈：放电 u_ff；充电直接存本拍实际占空比（CAN 观测用） */

    Uint16 u16DutySat;                     /* 上一拍内环输出顶上限标志：放电=Out夹位(OutMax-Ff)处判定，充电=电流指令SAT或占空比fmin夹位。均在夹位点精确比较，避免回加Ff的ULP漂移，供外环抗饱和判据读 */
} ST_DAB_PI;

/*==============================================================================
 * 发波记忆（每路一份）
 * 充电存低压侧占空比；放电存 0~1 相对档，再换算成发波。
 *============================================================================*/
typedef struct
{
    float  f32Duty;                        /* 交给 ePWM 的占空比 */
    float  f32SrDuty;                      /* 充电同步整流占空比；放电不用 */
    Uint16 u16SrEnable;                    /* SR：关断看瞬时功率单拍即撤，开通需连续确认 */
    Uint16 u16SrOnCnt;                     /* SR 开通确认计数：连续 N 拍过门限才置使能 */
    Uint16 u16SrSoft;                      /* SR 软起计数：使能后窗口按 u16SrSoft/N 线性放开 */
} ST_DAB_DUTY;

/*==============================================================================
 * 1ms 斜坡
 * v：电压给定。iPos：放电上限(≥0)。iNeg：充电下限(≤0)。
 * 电压常规 1V/ms，换向后按偏差分档加快。电流限值每毫秒最多放宽 2A，收紧立即生效。
 * 任务侧逐字段写入。运行中换向在封波关中断窗口内提交。
 *============================================================================*/
typedef struct
{
    float v;
    float iPos;
    float iNeg;
} ST_DAB_RAMP;

/*==============================================================================
 * 对象
 *============================================================================*/
extern volatile ST_DAB_PARAM stDabParam;
extern volatile ST_DAB_BUS Bus;
extern ST_DAB_PI stDabPi0, stDabPi1;
extern ST_DAB_DUTY stDabDuty0, stDabDuty1;

extern ST_DAB_RAMP Ramp;                               /* 当前电压给定 / 放电限 / 充电限 */
extern ST_DAB_RAMP RampTarget;                         /* 终点，供 Ctrl_RampDone 与均流 Iset */
extern float g_CtrlBusRef_V;                           /* ISR 电压给定 = Ramp.v + voltAdj */
/* g_stDabIsrState 由 Interrupt_CtrlDab.h 发布 */

/*==============================================================================
 * 任务接口
 *============================================================================*/
void Ctrl_Init(void);                                  /* INIT：复位斜坡、补偿和环路运行态 */
void Ctrl_StandbyClear(void);                          /* STANDBY 入口：仅清均流；Bus/PI/Duty 由出态 SetMode+RampInit 清 */
void Ctrl_SetMode(Uint16 workMode);                    /* 封波关中断窗：装工况并清内环/Duty */

void Ctrl_RampCfg(void);                               /* 每 1ms 读目标并推进一步 */
Uint16 Ctrl_RampDone(void);                            /* 1：电压和两向限值都到位 */

void Ctrl_RampInitCold(float f32VoltFbk);              /* 冷启斜坡初值 */
void Ctrl_RampInitSwitch(float f32VoltFbk);            /* 换向/出待机斜坡初值 */
Uint16 Ctrl_CalcGiven(float *pf32Dis, float *pf32Chg); /* 有效给定+禁向：判向拍一次算出，冷启装载复用 */

#endif /* APP_CTRL_DATA_H */
