/*************************************************************************************************
 * ConfigChip_EpwmInit.h  —  DAB PWM 底座（F28P550SG / EPWMCLK = SYSCLK = 150 MHz）
 *
 * 职责：只做发波配置与封波；无内部启停状态机、无软件故障策略。
 * 工况唯一来源：状态机 RunState.workMode（WORK_MODE_CHARGE / WORK_MODE_DISCHARGE）。
 * 硬件映射 / 分桥 GLD：见 ConfigChip_DabPwmMap.h（桥1@EPWM10，桥2@EPWM7）。
 *
 * 频率换算（up-down 模式，f = EPWMCLK / (2 * TBPRD)）：
 *   充电 50 kHz：TBPRD = 150e6 / (2 * 50e3) = 1500
 *   放电 60 kHz：TBPRD = 150e6 / (2 * 60e3) = 1250
 *   5 * 1500 = 6 * 1250 = 7500 → 充放两档折算的外环/陷波节拍都是 100 us（10 kHz）。
 *************************************************************************************************/
#ifndef INCLUDE_BSP_EPWMINIT_H
#define INCLUDE_BSP_EPWMINIT_H

#define DAB_TBCLK_FREQ_HZ   (150000000UL)  /* F28P550 SYSCLK = EPWMCLK */
#define DAB_CHG_TBPRD       (1500U)        /* 50 kHz */
#define DAB_DIS_TBPRD       (1250U)        /* 60 kHz */

/* 充电开关频率联动防呆：App_DabModel.h 中 DAB_CHARGE_KFL(1.75f)、
 * DAB_CHARGE_ALPHA_I、DAB_CHARGE_WZ_I_TS 均按 50 kHz(20us) 写死。
 * 调整 DAB_CHG_TBPRD 改频时，必须同步修改上述三个宏，否则此处编译报错。 */
#define DAB_CHG_FS_HZ         (DAB_TBCLK_FREQ_HZ / (2UL * (unsigned long)DAB_CHG_TBPRD))
_Static_assert(DAB_CHG_FS_HZ == 50000UL,
               "DAB_CHG_TBPRD changed: update DAB_CHARGE_KFL / DAB_CHARGE_ALPHA_I / DAB_CHARGE_WZ_I_TS in App_DabModel.h");

#define DAB_DUTY_MIN        (0.060f)
#define DAB_D_MIN           DAB_DUTY_MIN
#define DAB_CHG_D_TOP_MILLI (450U)         /* 固定发波占空比整数源(×1000)，供 C 层静态断言用 */
#define DAB_CHG_D_TOP       ((float)DAB_CHG_D_TOP_MILLI / 1000.0f)  /* 0.450 */
#define DAB_CHG_D_LOW_MIN   DAB_DUTY_MIN
/* 充电变量脉冲（HV 下管 S2/S4 后沿）占空比上限：必须 < DAB_CHG_D_TOP(0.45)，
 * 差值决定续流窗口 = (0.45-d)·2·Th。0.430 → 窗口 0.4us，恰覆盖近匹配点
 * 最大电流衰减尾(t2≈0.2us)；驱动关断延时+死区若超过 0.4us 需回退到 0.42。 */
#define DAB_CHG_D_LOW_MAX_MILLI (430U)     /* 变量脉冲占空比上限整数源(×1000) */
#define DAB_CHG_D_LOW_MAX   ((float)DAB_CHG_D_LOW_MAX_MILLI / 1000.0f)  /* 0.430 */
#define DAB_CHG_D_SR_MAX    (0.450f)
#define DAB_DIS_D_MIN       (0.450f)
#define DAB_DIS_D_MAX       (0.567f)
#define DAB_DIS_D_HV_MIN    DAB_DIS_D_MIN
#define DAB_DIS_D_HV_MAX    DAB_DIS_D_MAX
#define DAB_DIS_D_LV        (0.440f)
#define DAB_DIS_D_LV_MAX    DAB_DIS_D_LV


extern volatile Uint16 g_u16DabPwmTicks100us; /* 当前载波下 100us 的 PWM 拍数 */

void   DAB_PWM_Init(void);
void   DAB_PWM_Stop(void);              /* AQCSFRC 软件封波（换向/待机等） */
void   DAB_PWM_OstStop(void);           /* 关 INTEN → TZFRC.OST → 清 flag → 开 INTEN：故障/关机硬封波 */
void   DAB_PWM_Start(void);             /* 仅清 TZCLR.OST 开发波；不碰 INTEN */
void   DAB_PWM_SetMode(Uint16 workMode);/* 封波期间只提交 TBPRD/AQ；工况参数由 Ctrl_SetMode */

void InitChip_EpwmRegs(void);
void EPWMStartTimer(void);
void EPWM_TzEnable(void);

#define DAB_Charge_SFO_Step()       (1U)
#define DAB_PWM_SFO_Step()          (1U)
#define DAB_PWM_RunSfo1ms()         (1U)

#define DAB_FAULT_NONE          (0U)
#define DAB_FAULT_PARAM_LIMIT   (1U)
#define DAB_FAULT_HARDWARE_TRIP (2U)
#define DAB_FAULT_SFO_CALIB     (4U)
#define DAB_PWM_FAULT_NONE      (0U)
#define DAB_PWM_FAULT_CMD       (1U)
#define DAB_PWM_FAULT_TRIP      (2U)
#define DAB_PWM_FAULT_SFO       (4U)
#define DAB_PWM_STOPPED         (0U)
#define DAB_PWM_RUNNING         (1U)

#endif /* INCLUDE_CONFIGCHIP_EPWMINIT_H */
