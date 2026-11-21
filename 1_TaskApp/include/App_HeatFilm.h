/*************************************************************************************************
 * App_HeatFilm.h  —  加热膜（GPIO 1 kHz 软 PWM）
 *
 * 资源表：GPIO27=DR_HOT（软 PWM），GPIO56=HOT-EN
 * 后台 HeatFilm_Task 跟 UART hotCmd；EPWM10_ISR 内 HeatFilm_Isr 只翻转占空。
 *************************************************************************************************/
#ifndef APP_HEAT_FILM_H_
#define APP_HEAT_FILM_H_

#include "f28x_project.h"
#include "ConfigChip_BoardCfg.h"

/* 脚号：Board 配置；此处别名供调用层 writePin */
#define HEAT_FILM_GPIO_PWM        BOARD_GPIO_HEAT_PWM
#define HEAT_FILM_GPIO_EN         BOARD_GPIO_HEAT_EN

/* 充电 50 kHz → 1 kHz=50 拍；放电 60 kHz → 1 kHz=60 拍 */
#define HEAT_FILM_TICKS_CHG       (50U)
#define HEAT_FILM_TICKS_DIS       (60U)

/** 运行时状态（管脚/周期拍数用宏，不占 RAM） */
typedef struct
{
    Uint16 enable;        /* 1=允许发波（跟 HOT-EN） */
    Uint16 dutyPct;       /* 目标占空 0~100 */
    Uint16 onTicksChg;    /* 充电周期内高电平拍数（后台算好） */
    Uint16 onTicksDis;    /* 放电周期内高电平拍数 */
    Uint16 cnt;           /* ISR 软 PWM 计数 */
} HeatFilm_t;

extern HeatFilm_t g_stHeatFilm;

void HeatFilm_Init(void);
void HeatFilm_Off(void);
void HeatFilm_SetEnable(Uint16 on);
void HeatFilm_SetDutyPct(Uint16 pct0to100);

/** 1ms：跟 hotCmd，更新 enable/duty/onTicks；关断时拉低并清 cnt */
void HeatFilm_Task(void);

/** EPWM10_ISR：仅按 g_stHeatFilm 翻转 GPIO，不做复位 */
void HeatFilm_Isr(Uint16 isCharge);

#endif /* APP_HEAT_FILM_H_ */
