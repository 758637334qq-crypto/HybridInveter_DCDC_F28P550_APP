/*************************************************************************************************
 * App_Gpio.c  —  数字输出调用层（只写电平，不配脚）
 *************************************************************************************************/
#include "Includes.h"

void AppGpio_Init(void)
{
    AppGpio_PowerSafeOff();
}

void AppGpio_PowerSafeOff(void)
{
    GPIO_writePin(BOARD_GPIO_RELAY_BUS1P, 0U);
    GPIO_writePin(BOARD_GPIO_RELAY_BUS1N, 0U);
    GPIO_writePin(BOARD_GPIO_RELAY_BUS2P, 0U);
    GPIO_writePin(BOARD_GPIO_RELAY_BUS2N, 0U);
    GPIO_writePin(BOARD_GPIO_DRV_EN, 0U);
    GPIO_writePin(BOARD_GPIO_SOFT_RLY, 0U);
}

void AppGpio_SafeOff(void)
{
    AppGpio_PowerSafeOff();
    HeatFilm_Off();
}

void AppGpio_RelayBusSet(Uint16 on)
{
    Uint16 v = (on != 0U) ? 1U : 0U;
    GPIO_writePin(BOARD_GPIO_RELAY_BUS1P, v);
    GPIO_writePin(BOARD_GPIO_RELAY_BUS1N, v);
    GPIO_writePin(BOARD_GPIO_RELAY_BUS2P, v);
    GPIO_writePin(BOARD_GPIO_RELAY_BUS2N, v);
}

void AppGpio_SoftRlySet(Uint16 on)
{
    GPIO_writePin(BOARD_GPIO_SOFT_RLY, (on != 0U) ? 1U : 0U);
}

void AppGpio_DrvEnSet(Uint16 on)
{
    GPIO_writePin(BOARD_GPIO_DRV_EN, (on != 0U) ? 1U : 0U);
}
