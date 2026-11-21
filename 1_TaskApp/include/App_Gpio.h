/*************************************************************************************************
 * App_Gpio.h  —  数字输出调用层（继电器 / 驱动使能 / 软启）
 * 脚号与复用配置在 ConfigChip_BoardCfg；本模块只 writePin。
 *************************************************************************************************/
#ifndef APP_GPIO_H_
#define APP_GPIO_H_

#include "f28x_project.h"
#include "ConfigChip_BoardCfg.h"

void AppGpio_Init(void);              /* 上电后拉低功率侧 */
void AppGpio_PowerSafeOff(void);
void AppGpio_SafeOff(void);           /* 功率侧 + 加热膜全关 */

void AppGpio_RelayBusSet(Uint16 on);
void AppGpio_SoftRlySet(Uint16 on);
void AppGpio_DrvEnSet(Uint16 on);

#endif /* APP_GPIO_H_ */
