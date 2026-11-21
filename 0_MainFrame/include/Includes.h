/*************************************************************************************************
Copyright 2022, celink Corporation.									                                    	*								                                            			*
File name:		 Includes.h								                                       	*
Description:		System Include Head file 							                                				*
Update Date:		 2026/11/20（p550 DAB 移植：驱动/App/Task 三层，删除 CLA 与旧中断链路）		            				*
Writer:			zy									                                            	*
*************************************************************************************************/

#ifndef INCLUDE_INCLUDES_H
#define INCLUDE_INCLUDES_H

#include <math.h>
#include <string.h>

typedef unsigned char   Uint8;
typedef char            Int8;
typedef float         float32;
typedef double        float64;

#include "f28x_project.h"     // Device Header file and Examples Include File
#include "device.h"
#include "driverlib.h"

//-------------------------------------------------------------------------------
// 4_PhysicalChip 驱动层（p550 Bsp 移植）
//-------------------------------------------------------------------------------
#include "ConfigChip_DspInit.h"
#include "ConfigChip_SysCtrlInit.h"
#include "ConfigChip_AdcInit.h"			/* p550：ADC + 双 SOCA 槽位宏 DAB_ADC_* */
#include "ConfigChip_EpwmInit.h"		/* p550：DAB PWM（EPWM10 中断时基） */
#include "ConfigChip_CmpssInit.h"		/* p550：CMPSS1/2/4 → TRIP4 → 双桥 OST */
#include "ConfigChip_SpiInit.h"
#include "ConfigChip_BoardCfg.h"		/* p550：Board_Init / Board_PrepareForBoot */
#include "ConfigChip_DabPwmMap.h"		/* p550：DAB PWM 资源映射 */
#include "ConfigChip_CustomUart.h"		/* p550：SCIB 底层字节通道 */
#include "ConfigChip_McanInit.h"		/* p550：MCAN 经典 CAN */

#include "Config_SysParaInit.h"			/* 含 p550 DAB 比例宏（BUS_VOLT_RATIO 等） */

//-------------------------------------------------------------------------------
// 3_UcosCode 调度层
//-------------------------------------------------------------------------------
#include "TaskAppInclude.h"
#include "TaskOS.h"          //#include "OS_KERNEL.h"

//-------------------------------------------------------------------------------
// 2_InterruptApp 中断层（p550 Isr 移植：EPWM10 DAB 控制 + Notch2f）
//-------------------------------------------------------------------------------
#include "Interrupt_CtrlDab.h"
#include "Interrupt_Notch2f.h"

//-------------------------------------------------------------------------------
// 1_TaskApp 应用层（p550 App/Task 保名移植）
//-------------------------------------------------------------------------------
#include "App_AdcDataDeal.h"
#include "App_DabModel.h"
#include "App_CanComm.h"
#include "App_CtrlData.h"
#include "App_CurShare.h"
#include "App_DataCalc.h"
#include "App_Fault.h"
#include "App_Gpio.h"
#include "App_HeatFilm.h"
#include "App_StateMachine.h"
#include "App_UartProto.h"
#include "App_BootProto.h"		/* ARM 升级协议（原始工程 SCIB 内部通讯协议 boot 部分，与 UartProto 共存） */

#include "Task_100us.h"
#include "Task_1ms.h"
#include "Task_2ms.h"
#include "Task_5ms.h"
#include "Task_50ms.h"

#include "globaldatadef.h"

#include "TaskSched.h"					/* p550 CBB 调度器：调度主体未接线（主循环走 TaskOS），时基由 OSTimerTick 调用 */

extern Uint16 u6FirmwareCode[2];
extern Uint16 u16BootSoftwareVerison;
extern Uint16 u16BootSoftwareSubverison;
extern Uint16 u16SoftwareVerison;
extern Uint16 u16SoftwareSubverison;
extern Uint16 u16HardwareVerison;

#endif /* INCLUDE_INCLUDES_H */
