/*************************************************************************************************
Copyright 2022, CELink Corporation.									                                    	*
VERSION:										                                            			*
File name:		Config_SysParaInit.h								                                       	*
Description:		System Parameters Initial.							                                				*
Update Date:			2026-11-20 清理 280039 遗留死代码									                                    			*
Writer:			zy									                                            	*
*************************************************************************************************/
/*------------------------------------------------------------------------------------------------
 * 2026-11-20 移植清理：删除 280039 旧平台遗留的 396 个未引用宏与 11 组未引用
 * typedef/union/enum（PV/MPPT/BOOST/继电器 GPIO/NTC 温度表/BUS 电压表/ADJ 校准等）。
 * 保留口径：全工程 .c/.h（不含 TI 库/Debug/对比目录）有实际引用的内容：
 *   - 通用宏：NULL/FALSE/TRUE/ON/OFF
 *   - p550 DAB 比例宏：BUS_VOLT_RATIO/LOAD_VOLT_RATIO/BUS_CURR_RATIO/BUS_CURR_MID_ADC
 * 被删内容需要时从 SVN 历史（280039 基线）找回。
 *------------------------------------------------------------------------------------------------*/
#ifndef INCLUDE_CONFIG_SYSPARAINIT_H
#define INCLUDE_CONFIG_SYSPARAINIT_H

//--------------------------------------------
//     Generic define
//--------------------------------------------
#define NULL				0
#define FALSE				0
#define TRUE				1

#define ON    		1
#define OFF   		0

/*************************************************************************************************
 * 以下为 p550 DAB 工程移植的比例宏（ADC 码值→物理量，单位：V/码、A/码）。
 * 比例沿用 280039 DAB 工程（同一套采样调理电路），换硬件调理时必须同步修改。
 *************************************************************************************************/

/* --- DAB 功率通道（资源表管脚；比例仍按现有调理，上板重标） --- */
#define BUS_VOLT_RATIO          0.16716926f     /* 母线：VBUS1/2 + VMBUS(继电器VM) + VPBUS(PCS端) */
#define LOAD_VOLT_RATIO         0.25341796f     /* 电池：A10 VDC_ADC */
#define BUS_CURR_RATIO          0.02221679f     /* 电流：A7 IDD1 / B8 IDD2（中点偏置） */
#define BUS_CURR_MID_ADC        (2048.0f)       /* 0 A 对应码，约 1.65 V */

#endif /* INCLUDE_CONFIG_SYSPARAINIT_H */
