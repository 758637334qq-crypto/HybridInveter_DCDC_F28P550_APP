/*************************************************************************************************
Copyright 2011, celink Corporation.
File name:       App_DataCalc.h
Description:     CAN 遥测汇总（把运行采样/设定/状态整理进 g_CanMon，供 0x11 上报）
*************************************************************************************************/

#ifndef INCLUDE_APP_DATACALC_H
#define INCLUDE_APP_DATACALC_H

extern void Update_CanMon(void);    /* 由 Task_5ms 调用 */

#endif /* INCLUDE_APP_DATACALC_H */
