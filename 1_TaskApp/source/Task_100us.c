/*********************************************************************************
* File:        Task_100us.c
* Description: DAB 100us 节拍片（CPU-Timer2 时基，10 kHz）
*              只做调度装配，具体功能在 Isr_/App_/Bsp 模块里。
***********************************************************************************/

#include "Includes.h"

/**
 * @brief  100us 任务体（与 CPU-Timer2 调度时基同拍）
 * @note   只放需要高实时性的环节：
 *           - 待机/换向：SM_Standby_100us；
 *           - 放电前馈比例：Dab_Ctrl_DisFfUpdate100us（慢变量除法预算）。
 *         CAN 接收 / SCI 接收拼帧链路有硬件 FIFO 缓冲，1ms 轮询足够，已移至 Task_1ms。
 */
void Task100us(void)
{
    SM_Standby_100us();
    Dab_Ctrl_DisFfUpdate100us(); /* 放电前馈比例：100us 预算一次，ISR 只取用 */
}

/**
 * @brief  TaskSched 入口：统一 void(void *) 原型
 */
void Task100us_Entry(void *arg)
{
    (void)arg;
    Task100us();
}
