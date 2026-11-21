/*********************************************************************************
* File:        Task_50ms.c
* Description: DAB 50ms 节拍片；均流在 App_CurShare 内再分频到 100ms。
***********************************************************************************/

#include "Includes.h"

/**
 * @brief  50ms 任务体
 * @note   调用 CurShare_Task_50ms；真正 AverCal 为 100ms（对齐 0x41）。
 */
void Task50ms(void)
{
    CurShare_Task_50ms();
}

/**
 * @brief  TaskSched 入口：统一 void(void *) 原型
 */
void Task50ms_Entry(void *arg)
{
    (void)arg;
    Task50ms();
}
