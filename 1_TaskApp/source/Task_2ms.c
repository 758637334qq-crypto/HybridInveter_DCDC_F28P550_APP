/*********************************************************************************
* File:        Task_2ms.c
* Description: DAB 2ms 节拍片（主状态机单步）
*              只做调度装配，具体功能在 App_StateMachine.c 里。
***********************************************************************************/

#include "Includes.h"

/**
 * @brief  2ms 任务体：状态机按 2ms 离散节拍推进一步
 */
void Task2ms(void)
{
    SM_Step_2ms();
}

/**
 * @brief  TaskSched 入口：统一 void(void *) 原型
 */
void Task2ms_Entry(void *arg)
{
    (void)arg;
    Task2ms();
}
