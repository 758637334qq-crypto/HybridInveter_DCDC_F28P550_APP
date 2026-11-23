/*********************************************************************************
* File:        Task_5ms.c
* Description: DAB 5ms 节拍片（陷波参数 / 慢保护 / CAN 遥测汇总 / CAN 内务 / 串口协议）
*              只做调度装配，具体功能在 Isr_/App_ 模块里。
***********************************************************************************/

#include "Includes.h"

/**
 * @brief  5ms 任务体
 * @note   顺序：陷波 → 软起计时(SM) → 慢保护 → 遥测汇总 → 串口遥测 → CAN 内务
 */
void Task5ms(void)
{
    /* 陷波参数：按 g_Notch2f.f32LineHz / f32K 双缓冲发布 */
    Notch2f_ParamUpdate5ms();

    SM_SoftStartTick5ms(); /* 软起计时在 SM；满则 Fault_SetSoftTo（排在 Fault_Detect5ms 之前，受开路测试门控统一约束） */
    Fault_Detect5ms();
    Update_CanMon();

    /* 串口遥测：排在 Update_CanMon 之后，本拍即发送最新遥测/故障快照（RX 已在 Task_1ms 处理） */
    UartProto_Task_5ms();

    CanComm_Task_5ms(); /* Bus-Off / INIT / ASM 恢复 */
}

/**
 * @brief  TaskSched 入口：统一 void(void *) 原型
 */
void Task5ms_Entry(void *arg)
{
    (void)arg;
    Task5ms();
}
