/*============================================================================
 * 文件:  Task_1ms.h
 * 平台:  TI C2000 F28P550SJ9（裸机）
 * 功能:  1ms 节拍片声明（滤波 / 1-Vbat / 缓起 / CAN 发送）
 *===========================================================================*/
#ifndef TASK_1MS_H_
#define TASK_1MS_H_

void Task1ms(void);                 /* 1ms 任务体（含串口 TX 搬运） */
void Task1ms_Entry(void *arg);      /* TaskSched 入口 */

#endif /* TASK_1MS_H_ */
