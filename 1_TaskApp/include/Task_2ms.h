/*============================================================================
 * 文件:  Task_2ms.h
 * 平台:  TI C2000 F28P550SJ9（裸机）
 * 功能:  2ms 节拍片声明（主状态机单步 SM_Step_2ms）
 *===========================================================================*/
#ifndef TASK_2MS_H_
#define TASK_2MS_H_

void Task2ms(void);                 /* 2ms 任务体 */
void Task2ms_Entry(void *arg);      /* TaskSched 入口 */

#endif /* TASK_2MS_H_ */
