/*============================================================================
 * 文件:  Task_100us.h
 * 平台:  TI C2000 F28P550SJ9（裸机）
 * 功能:  100us 节拍片声明（待机/换向仲裁 + 放电前馈预算）
 *        CAN/SCI RX 已迁到 Task_1ms（硬件 FIFO 缓冲足够）
 *===========================================================================*/
#ifndef TASK_100US_H_
#define TASK_100US_H_

void Task100us(void);               /* 100us：SM_Standby + Dab_Ctrl_DisFfUpdate */
void Task100us_Entry(void *arg);    /* TaskSched 入口 */

#endif /* TASK_100US_H_ */
