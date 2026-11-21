/*============================================================================
 * 文件:  Task_100us.h
 * 平台:  TI C2000 F28P550SJ9（裸机）
 * 功能:  100us 节拍片声明（待机/换向 / CAN 接收 / SCI RX 拼帧）
 *===========================================================================*/
#ifndef TASK_100US_H_
#define TASK_100US_H_

void Task100us(void);               /* 100us：STANDBY + CAN 收 + CustomUart RX + UartProto 拼帧 */
void Task100us_Entry(void *arg);    /* TaskSched 入口 */

#endif /* TASK_100US_H_ */
