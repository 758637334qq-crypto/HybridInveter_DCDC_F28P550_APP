/*************************************************************************************************
 * App_CanComm.h  —  CAN 应用协议（唯一协议层；底层仅 ConfigChip_McanInit）
 *
 * 职责：
 *   - 0x11 遥测、0x20 整定、示波器 0x1F、地址握手收尾
 *   - 1ms TX / 100us RX / 5ms Bus-Off 与地址计时
 *   - 0x41 转交 App_CurShare
 *************************************************************************************************/
#ifndef APP_CAN_COMM_H
#define APP_CAN_COMM_H

#include <stdint.h>

#define BORAD_ALL               0
#define BORAD_TO                1
#define ADDRcmd_end             1
#define ADDRcmd_on              0

#ifdef __cplusplus
extern "C" {
#endif

/** 协议状态初始化（须在 init_Drv_can 之后） */
void CanComm_Init(void);

/** 1ms：0x41 均流广播 + 0x11 遥测 + 示波器 */
void CanComm_Task_Tx_1ms(void);
/* ★ 0x41 诊断：硬件确认成功、未成功或未提交的帧数。 */
extern volatile uint32_t g_u32CanShareTxOk;
extern volatile uint32_t g_u32CanShareTxMiss;

/** 100us：取空 RX FIFO 并解析协议帧 */
void CanComm_Task_Rx(void);

/** 5ms：地址计时 + Bus-Off 恢复 */
void CanComm_Task_5ms(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CAN_COMM_H */
