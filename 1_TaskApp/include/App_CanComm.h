/*************************************************************************************************
 * App_CanComm.h  —  CAN 应用协议（唯一协议层；底层仅 ConfigChip_McanInit）
 *
 * 职责：
 *   - 0x11 遥测、0x20 整定、示波器 0x1F 上行
 *   - 1ms RX→TX、5ms Bus-Off/INIT/ASM 恢复（无 100us RX、无 CAN 地址握手）
 *   - 逻辑地址由 UART 分址（bCalIDCmd）；0x41 组包/解包转交 App_CurShare
 *************************************************************************************************/
#ifndef APP_CAN_COMM_H
#define APP_CAN_COMM_H

#include <stdint.h>

/* BORAD_* / ADDRcmd_* 唯一定义见 ConfigChip_McanInit.h（Includes.h 已先包含） */

#ifdef __cplusplus
extern "C" {
#endif

/** 协议状态初始化（须在 init_Drv_can / init_global_data 之后） */
void CanComm_Init(void);

/** UART 分址：撤未发 0x41 → 写 uIndex/bCalIDCmd → ResetPeers */
void CanComm_SetLocalAddr(uint16_t u16Addr);

/** 1ms：0x41 均流广播 + 0x11 遥测 + 示波器 */
void CanComm_Task_Tx_1ms(void);
/* ★ 0x41 诊断：硬件确认成功、未成功或未提交的帧数。 */
extern volatile uint32_t g_u32CanShareTxOk;
extern volatile uint32_t g_u32CanShareTxMiss;

/** 1ms：取空 RX FIFO 并解析协议帧 */
void CanComm_Task_Rx(void);

/** 5ms：InitPoll 推进 / Bus-Off·ASM 诊断；失败空闲后 100ms Reinit */
void CanComm_Task_5ms(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CAN_COMM_H */
