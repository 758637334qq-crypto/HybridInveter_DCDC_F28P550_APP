/*********************************************************************************
* File:        App_UartProto.h
* Description: SCIB (AA55) BMS/上位机通讯协议（公共定义）
*              模块结构：驱动 FIFO 直递（Task_1ms 拼帧/校验/当拍执行），
*              5ms 只做遥测轮发；状态块 g_UartProto 模块私有。
***********************************************************************************/
#ifndef APP_UART_PROTO_H
#define APP_UART_PROTO_H

#include <stdint.h>
#include <stdbool.h>

/* ---- 帧格式（见 App_UartProto.c 文件头）：帧头 0xAA 0x55；本机地址 0x01，广播 0xFF ---- */
#define PROTOCOL_SOF1               (0xAAU)
#define PROTOCOL_SOF2               (0x55U)
#define PROTOCOL_MY_ADDR            (0x01U)
#define PROTOCOL_BROADCAST          (0xFFU)
/* BMS 下发 0x00~0x05，数据区均为 9 字节 */
#define PROTOCOL_CMD_RUN_0          (0x00U)
#define PROTOCOL_CMD_CAL_BUS12_1    (0x01U)
#define PROTOCOL_CMD_CAL_PCS_I1_2   (0x02U)
#define PROTOCOL_CMD_CAL_I2_ITOT_3  (0x03U)
#define PROTOCOL_CMD_CAL_VBAT_IBAT_4 (0x04U)
#define PROTOCOL_CMD_SET_PACK_BUS_5 (0x05U)

/* 本机轮发上报 0x40~0x52 */
#define PROTOCOL_CMD_TX_FIRST       (0x40U)
#define PROTOCOL_CMD_TX_LAST        (0x52U)

/* 收帧：定长 16 字节（SOF2+addr+cmd+len+9数据+CRC2），LEN 字节恒 9 */
#define PROTOCOL_RX_FRAME_SIZE      (16U)
#define PROTOCOL_BMS_PAYLOAD_LEN    (9U)

/* 发帧：数据区最多 24 字节：TX 环 32 格可用 31（留一格区分满/空），整帧 7+N ≤ 31 → N ≤ 24 */
#define PROTOCOL_MAX_PAYLOAD        (24U)
#define PROTOCOL_FRAME_SIZE         (PROTOCOL_MAX_PAYLOAD + 7U)
#define PROTOCOL_TELEMETRY_DIV      (4U)        /* 5ms×4=20ms 轮发一帧遥测 */

/* ---- 公共 API ---- */
void UartProto_Init(void);
void UartProto_RxTask(void);        /* 1ms：3ms 断帧 → 驱动 FIFO 直递 → 校验 → 当拍执行（字节同步喂 Boot） */
void UartProto_Task_5ms(void);      /* 5ms：20ms 轮发一帧遥测 */
bool UartProto_SendPacket(uint16_t addr, uint16_t cmd,
                          const uint16_t *pData, uint16_t len);
uint32_t UartProto_CmdIdleMs(void); /* 最近有效 0x00 距今毫秒数：App_Fault 的 SCI 掉线判据 */
uint16_t UartProto_GetHotCmd(void);     /* 0x00 Byte7 bit1：加热命令（App_HeatFilm 查询） */
uint16_t UartProto_GetPackSum(void);    /* 0x05：电池包总数（App_CurShare 查询） */

#endif /* APP_UART_PROTO_H */
