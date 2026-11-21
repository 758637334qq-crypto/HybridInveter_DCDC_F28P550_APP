/*********************************************************************************
* File:        App_UartProto.h
* Description: SCIB (AA55) BMS/上位机通讯协议（公共定义）
*              界面规则分区：1 协议常量 → 2 类型定义 → 3 变量声明 → 4 公共 API
*              模块结构：驱动 FIFO 直递（Task_1ms 拼帧/校验/当拍执行），
*              5ms 只做遥测轮发。
***********************************************************************************/
#ifndef APP_UART_PROTO_H
#define APP_UART_PROTO_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 1. 协议常量 ==================== */

/* 帧格式（见 App_UartProto.c 文件头）：帧头 0xAA 0x55；本机地址 0x01，广播 0xFF */
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

/* 半帧超时：3ms 无新字节作废残帧 */
#define UART_PROTO_FRAME_GAP_MS     (3U)

/* ==================== 2. 类型定义 ==================== */

/* 0x01~0x04 校准帧，LEN=9：前 8 字节 k/b，最后 1 字节保留 */
typedef struct
{
    uint16_t k1;
    int16_t  b1;
    uint16_t k2;
    int16_t  b2;
    uint16_t rsvd;
} UartBmsCalFrame_t;

/* 四组校准数据 */
typedef struct
{
    UartBmsCalFrame_t bus12;
    UartBmsCalFrame_t busPcsI1;
    UartBmsCalFrame_t i2Itot;
    UartBmsCalFrame_t vbatIbat;
} UartBmsCal_t;

/* BMS 侧暂存：协议有、固件暂无独立变量的量 */
typedef struct
{
    uint16_t soc_dPct;    /* 0x00 Byte0~1：SOC 0.1% */
    uint16_t hotCmd;      /* 0x00 Byte7 bit1：加热控制，0=关 1=开 */
    uint16_t phase;       /* 0x00 Byte7 bit5：相数选择，0=单相 1=三相（预留） */
    uint16_t bmsMode;     /* 0x00 mode bit1：BMS 模式，0=正常 1=BMS 模式 */
    uint16_t adiMode;     /* 0x00 mode bit3：校准模式，1=进入校准 */
    uint16_t PackSum;     /* 0x05：电池包总数 */
    UartBmsCal_t cal;
} UartBmsSide_t;

/* 遥测映射行：命令码 + 4 个变量地址，NULL 填 0 */
typedef struct
{
    uint16_t cmd;
    const uint16_t *w[4];
} UartTxMap_t;

/* 协议总状态块：类型开放给界面查看，实例定义在 App_UartProto.c */
typedef struct
{
    uint16_t          rxFrameBuf[PROTOCOL_RX_FRAME_SIZE];   /* 一字节一格，只存低 8 位（恒 16 字） */
    uint16_t          u16RxFramePos;   /* 收帧位置：0=等 0xAA，1=等 0x55，2~4=头，5~15=数据+CRC */
    volatile uint32_t u32LastRxByteTick;               /* 最后字节时刻：3ms 断帧判据 */
    volatile uint32_t u32LastCmdTick;                  /* 最后有效 0x00 时刻：SCI 掉线判据 */

    uint16_t          u16TelemetryTick;
    uint16_t          u16TxCmdIdx;
    volatile uint16_t u16TxDropCnt;
} UartProtoCtrl_t;

/* ==================== 3. 变量声明 ==================== */

extern UartProtoCtrl_t g_UartProto;   /* 实例定义在 App_UartProto.c；外部只读查询建议走 CmdIdleMs() */

/* ==================== 4. 公共 API ==================== */

void UartProto_Init(void);
void UartProto_RxTask(void);        /* 1ms：3ms 断帧 → 驱动 FIFO 直递 → 校验 → 当拍执行（字节同步喂 Boot） */
void UartProto_Task_5ms(void);      /* 5ms：20ms 轮发一帧遥测 */
bool UartProto_SendPacket(uint16_t addr, uint16_t cmd,
                          const uint16_t *pData, uint16_t len);
uint32_t UartProto_CmdIdleMs(void); /* 最近有效 0x00 距今毫秒数：App_Fault 的 SCI 掉线判据 */
uint16_t UartProto_GetHotCmd(void);     /* 0x00 Byte7 bit1：加热命令（App_HeatFilm 查询） */
uint16_t UartProto_GetPackSum(void);    /* 0x05：电池包总数（App_CurShare 查询） */

#endif /* APP_UART_PROTO_H */
