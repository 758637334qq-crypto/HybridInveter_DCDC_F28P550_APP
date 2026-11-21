/* App_UartProto.h — AA55 帧协议的常量、结构体与对外接口
 * 帧：AA 55 | ADDR | CMD | LEN | DATA[N] | CRC_L CRC_H（小端 CRC16） */

#ifndef APP_UARTPROTO_H_
#define APP_UARTPROTO_H_

#include <stdint.h>
#include <stdbool.h>

/* 帧头 0xAA 0x55；本机地址 0x01，广播 0xFF */
#define PROTOCOL_SOF1              (0xAAU)
#define PROTOCOL_SOF2              (0x55U)
#define PROTOCOL_MY_ADDR           (0x01U)
#define PROTOCOL_BROADCAST         (0xFFU)
/* 数据区最多 24 字节：TX 环 32 格可用 31（留一格区分满/空），整帧 7+N ≤ 31 → N ≤ 24 */
#define PROTOCOL_MAX_PAYLOAD       (24U)
#define PROTOCOL_FRAME_SIZE        (PROTOCOL_MAX_PAYLOAD + 7U)

/* BMS 下发 0x00~0x05，数据区均为 9 字节 */
#define PROTOCOL_BMS_PAYLOAD_LEN     (9U)
/* BMS 完整帧恒 7+9=16 字（LEN 入口钉死 9），RX 全路径按此定长 */
#define PROTOCOL_RX_FRAME_SIZE       (PROTOCOL_BMS_PAYLOAD_LEN + 7U)
#define PROTOCOL_CMD_RUN_0           0
#define PROTOCOL_CMD_SET_PACK_BUS_5  5

/* 帧队列 4 格，& MASK 绕回（容量须为 2 的幂） */
#define UART_PROTO_RX_SLOT_NUM     (4U)
#define UART_PROTO_RX_SLOT_MASK    (UART_PROTO_RX_SLOT_NUM - 1U)

#define UART_PROTO_FRAME_GAP_MS    (3U)   /* 半帧超时 3ms 后丢弃残帧 */
#define UART_PROTO_TELEMETRY_TICKS (4U)   /* 4×5ms = 20ms 发一帧 */

/* 收帧状态：等 AA → 等 55 → 收 3 字节头 → 收数据+CRC */
typedef enum
{
    eUartRxWaitSof1 = 0,
    eUartRxWaitSof2,
    eUartRxHead,
    eUartRxBody
} teUartRxState;

/* 0x05 并联/母线设定帧，LEN=9 */
typedef struct
{
    uint16_t packSum;
    uint16_t busVolSet_dV;
    uint16_t busDischgCurSet_dA;
    uint16_t openLoopDuty;
    uint16_t rsvd;
} UartBmsBusFrame_t;

/* BMS 侧暂存：仅保留有使用方的字段（soc/相数/模式/校准数据无读取端，已删） */
typedef struct
{
    uint16_t bmsAddr;     /* 0x00 Byte6：BMS 地址（变更分址） */
    uint16_t hotCmd;      /* 0x00 Byte7 bit1：加热控制，0=关 1=开 */
    uint16_t PackSum;     /* 0x05：电池包总数 */
} UartBmsSide_t;

/* 遥测映射行：命令码 + 4 个变量地址，NULL 填 0 */
typedef struct
{
    uint16_t cmd;
    const uint16_t *w[4];
} UartTxMap_t;

/* 协议总状态块；跨任务共享成员一律 volatile */
typedef struct
{
    uint16_t      rxFrameBuf[PROTOCOL_RX_FRAME_SIZE];   /* 一字节一格，只存低 8 位（恒 16 字） */
    uint16_t      u16RxFramePos;
    teUartRxState eRxState;
    volatile uint32_t u32LastRxByteTick;

    volatile uint16_t rxFrameQueue[UART_PROTO_RX_SLOT_NUM][PROTOCOL_RX_FRAME_SIZE];   /* 完整帧队列（恒 16 字）：1ms 写 / 5ms 读 */
    volatile uint16_t u16RxWritePos;
    volatile uint16_t u16RxReadPos;
    volatile uint16_t u16RxFrameDropCnt;   /* 帧队列满被丢弃的帧数（诊断用） */

    /* 空闲计数每 5ms 加 1；分别监视字节接收和带控制位的 0x00 帧。 */
    volatile uint16_t u16RxIdleCnt;
    volatile uint16_t u16RunCmdIdleCnt;

    uint16_t u16TelemetryTick;
    uint16_t u16TxCmdIdx;
    volatile uint16_t u16TxDropCnt;
} UartProtoCtrl_t;

extern UartProtoCtrl_t g_UartProto;

void UartProto_Init(void);   /* 上电清零 */
void UartProto_RxTask(void);         /* rxRing → 拼帧（1ms 任务）；须排在 CustomUart_Task_Rx 之后 */
void UartProto_Task_5ms(void);   /* 取帧 → 执行命令 → 轮发遥测（地址/CRC 已在收帧入口过滤） */
/* 组帧发送；false = TX 环空间不足 */
bool UartProto_SendPacket(uint16_t addr, uint16_t cmd,
                          const uint16_t *payload, uint16_t len);

/** 0x05 下发的并联模块数，均流 0x42 上报用 */
uint16_t UartProto_GetPackSum(void);

/** 0x00 Byte7 bit1：加热控制，供 App_Gpio 软 PWM 使用 */
uint16_t UartProto_GetHotCmd(void);

#endif
