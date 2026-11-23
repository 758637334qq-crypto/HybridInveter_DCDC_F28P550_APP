/*******************************************************************************
 * @file    ConfigChip_McanInit.h
 * @brief   DAB 标准 CAN 通讯（F28P550 MCAN，经典 CAN 2.0B，500 kbps，29 位扩展帧）
 *
 * MCAN RX FIFO0(WATCHID) + RX FIFO1(AVERID) + 3 个 TX Buffer；SCOPEID 仅上行、硬件滤除。
 *
 * 29 位扩展 ID 位域（与上位机协议一致，不可改）：
 *   Bit 0~18  : ADDR     目的地址（0xFF=上位机，0x10=并联平均广播，0x1F=示波器）
 *   Bit 19~26 : LogADDR  源逻辑地址（本机地址，0~255；由 UART 分址写入）
 *   Bit 27    : PTP      0=广播，1=点对点
 *   Bit 28    : ADDRcmd  线侧标志位（历史名）；本机发帧恒填 ADDRcmd_end=1
 *                        逻辑地址不再走 CAN 握手，由 UART 写 uIndex + bCalIDCmd
 *******************************************************************************/

#ifndef BSP_CAN_H_
#define BSP_CAN_H_

#include <stdint.h>
#include "driverlib.h"

/* ============================================================================
 * 协议地址与命令
 * ============================================================================ */
#define WATCHID                 (0xFF)  /* 上位机 / 广播心跳 ID */
#define SCOPEID                 (0x1F)  /* 软件示波器波形数据专用报文 ID */
#define AVERID                  (0x10)  /* 并联均流 ID */

#define BORAD_ALL               0       /* 全网全局广播 */
#define BORAD_TO                1       /* 定向单播 */

#define ADDRcmd_end             1       /* ID.ADDRcmd=1：普通报文（本机发帧恒用） */
#define ADDRcmd_on              0       /* ID.ADDRcmd=0：历史线侧标志；应用层待分址态见 bCalIDCmd */

#define CAN_TELEMETRY_CMD       0x11
#define CAN_TELEMETRY_MAX_CH    0x2C    /* 0x00 ~ 0x2C；0x2B/0x2C 为环路参考和前馈 */

/* 软件版本/构建信息（0x21/0x22 遥测通道上报） */
#define SOFTER_TIME             (261120)
#define SOFTER_VER              (100)

/* ============================================================================
 * 29 位 ID 拼装（发送用）
 * ============================================================================ */
#define CAN_ID_MAKE(dest, src, ptp, cmd)                                  \
    ( ((uint32_t)(dest)  & 0x0007FFFFUL)                                  \
    | (((uint32_t)(src)  & 0x000000FFUL) << 19)                           \
    | (((uint32_t)(ptp)  & 0x00000001UL) << 27)                           \
    | (((uint32_t)(cmd)  & 0x00000001UL) << 28) )

/* ============================================================================
 * 大端数据场读写（CAN 线序：高字节在前）
 * ============================================================================ */
#define CAN_DAT_BE16(d, i) \
    ((Uint16)((((Uint16)(d)[(i)]) << 8) | ((Uint16)(d)[(i) + 1] & 0xFFu)))

#define CAN_DAT_PUT_BE16(d, i, v) \
    do { \
        (d)[(i)]     = (Uint16)(((v) >> 8) & 0xFFU); \
        (d)[(i) + 1] = (Uint16)((v) & 0xFFU);        \
    } while(0)

/* ============================================================================
 * 浮点/整型/字节流复用转换
 * ============================================================================ */
typedef union
{
    uint32_t f_32;
    float    all;
    struct
    {
        uint16_t lo;
        uint16_t hi;
    } u16;
    struct
    {
        int16_t lo;
        int16_t hi;
    } i16;
} CHANGTO_FLOAT;

/* ============================================================================
 * 29 位扩展标识符结构（收发解析用）
 * ============================================================================ */
typedef struct
{
    Uint32 ADDR:19;                     /* Bit 0~18 : 目的地址 */
    Uint32 LogADDR:8;                   /* Bit 19~26: 源逻辑地址 */
    Uint32 PTP:1;                       /* Bit 27   : 0 广播 1 点对点 */
    Uint32 ADDRcmd:1;                   /* Bit 28   : 线侧标志（本机发帧=1） */
    Uint32 res:3;                       /* Bit 29~31: 预留 */
} CANMSGID_BITS;

typedef union
{
    Uint32         all;
    CANMSGID_BITS  bit;
} CANMSGID_REG;

/* ============================================================================
 * 8 字节数据场（按 16 位字镜像 eCAN 的 LAM/MDL/MDH 访问方式）
 * ============================================================================ */
typedef struct
{
    Uint32 WORD0;
    Uint32 WORD1;
    Uint32 WORD2;
    Uint32 WORD3;
} CANDATAWORD;

typedef union
{
    CANDATAWORD   CANWord;
    Uint16        Dat[8];
} CANDATA_REG;

typedef struct
{
    CANMSGID_REG CanId;
    CANDATA_REG  CanData;
} DATACanType;

/* ============================================================================
 * 全局对象
 * ============================================================================ */
#define CAN_TX_BUF_CMD          (0U)
#define CAN_TX_BUF_WATCH        (1U)
#define CAN_TX_BUF_SCOPE        (2U)
#define CAN_RX_FIFO_WATCH       MCAN_RX_FIFO_NUM_0
#define CAN_RX_FIFO_AVER        MCAN_RX_FIFO_NUM_1
#define CAN_RX_FIFO_DEPTH       (6U)   /* ★ 两个接收 FIFO 各可积压 6 帧 */

/* ============================================================================
 * 接口
 * ============================================================================ */
/* ★ MCAN 驱动接口，协议调度由 App_CanComm 执行。
 *   init_Drv_can：上电同步推完 if/基数状态机；运行恢复用 Reinit+InitPoll。 */
void init_Drv_can(void);
Uint16 DabCan_TxReady(Uint16 u16BufIdx);
/* ★ Send 返回 1 表示已提交；发送成功由 TXBTO 确认。 */
Uint16 DabCan_Send(Uint16 u16BufIdx, uint32_t u32Id, const Uint16 *pu16Data);
Uint16 DabCan_TxSucceeded(Uint16 u16BufIdx);
void DabCan_CancelTx(Uint16 u16BufIdx);
/* ★ 每次最多出队一帧，无效帧也占用接收预算。 */
typedef enum
{
    DAB_CAN_RX_EMPTY = 0,
    DAB_CAN_RX_VALID,
    DAB_CAN_RX_INVALID
} DAB_CAN_RX_RESULT;

DAB_CAN_RX_RESULT DabCan_Recv(uint32_t u32FifoNum, DATACanType *pMsg);
void DabCan_CheckState(void);
void DabCan_Reinit(void);           /* 复位 MCANA + 启动 if/基数状态机，不阻塞 */
void DabCan_InitPoll(void);         /* 推进一拍初始化；IDLE 时为空操作 */
Uint16 DabCan_InitBusy(void);       /* 1=状态机未结束 */
extern Uint16 g_u16DabCanInitError;
extern volatile uint32_t g_u32DabCanUnexpectedInitCount;
extern volatile uint32_t g_u32DabCanRestrictedAsmCount; /* ★ CCCR.ASM 进入次数 */
/* ★ 下标对应 FIFO0/1，同次轮询前的多帧溢出合计为一次事件。 */
extern volatile uint32_t g_au32DabCanRxLostEvents[2];
                                /* 底层发送：u16BufIdx=0/1/2 对应 eCAN 邮箱 11/12/13，
                                   该缓冲上帧未发完则丢弃本帧 */

#endif /* BSP_CAN_H_ */
