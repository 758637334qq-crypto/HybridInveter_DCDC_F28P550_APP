/* ConfigChip_CustomUart.c — SCIB 115200-8-N-1 字节通道（轮询，不用中断）
 * 资源表：GPIO14=SCIB_TX / GPIO15=SCIB_RX（ARM 通讯）
 * TX：Write() → TX 环 → 1ms 搬进 TX FIFO；
 * RX：无软件环，任务层经 RxService/PopRxByte 从硬件 FIFO 直递 */

#include "Includes.h"

#define UART_HW_BASE    (SCIB_BASE)   /* ARM 通讯：SCIB */

ST_CUSTOM_UART_CTRL stCustomUart;

/* 上电调一次：SWRESET 期间配 8-N-1 / 波特率 / 增强 FIFO */
void CustomUart_Init(void)
{
    uint16_t baudDiv;

    memset(&stCustomUart, 0, sizeof(stCustomUart));

    /* BRR 四舍五入：(LSPCLK + baud×4) / (baud×8) − 1；37.5MHz/115200 → 40（-0.76%，原截断 39 为 +1.73%） */
    baudDiv = (uint16_t)(((DEVICE_LSPCLK_FREQ + (115200UL * 4UL)) / (115200UL * 8UL)) - 1UL);

    ScibRegs.SCICTL1.bit.SWRESET = 0;   /* 配置期间保持复位，避免误收发 */

    /* 8-N-1：8 数据位、1 停止位、无校验；空闲线模式、关自环 */
    ScibRegs.SCICCR.bit.SCICHAR       = 7;
    ScibRegs.SCICCR.bit.ADDRIDLE_MODE = 0;
    ScibRegs.SCICCR.bit.LOOPBKENA     = 0;
    ScibRegs.SCICCR.bit.STOPBITS      = 0;
    ScibRegs.SCICCR.bit.PARITYENA     = 0;
    ScibRegs.SCICCR.bit.PARITY        = 0;

    /* 使能收发，关闭 SLEEP/TXWAKE 多机唤醒 */
    ScibRegs.SCICTL1.bit.RXENA  = 1;
    ScibRegs.SCICTL1.bit.TXENA  = 1;
    ScibRegs.SCICTL1.bit.SLEEP  = 0;
    ScibRegs.SCICTL1.bit.TXWAKE = 0;

    /* BRR 拆成高 8 位 / 低 8 位写入 */
    ScibRegs.SCIHBAUD.bit.BAUD = (baudDiv >> 8) & 0xFFU;
    ScibRegs.SCILBAUD.bit.BAUD = baudDiv & 0xFFU;

    /* TX FIFO 0→1 复位脉冲（0xC040 复位 FIFO，再写 0xE040 释放并清标志），清软跳残留字节 */
    ScibRegs.SCIFFTX.all = 0xC040U;
    ScibRegs.SCIFFTX.all = 0xE040U;

    /* RX：0→1 复位脉冲，清溢出/中断标志，关接收中断（轮询） */
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
    ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    ScibRegs.SCIFFRX.bit.RXFFIENA = 0;
    ScibRegs.SCIFFRX.bit.RXFFIL = 0;

    ScibRegs.SCIFFCT.all = 0x0000U;   /* 关自动波特率检测 */

    ScibRegs.SCICTL1.bit.SWRESET = 1;   /* 退出复位，SCI 开始工作 */
}

/* 整包写入 TX 环；空间不足则一个字节都不写（原子性） */
bool CustomUart_Write(const uint16_t *data, uint16_t len)
{
    uint16_t free_space;
    uint16_t head;
    uint16_t tail;
    uint16_t i;

    if ((len == 0U) || (data == 0))
    {
        return false;
    }

    head = stCustomUart.u16TxHead;
    tail = stCustomUart.u16TxTail;

    /* 剩余空间 = (tail − head − 1) & MASK；留一格区分满/空 */
    free_space = (uint16_t)((tail - head - 1U) & CUSTOM_UART_TX_RING_MASK);
    if (free_space < len)
    {
        return false;
    }

    for (i = 0U; i < len; i++)
    {
        stCustomUart.u16TxRing[head] = data[i] & 0xFFU;   /* UART 只发低 8 位 */
        head = (uint16_t)((head + 1U) & CUSTOM_UART_TX_RING_MASK);
    }

    /* 全部写完才更新 head，1ms 任务此时才看得见 */
    stCustomUart.u16TxHead = head;
    return true;
}

/* 1ms 拍首调（UartProto_RxTask 内）：RX 硬件溢出/错误标志检查与恢复。
 * 出错按设计丢弃整段 FIFO（含错误字节前已收到的字节），交由上位机超时重发。
 * 返回 true = 本拍发生过错误恢复，调用方应立即作废正在接收中的半帧。
 * 仅 SCIRXST.RXERROR 置位时才做 SWRESET（仅 RXFFOVF 溢出时只复位 RX FIFO，避免无谓撕裂 TX）；
 * 溢出标志清零放在 SCI_resetRxFIFO 之后，消除清标志到复位间的竞态窗口。 */
bool CustomUart_RxService(void)
{
    bool hasOvf;
    bool hasErr;

    /* 硬件溢出/错误标志不清会粘住，本拍先清干净 */
    hasOvf = SCI_getOverflowStatus(UART_HW_BASE);
    if (hasOvf != false)
    {
        stCustomUart.u16RxHwOvfCnt++;
    }

    hasErr = ((SCI_getRxStatus(UART_HW_BASE) & SCI_RXSTATUS_ERROR) != 0U);
    if (hasErr != false)
    {
        stCustomUart.u16RxHwErrCnt++;
        SCI_performSoftwareReset(UART_HW_BASE);
    }

    if ((hasOvf != false) || (hasErr != false))
    {
        SCI_resetRxFIFO(UART_HW_BASE);
        SCI_clearOverflowStatus(UART_HW_BASE);
        return true;
    }

    return false;
}

/* 从硬件 RX FIFO 取一个字节；false = FIFO 空（调用方 1ms 拍内循环直至空）。
 * 115200bps 下 16 级 FIFO 约 1.39ms 填满，1ms 轮询含调度抖动余量约 0.4ms 内不会溢出；
 * 万一溢出由 u16RxHwOvfCnt 计数暴露（RX 无软件环，FIFO 直递任务层） */
bool CustomUart_PopRxByte(uint16_t *b)
{
    if (SCI_getRxFIFOStatus(UART_HW_BASE) == SCI_FIFO_RX0)
    {
        return false;
    }
    *b = (uint16_t)(SCI_readCharNonBlocking(UART_HW_BASE) & 0xFFU);
    return true;
}

/* 1ms：TX 环 → TX FIFO → 串口线 */
void CustomUart_Task_1ms(void)
{
    uint16_t hw_used;
    uint16_t hw_free;
    uint16_t tail;

    hw_used = (uint16_t)SCI_getTxFIFOStatus(UART_HW_BASE);

    if (hw_used < 16U)   /* TX FIFO 共 16 级 */
    {
        hw_free = (uint16_t)(16U - hw_used);

        tail = stCustomUart.u16TxTail;

        while ((hw_free > 0U) && (stCustomUart.u16TxHead != tail))
        {
            SCI_writeCharNonBlocking(UART_HW_BASE, stCustomUart.u16TxRing[tail]);

            tail = (uint16_t)((tail + 1U) & CUSTOM_UART_TX_RING_MASK);

            hw_free--;
        }

        stCustomUart.u16TxTail = tail;   /* 搬完一次性释放空间给 Write() */
    }
}
