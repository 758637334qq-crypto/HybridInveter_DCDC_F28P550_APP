/* ConfigChip_CustomUart.c — SCIB 115200-8-N-1 字节通道（轮询，不用中断）
 * 资源表：GPIO14=SCIB_TX / GPIO15=SCIB_RX（ARM 通讯）
 * Write() → TX 环 → 1ms 搬进 TX FIFO；100us 把 RX FIFO 搬进 RX 环 */

#include "Includes.h"

#define UART_HW_BASE    (SCIB_BASE)   /* ARM 通讯：SCIB */

CustomUartCtrl_t g_CustomUart;

/* 上电调一次：SWRESET 期间配 8-N-1 / 波特率 / 增强 FIFO */
void CustomUart_Init(void)
{
    uint16_t baudDiv;

    memset(&g_CustomUart, 0, sizeof(g_CustomUart));

    baudDiv = (uint16_t)((DEVICE_LSPCLK_FREQ / (115200UL * 8UL)) - 1UL);   /* BRR = LSPCLK / (115200 × 8) − 1 */

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

    /* 0xE040：开增强 FIFO、释放 TX 复位、清中断标志；TX 中断关闭 */
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

/* 挂接 RX 镜像环：ring/mask/head/tail 由调用方（App_BootProto）提供 */
void CustomUart_AttachRxMirror(volatile uint16_t *ring, uint16_t mask,
                               volatile uint16_t *head, volatile uint16_t *tail)
{
    g_CustomUart.pRxMirrorRing = ring;
    g_CustomUart.u16RxMirrorMask = mask;
    g_CustomUart.pRxMirrorHead = head;
    g_CustomUart.pRxMirrorTail = tail;
}

/* 整包写入 TX 环；空间不足则一个字节都不写（原子性） */
bool CustomUart_Write(const uint16_t *data, uint16_t len)
{
    uint16_t free_space;
    uint16_t head;
    uint16_t tail;
    uint16_t i;
    bool ok = false;

    if ((len > 0U) && (data != 0))
    {
        head = g_CustomUart.txHead;
        tail = g_CustomUart.txTail;

        /* 剩余空间 = (tail − head − 1) & MASK；留一格区分满/空 */
        free_space = (uint16_t)((tail - head - 1U) & CUSTOM_UART_TX_RING_MASK);

        if (free_space >= len)
        {
            for (i = 0U; i < len; i++)
            {
                g_CustomUart.txRing[head] = data[i] & 0xFFU;   /* UART 只发低 8 位 */

                head = (uint16_t)((head + 1U) & CUSTOM_UART_TX_RING_MASK);   /* & MASK 走到末尾绕回 */
            }

            /* 全部写完才更新 head，1ms 任务此时才看得见 */
            g_CustomUart.txHead = head;
            ok = true;
        }
    }
    return ok;
}

/* 1ms 轮询：RX FIFO → RX 环；出错先复位 SCI 再收（CAN/SCI 接收统一放 1ms 任务） */
void CustomUart_Task_Rx(void)
{
    uint16_t rx_char;
    uint16_t next;
    uint16_t mNext;
    uint16_t rxStatus;
    bool needRecover;

    /* 硬件溢出/错误标志不清会粘住，本拍先清干净 */
    needRecover = false;
    if (SCI_getOverflowStatus(UART_HW_BASE) != false)
    {
        g_CustomUart.u16RxHwOvfCnt++;
        needRecover = true;
    }
    rxStatus = SCI_getRxStatus(UART_HW_BASE);
    if ((rxStatus & SCI_RXSTATUS_ERROR) != 0U)
    {
        g_CustomUart.u16RxHwErrCnt++;
        needRecover = true;
    }
    if (needRecover != false)
    {
        /* 注意：SWRESET 会同时复位 SCI 发送状态机，正在移位输出的字节
         * 可能被中止（对端看到被撕坏的帧，靠 CRC 兜底重发）；
         * RX 侧按设计丢弃整段 FIFO（含错误字节前已收到的字节），
         * 交由上位机超时重发 */
        while (SCI_getRxFIFOStatus(UART_HW_BASE) != SCI_FIFO_RX0)
        {
            (void)SCI_readCharNonBlocking(UART_HW_BASE);
        }
        SCI_clearOverflowStatus(UART_HW_BASE);
        SCI_performSoftwareReset(UART_HW_BASE);
        SCI_resetRxFIFO(UART_HW_BASE);
        return;   /* 本拍丢弃污染数据 */
    }

    /* 115200bps 下 16 级 FIFO 约 1.39ms 填满，1ms 轮询含调度抖动余量约 0.4ms 内不会溢出；
     * 万一溢出由 u16RxHwOvfCnt / u16RxDropCnt 计数暴露 */
    while (SCI_getRxFIFOStatus(UART_HW_BASE) != SCI_FIFO_RX0)
    {
        rx_char = (uint16_t)(SCI_readCharNonBlocking(UART_HW_BASE) & 0xFFU);
        next = (uint16_t)((g_CustomUart.rxHead + 1U) & CUSTOM_UART_RX_RING_MASK);

        if (next != g_CustomUart.rxTail)   /* 软件环留一格区分满/空 */
        {
            g_CustomUart.rxRing[g_CustomUart.rxHead] = rx_char;
            g_CustomUart.rxHead = next;
        }
        else
        {
            g_CustomUart.u16RxDropCnt++;   /* 环满仍读走硬件字节，避免 FIFO 再溢 */
        }

        /* 镜像环（可选）：同一字节流复制给 boot 协议（App_BootProto），不影响主环 */
        if (g_CustomUart.pRxMirrorRing != 0)
        {
            mNext = (uint16_t)((*g_CustomUart.pRxMirrorHead + 1U) & g_CustomUart.u16RxMirrorMask);

            if (mNext != *g_CustomUart.pRxMirrorTail)
            {
                g_CustomUart.pRxMirrorRing[*g_CustomUart.pRxMirrorHead] = rx_char;
                *g_CustomUart.pRxMirrorHead = mNext;
            }
            else
            {
                g_CustomUart.u16RxMirrorDropCnt++;   /* 镜像环满丢弃，由协议侧断帧恢复 */
            }
        }
    }
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

        tail = g_CustomUart.txTail;

        while ((hw_free > 0U) && (g_CustomUart.txHead != tail))
        {
            SCI_writeCharNonBlocking(UART_HW_BASE, g_CustomUart.txRing[tail]);

            tail = (uint16_t)((tail + 1U) & CUSTOM_UART_TX_RING_MASK);

            hw_free--;
        }

        g_CustomUart.txTail = tail;   /* 搬完一次性释放空间给 Write() */
    }
}

/* TX 空闲判定：软件环已搬空且硬件 FIFO 已空。
 * 用途：复位进 Boot 前确认 ACK（'Y'）已上线——提交点的 TX 完成侧判据。
 * 注意只能看到 FIFO 级，最后一个字节可能仍在移位寄存器（≤1 字节时间 ~87us），
 * 调用侧需留出该裕量 */
bool CustomUart_TxIdle(void)
{
    return ((g_CustomUart.txHead == g_CustomUart.txTail) &&
            (SCI_getTxFIFOStatus(UART_HW_BASE) == SCI_FIFO_TX0));
}
