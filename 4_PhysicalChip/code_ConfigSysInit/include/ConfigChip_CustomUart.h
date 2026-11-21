/* ConfigChip_CustomUart.h — SCI 字节通道的收发环形缓冲区与控制块
 * TX 32 / RX 64，容量须为 2 的幂（& MASK 等价于取模） */

#ifndef BSP_CUSTOMUART_H_
#define BSP_CUSTOMUART_H_

#include <stdint.h>
#include <stdbool.h>

#define CUSTOM_UART_TX_RING_SIZE    (32U)   /* 可用 31 字节，留一格区分满/空 */
#define CUSTOM_UART_TX_RING_MASK    (CUSTOM_UART_TX_RING_SIZE - 1U)   /* 指针 & MASK 绕回 */

#define CUSTOM_UART_RX_RING_SIZE    (64U)   /* 可用 63 字节 > 最大帧 39 */
#define CUSTOM_UART_RX_RING_MASK    (CUSTOM_UART_RX_RING_SIZE - 1U)

/* volatile：head/tail/ring/计数跨任务共享，禁止编译器缓存 */
typedef struct
{

    /* TX：Write() 写 head，1ms 任务读 tail；C2000 最小寻址 16 位，只存低 8 位 */
    volatile uint16_t txRing[CUSTOM_UART_TX_RING_SIZE];

    volatile uint16_t txHead;

    volatile uint16_t txTail;

    /* RX：100us 任务写 head，协议任务读 tail */
    volatile uint16_t rxRing[CUSTOM_UART_RX_RING_SIZE];

    volatile uint16_t rxHead;

    volatile uint16_t rxTail;

    volatile uint16_t u16RxHwOvfCnt;   /* 硬件 RX FIFO 溢出次数 */
    volatile uint16_t u16RxHwErrCnt;   /* SCIRXST 错误次数 */
    volatile uint16_t u16RxDropCnt;   /* 软件环满丢字节数 */

    /* RX 镜像（可选）：同一字节流复制一份给第二消费者（boot 协议，App_BootProto）。
     * ring/mask/head/tail 均由挂接方提供，pRxMirrorRing == NULL 表示未挂接（零开销）。
     * 镜像只读走当前字节流，不影响主 rxRing 与 UartProto —— 两套协议共存。 */
    volatile uint16_t *pRxMirrorRing;
    volatile uint16_t *pRxMirrorHead;
    volatile uint16_t *pRxMirrorTail;
    uint16_t u16RxMirrorMask;
    volatile uint16_t u16RxMirrorDropCnt;   /* 镜像环满丢字节数 */

} CustomUartCtrl_t;

extern CustomUartCtrl_t g_CustomUart;

void CustomUart_Init(void);   /* 上电调一次 */

void CustomUart_AttachRxMirror(volatile uint16_t *ring, uint16_t mask,
                               volatile uint16_t *head, volatile uint16_t *tail);   /* 挂接 RX 镜像环（应用层 Init 时调用一次） */

void CustomUart_Task_Rx(void);      /* RX FIFO → RX 环（1ms 任务轮询） */

void CustomUart_Task_1ms(void);   /* TX 环 → TX FIFO */

bool CustomUart_Write(const uint16_t *data, uint16_t len);   /* 整包入 TX 环；false = 空间不足，一个字节都不写 */

bool CustomUart_TxIdle(void);   /* TX 空闲：软件环已搬空且硬件 FIFO 已空 */

#endif
