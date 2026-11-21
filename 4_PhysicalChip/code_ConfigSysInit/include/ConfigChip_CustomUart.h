/* ConfigChip_CustomUart.h — SCI 字节通道的发送环形缓冲区与控制块
 * TX 环 32（容量须为 2 的幂）；RX 无软件环，任务层经 RxService/PopRxByte 直递硬件 FIFO */

#ifndef BSP_CUSTOMUART_H_
#define BSP_CUSTOMUART_H_

#include <stdint.h>
#include <stdbool.h>

#define CUSTOM_UART_TX_RING_SIZE    (32U)   /* 可用 31 字节，留一格区分满/空 */
#define CUSTOM_UART_TX_RING_MASK    (CUSTOM_UART_TX_RING_SIZE - 1U)   /* 指针 & MASK 绕回 */

/* volatile：head/tail/ring/计数跨任务共享，禁止编译器缓存 */
typedef struct
{
    /* TX：Write() 写 head，1ms 任务读 tail；C2000 最小寻址 16 位，只存低 8 位 */
    volatile uint16_t txRing[CUSTOM_UART_TX_RING_SIZE];

    volatile uint16_t txHead;

    volatile uint16_t txTail;

    volatile uint16_t u16RxHwOvfCnt;   /* 硬件 RX FIFO 溢出次数 */
    volatile uint16_t u16RxHwErrCnt;   /* SCIRXST 错误次数 */

} CustomUartCtrl_t;

extern CustomUartCtrl_t g_CustomUart;

void CustomUart_Init(void);   /* 上电调一次 */

void CustomUart_RxService(void);   /* RX 溢出/错误检查与恢复（1ms 拍首调，出错丢弃整段 FIFO） */

bool CustomUart_PopRxByte(uint16_t *b);   /* 从硬件 RX FIFO 取一个字节；false = 空（拍内循环） */

void CustomUart_Task_1ms(void);   /* TX 环 → TX FIFO */

bool CustomUart_Write(const uint16_t *data, uint16_t len);   /* 整包入 TX 环；false = 空间不足，一个字节都不写 */

bool CustomUart_TxIdle(void);   /* TX 空闲：软件环已搬空且硬件 FIFO 已空 */

#endif
