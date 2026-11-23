/* ConfigChip_CustomUart.h — SCI 字节通道的发送环形缓冲区与控制块
 * TX 环 32（容量须为 2 的幂）；RX 无软件环，任务层经 RxService/PopRxByte 直递硬件 FIFO */

#ifndef BSP_CUSTOMUART_H_
#define BSP_CUSTOMUART_H_

#include <stdint.h>
#include <stdbool.h>

/* ==================== 1. 常量 ==================== */

#define CUSTOM_UART_TX_RING_SIZE    (32U)   /* 可用 31 字节，留一格区分满/空 */
#define CUSTOM_UART_TX_RING_MASK    (CUSTOM_UART_TX_RING_SIZE - 1U)   /* 指针 & MASK 绕回 */

/* ==================== 2. 类型定义 ==================== */

/* volatile：head/tail/ring/计数跨任务共享，禁止编译器缓存 */
typedef struct
{
    /* TX：Write() 写 head，1ms 任务读 tail；C2000 最小寻址 16 位，只存低 8 位 */
    volatile uint16_t u16TxRing[CUSTOM_UART_TX_RING_SIZE];

    volatile uint16_t u16TxHead;

    volatile uint16_t u16TxTail;

    volatile uint16_t u16RxHwOvfCnt;   /* 硬件 RX FIFO 溢出次数 */
    volatile uint16_t u16RxHwErrCnt;   /* SCIRXST 错误次数 */

} ST_CUSTOM_UART_CTRL;

/* ==================== 3. 变量声明 ==================== */

extern ST_CUSTOM_UART_CTRL stCustomUart;

/* ==================== 4. 公共 API ==================== */

void CustomUart_Init(void);   /* 上电调一次 */

bool CustomUart_RxService(void);   /* RX 溢出/错误检查与恢复（1ms 拍首调，出错丢弃整段 FIFO）；返回 true=本拍已恢复，调用方须作废在收半帧 */

bool CustomUart_PopRxByte(uint16_t *b);   /* 从硬件 RX FIFO 取一个字节；false = 空（拍内循环） */

void CustomUart_Task_1ms(void);   /* TX 环 → TX FIFO */

bool CustomUart_Write(const uint16_t *data, uint16_t len);   /* 整包入 TX 环；false = 空间不足，一个字节都不写 */

#endif
