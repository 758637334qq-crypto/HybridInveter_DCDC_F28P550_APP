/*********************************************************************************
* File:        Task_1ms.c
* Description: DAB 1ms 节拍片（滤波 / 1-Vbat / 缓起 / CAN 发送）
*              只做调度装配，具体功能在 Isr_/App_ 模块里。
***********************************************************************************/

#include "Includes.h"
#include "App_DabModel.h"   /* DAB_BAT_VOLT_MIN：电池电压合法区间 */

/**
 * @brief  1ms 任务体
 * @note   执行顺序：滤波 → 1/Vbat → 斜坡 → CAN/SCIB 接收 → CAN 发送 → SCI TX。
 *         CAN/SCI RX 有硬件 FIFO，1ms 轮询足够（已从 Task100us 迁出）；
 *         SCIB 拼帧/校验/命令在 Task_1ms（FIFO 直递当拍执行），遥测在 Task_5ms。
 *         逻辑地址由 UartProto 分址，非 CAN 握手。
 */
void Task1ms(void)
{
    /* 普通 PWM 无 SFO；宏占位保持调用点 */
    (void)DAB_Charge_SFO_Step();

    /* DAB 母线/电流/电池慢滤波（stADC 10/100/400ms 三档） */
    AdcDabFilt1ms();

    /* BatCurr 的分母倒数：用刚刷新的 10ms 滤波电池电压预算 1/Vbat，
     * 高频 ISR 里只做乘法（必须排在滤波之后）。 */
    Dab_Ctrl_InvBatVoltUpdate();

    /* V/±Ilim 缓起 + 均流下垂：仅闭环充放电态爬坡；均流每拍更新并发布 ISR 电压给定 */
    if (SM_IsClosedLoopActive() != 0U)
    {
        RunState.curLowSealing = Ctrl_RampStep1ms();
    }
    {
        const float f32RampV = Ramp.v;

        CurShare_Update1ms(DAB_VOLT_MIN - f32RampV, DAB_VOLT_MAX - f32RampV);
        g_CtrlBusRef_V = f32RampV + CurShare.voltAdj;
    }

    /* ---- 接收链路：1ms Rx（非 100us）；SCIB 顺序：Boot 静默检查 → UartProto 喂字节 ---- */
    CanComm_Task_Rx();       /* CAN 取帧 + 0x20/均流转发 */
    BootProto_Task_Rx();     /* ARM 升级协议：静默断帧检查（字节由 UartProto_RxTask 直递） */
    UartProto_RxTask();      /* SCIB FIFO 直递：3ms 断帧 → 校验 → 当拍执行；含 UART 分址 */

    CanComm_Task_Tx_1ms();   /* ★ 与接收统一：0x11/0x41/示波器；先 settle 再发 */
    CustomUart_Task_1ms();   /* TX 环 → TX FIFO → 串口线 */
    HeatFilm_Task();         /* 加热膜：跟 hotCmd；ISR 只跑软 PWM */
    BootProto_Task_1ms();    /* ARM 升级协议：升级态边沿封波 + FlashReady 2s 确认 → 跳 Boot */
}

/**
 * @brief  TaskSched 入口：统一 void(void *) 原型
 */
void Task1ms_Entry(void *arg)
{
    (void)arg;
    Task1ms();
}
