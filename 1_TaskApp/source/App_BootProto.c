/*********************************************************************************
* File:        App_BootProto.c
* Description: ARM 升级协议解析（原始工程 SCIB 内部通讯协议的 boot 部分）
*
* 与 p550 UartProto（AA55 协议）共存，两套协议并行解析同一个 SCIB 字节流：
*   - 字节源：CustomUart RX 镜像环（CustomUart_AttachRxMirror 挂接，字节复制一份），
*     不影响主 rxRing 与 UartProto —— UartProto 及其 0x55AA 应用层升级路径零改动；
*   - 帧格式（原始工程 Task_InterComm）：
*       [0]=0xD2 从机地址 / [1]=0x01 功能码 / 定长 96 字节 / 无 CRC；
*       字节间隔 >3.5ms 判为断帧（原 u16InterComRXIntervalCnt>INTER_COM_TIME_3MS）；
*   - 帧[9] 在线烧录命令（原始 DealInterReceiveData 分支）：
*       3(S_DSP_UPDATE) → UpdateFlag=1, FlashReady=1（允许跳 Boot）；
*       其它非 0        → UpdateFlag=1, FlashReady=0（升级中，封波）；
*       0               → 全 0（正常，取消升级）；
*   - 跳转（原始 Task_Status.PVBoostFlashMode）：
*       FlashReady 持续 2s（v102：SuperTask 10ms 节拍内数 200×10ms=2s）→ 封波 → FromAppRequestFlag=0x55AA
*       → asm MOV SP,#0 → asm LB 0x080000（直接跳 Boot 入口，不返回）；
*   - 升级取消（帧[9]=0）后不自动恢复 PWM：升级链路结束后由整机重启/状态机恢复，
*     避免升级过程半途开波。
***********************************************************************************/

#include "Includes.h"

/* FromAppRequestFlag 由 cmd 定址在 0x128（28p55x_cla_flash_lnk.cmd），Boot 阶段读取 */
extern uint16_t FromAppRequestFlag;

/*------------------------------------------------------------------------------
* 私有定义
*------------------------------------------------------------------------------*/
#define BOOT_PROTO_RX_RING_SIZE     (256U)                      /* 镜像环容量（2 的幂） */
#define BOOT_PROTO_RX_RING_MASK     (BOOT_PROTO_RX_RING_SIZE - 1U)

#define BOOT_ST_WAIT_ADDR           (0U)                        /* 等帧头地址 */
#define BOOT_ST_WAIT_FUNC           (1U)                        /* 等功能码 */
#define BOOT_ST_BODY                (2U)                        /* 收帧体（定长） */

static volatile uint16_t sg_BootRxRing[BOOT_PROTO_RX_RING_SIZE];   /* 镜像环（CustomUart 写 head，本模块读 tail） */
static volatile uint16_t sg_u16BootRxHead;
static volatile uint16_t sg_u16BootRxTail;

BootProtoCtrl_t g_BootProto;

/*------------------------------------------------------------------------------
* FUNCION :  BootProto_Init()
* PURPOSE :  清状态机/标志，并把本模块镜像环挂接到 CustomUart RX 通道
*------------------------------------------------------------------------------*/
void BootProto_Init(void)
{
    memset(&g_BootProto, 0, sizeof(g_BootProto));

    sg_u16BootRxHead = 0U;
    sg_u16BootRxTail = 0U;

    /* SCIB 字节流镜像挂接：CustomUart_Task_Rx 每收 1 字节同步复制一份到这里 */
    CustomUart_AttachRxMirror(sg_BootRxRing, BOOT_PROTO_RX_RING_MASK,
                              &sg_u16BootRxHead, &sg_u16BootRxTail);
}

/*------------------------------------------------------------------------------
* FUNCION :  BootProto_Task_Rx()
* PURPOSE :  镜像环 → ARM 帧状态机：0xD2 → 0x01 → 定长 96 字节收集 → 解析帧[9]
*            字节间隔 >4ms 断帧复位（原始 ISR 轮询逻辑等价移植；1ms 任务轮询）
*------------------------------------------------------------------------------*/
void BootProto_Task_Rx(void)
{
    uint16_t byte;
    uint16_t next;
    uint16_t cmd;

    /* 字节间隔计时（1ms 拍）：>4ms 无新字节 → 半帧作废，回到等帧头 */
    g_BootProto.u16SilenceCnt++;
    if (g_BootProto.u16SilenceCnt > BOOT_PROTO_SILENCE_TICKS)
    {
        if (g_BootProto.u16State != BOOT_ST_WAIT_ADDR)
        {
            g_BootProto.u16State  = BOOT_ST_WAIT_ADDR;
            g_BootProto.u16ByteIdx = 0U;
        }
    }

    while (sg_u16BootRxTail != sg_u16BootRxHead)
    {
        byte = sg_BootRxRing[sg_u16BootRxTail];
        sg_u16BootRxTail = (uint16_t)((sg_u16BootRxTail + 1U) & BOOT_PROTO_RX_RING_MASK);
        g_BootProto.u16SilenceCnt = 0U;   /* 有新字节：刷新静默计时 */

        switch (g_BootProto.u16State)
        {
            case BOOT_ST_WAIT_ADDR:                     /* 等从机地址 0xD2 */
                if (byte == BOOT_PROTO_SLAVE_ADDR)
                {
                    g_BootProto.u8Frame[0] = (uint8_t)byte;
                    g_BootProto.u16ByteIdx = 1U;
                    g_BootProto.u16State  = BOOT_ST_WAIT_FUNC;
                }
                break;

            case BOOT_ST_WAIT_FUNC:                     /* 等功能码 0x01 */
                if (byte == BOOT_PROTO_FUNC_CODE)
                {
                    g_BootProto.u8Frame[g_BootProto.u16ByteIdx] = (uint8_t)byte;
                    g_BootProto.u16ByteIdx++;
                    g_BootProto.u16State = BOOT_ST_BODY;
                }
                else
                {
                    /* 功能码不符：回到等地址；若该字节恰为 0xD2 则直接续接（原始同款处理） */
                    g_BootProto.u16ByteIdx = 0U;
                    if (byte == BOOT_PROTO_SLAVE_ADDR)
                    {
                        g_BootProto.u8Frame[0] = (uint8_t)byte;
                        g_BootProto.u16ByteIdx = 1U;
                        g_BootProto.u16State  = BOOT_ST_WAIT_FUNC;
                    }
                    else
                    {
                        g_BootProto.u16State = BOOT_ST_WAIT_ADDR;
                    }
                }
                break;

            case BOOT_ST_BODY:                          /* 收帧体至 96 字节 */
            default:
                g_BootProto.u8Frame[g_BootProto.u16ByteIdx] = (uint8_t)byte;
                g_BootProto.u16ByteIdx++;

                if (g_BootProto.u16ByteIdx >= BOOT_PROTO_FRAME_LEN)
                {
                    g_BootProto.u16FrameCnt++;

                    /* 帧[9] 在线烧录命令（原始 DealInterReceiveData 分支）。
                     * 原始协议无 CRC；与 AA55 协议共存时镜像流可能随机拼出假帧，
                     * 要求帧[9] 连续两帧一致才更新标志（对 ARM 周期广播仅延迟一帧）。 */
                    cmd = (uint16_t)g_BootProto.u8Frame[BOOT_PROTO_CMD_FLASH_OFFS];

                    if (cmd == g_BootProto.u16LastCmd)
                    {
                        g_BootProto.u16CmdRepeat++;
                    }
                    else
                    {
                        g_BootProto.u16LastCmd  = cmd;
                        g_BootProto.u16CmdRepeat = 1U;
                    }

                    if (g_BootProto.u16CmdRepeat >= 2U)
                    {
                        if (cmd == BOOT_PROTO_CMD_UPDATE)
                        {
                            g_BootProto.u16FlashUpdateFlag = 1U;
                            g_BootProto.u16FlashReady      = 1U;
                            g_BootProto.u16UpdateCmdCnt++;
                        }
                        else if (cmd != 0U)
                        {
                            g_BootProto.u16FlashUpdateFlag = 1U;
                            g_BootProto.u16FlashReady      = 0U;
                        }
                        else
                        {
                            g_BootProto.u16FlashUpdateFlag = 0U;
                            g_BootProto.u16FlashReady      = 0U;
                        }
                    }

                    g_BootProto.u16State  = BOOT_ST_WAIT_ADDR;
                    g_BootProto.u16ByteIdx = 0U;
                }
                break;
        }
    }
}

/*------------------------------------------------------------------------------
* FUNCION :  BootProto_Task_1ms()
* PURPOSE :  1) UpdateFlag 置位边沿 → 立即封波（原始由 PV 状态机进入 cPV_FlashStatus 完成）
*            2) FlashReady 持续 2s 确认（对齐 v102：200×10ms）→ 封波兜底 → 0x55AA + MOV SP,#0 + LB 0x080000 跳 Boot
*------------------------------------------------------------------------------*/
void BootProto_Task_1ms(void)
{
    /* 进入升级态边沿：立即封波（AQCSFRC 强制清输出，持续有效，不必重复调用） */
    if ((g_BootProto.u16FlashUpdateFlag != 0U) && (g_BootProto.u16PwmDisabled == 0U))
    {
        DAB_PWM_Stop();
        g_BootProto.u16PwmDisabled = 1U;
    }

    if (g_BootProto.u16FlashReady != 0U)
    {
        g_BootProto.u16ReadyCnt++;

        if (g_BootProto.u16ReadyCnt > BOOT_PROTO_READY_TICKS_1MS)
        {
            DAB_PWM_Stop();     /* 跳转前兜底封波 */

            OS_ENTER_CRITICAL();

            FromAppRequestFlag = 0x55AA;    /* cmd 符号 0x128：Boot 判定来自 App 的升级请求 */

            asm("    MOV SP,#0");
            asm("    LB 0x080000");         /* 直接跳 Boot 入口，不返回（原始 PVBoostFlashMode） */
        }
    }
    else
    {
        g_BootProto.u16ReadyCnt = 0U;       /* Ready 撤销：确认计数清零 */
    }
}
