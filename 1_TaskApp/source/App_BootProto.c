/*********************************************************************************
* File:        App_BootProto.c
* Description: ARM 升级协议解析（原始工程 SCIB 内部通讯协议的 boot 部分）
*
* 与 p550 UartProto（AA55 协议）共存，两套协议并行解析同一个 SCIB 字节流：
*   - 字节源：任务层直递——UartProto_RxTask 从驱动 FIFO 逐字节取出后同步调
*     BootProto_RxByte（原镜像环方案已删，字节序/时机与镜像方案等价）；
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

#define BOOT_ST_WAIT_ADDR           (0U)                        /* 等帧头地址 */
#define BOOT_ST_WAIT_FUNC           (1U)                        /* 等功能码 */
#define BOOT_ST_BODY                (2U)                        /* 收帧体（定长） */

ST_BOOT_PROTO_CTRL stBootProto;

/*------------------------------------------------------------------------------
* FUNCION :  BootProto_Init()
* PURPOSE :  清状态机/标志（字节由 UartProto_RxTask 直递，无需挂接）
*------------------------------------------------------------------------------*/
void BootProto_Init(void)
{
    memset(&stBootProto, 0, sizeof(stBootProto));
}

/*------------------------------------------------------------------------------
* FUNCION :  BootProto_ResetRx()
* PURPOSE :  作废在收半帧，回到等帧头（RX 硬件错误恢复 / 静默断帧共用）
*------------------------------------------------------------------------------*/
void BootProto_ResetRx(void)
{
    stBootProto.u16State      = BOOT_ST_WAIT_ADDR;
    stBootProto.u16ByteIdx    = 0U;
    stBootProto.u16SilenceCnt = 0U;
}

/*------------------------------------------------------------------------------
* FUNCION :  BootProto_Task_Rx()
* PURPOSE :  字节间隔计时（1ms 拍）：>4ms 无新字节 → 半帧作废，回到等帧头。
*            必须在 UartProto_RxTask 喂字节之前调用（Task_1ms 顺序保证），
*            否则新帧首字节先刷新静默计数会把新帧拼进旧半帧。
*------------------------------------------------------------------------------*/
void BootProto_Task_Rx(void)
{
    if (stBootProto.u16State != BOOT_ST_WAIT_ADDR)
    {
        stBootProto.u16SilenceCnt++;
        if (stBootProto.u16SilenceCnt > BOOT_PROTO_SILENCE_TICKS)
        {
            BootProto_ResetRx();
        }
    }
}

/*------------------------------------------------------------------------------
* FUNCION :  BootProto_RxByte()
* PURPOSE :  ARM 帧状态机：0xD2 → 0x01 → 定长 96 字节收集 → 解析帧[9]
*            由 UartProto_RxTask 从驱动 FIFO 直递调用（同一字节流），
*            每字节刷新静默计时；断帧检查见 BootProto_Task_Rx
*------------------------------------------------------------------------------*/
void BootProto_RxByte(uint16_t byte)
{
    uint16_t cmd;

    stBootProto.u16SilenceCnt = 0U;   /* 有新字节：刷新静默计时 */

    switch (stBootProto.u16State)
    {
            case BOOT_ST_WAIT_ADDR:                     /* 等从机地址 0xD2 */
                if (byte == BOOT_PROTO_SLAVE_ADDR)
                {
                    stBootProto.u16ByteIdx = 1U;
                    stBootProto.u16State  = BOOT_ST_WAIT_FUNC;
                }
                break;

            case BOOT_ST_WAIT_FUNC:                     /* 等功能码 0x01 */
                if (byte == BOOT_PROTO_FUNC_CODE)
                {
                    stBootProto.u16ByteIdx++;
                    stBootProto.u16State = BOOT_ST_BODY;
                }
                else
                {
                    /* 功能码不符：回到等地址；若该字节恰为 0xD2 则直接续接（原始同款处理） */
                    stBootProto.u16ByteIdx = 0U;
                    if (byte == BOOT_PROTO_SLAVE_ADDR)
                    {
                        stBootProto.u16ByteIdx = 1U;
                        stBootProto.u16State  = BOOT_ST_WAIT_FUNC;
                    }
                    else
                    {
                        stBootProto.u16State = BOOT_ST_WAIT_ADDR;
                    }
                }
                break;

            case BOOT_ST_BODY:                          /* 收帧体至 96 字节 */
            default:
                /* 全帧只读取帧[9]：仅暂存该字节，其余只计数（原 u8Frame[96] 缓冲已删） */
                if (stBootProto.u16ByteIdx == BOOT_PROTO_CMD_FLASH_OFFS)
                {
                    stBootProto.u8FlashCmd = (uint8_t)byte;
                }
                stBootProto.u16ByteIdx++;

                if (stBootProto.u16ByteIdx >= BOOT_PROTO_FRAME_LEN)
                {
                    stBootProto.u16FrameCnt++;

                    /* 帧[9] 在线烧录命令（原始 DealInterReceiveData 分支）。
                     * 原始协议无 CRC；与 AA55 协议共存时镜像流可能随机拼出假帧，
                     * 要求帧[9] 连续两帧一致才更新标志（对 ARM 周期广播仅延迟一帧）。 */
                    cmd = (uint16_t)stBootProto.u8FlashCmd;

                    if (cmd == stBootProto.u16LastCmd)
                    {
                        if (stBootProto.u16CmdRepeat < 2U)
                        {
                            stBootProto.u16CmdRepeat++;   /* 饱和到 2，防止连续 65536 帧后 16 位回卷 */
                        }
                    }
                    else
                    {
                        stBootProto.u16LastCmd  = cmd;
                        stBootProto.u16CmdRepeat = 1U;
                    }

                    if (stBootProto.u16CmdRepeat >= 2U)
                    {
                        if (cmd == BOOT_PROTO_CMD_UPDATE)
                        {
                            stBootProto.u16FlashUpdateFlag = 1U;
                            stBootProto.u16FlashReady      = 1U;
                            stBootProto.u16UpdateCmdCnt++;
                        }
                        else if (cmd != 0U)
                        {
                            stBootProto.u16FlashUpdateFlag = 1U;
                            stBootProto.u16FlashReady      = 0U;
                        }
                        else
                        {
                            stBootProto.u16FlashUpdateFlag = 0U;
                            stBootProto.u16FlashReady      = 0U;
                        }
                    }

                    stBootProto.u16State  = BOOT_ST_WAIT_ADDR;
                    stBootProto.u16ByteIdx = 0U;
                }
                break;
    }
}

/*------------------------------------------------------------------------------
* FUNCION :  BootProto_Task_1ms()
* PURPOSE :  1) UpdateFlag 置位边沿 → 立即软强制兜底；持续封波三路互补：
*               ISR 见升级标志每拍 seal；SM_StopReq 纳入停机四触发后 OstStop 锁存；
*               非 RUN 后退回 INIT 后仍由 ISR ① 门封波（对齐 v102 cPV_FlashStatus）
*            2) FlashReady 持续 2s 确认（对齐 v102：200×10ms）→ 封波兜底 → 0x55AA + MOV SP,#0 + LB 0x080000 跳 Boot
*------------------------------------------------------------------------------*/
void BootProto_Task_1ms(void)
{
    /* 进入升级态边沿：立即软强制（AQCSFRC）；OST 前靠 ISR 每拍 seal 防发波口重写顶掉；
     * 持续硬封波仍由状态机停机路径 OstStop 锁存。
     * 升级态撤销后清 u16PwmDisabled，确保再次进升级态仍能 1ms 立即封波。 */
    if (stBootProto.u16FlashUpdateFlag != 0U)
    {
        if (stBootProto.u16PwmDisabled == 0U)
        {
            DAB_PWM_Stop();
            stBootProto.u16PwmDisabled = 1U;
        }
    }
    else
    {
        stBootProto.u16PwmDisabled = 0U;
    }

    if (stBootProto.u16FlashReady != 0U)
    {
        stBootProto.u16ReadyCnt++;

        if (stBootProto.u16ReadyCnt > BOOT_PROTO_READY_TICKS_1MS)
        {
            OS_ENTER_CRITICAL();

            SysCtl_serviceWatchdog();
            SysCtl_disableWatchdog();
            Board_PrepareForBoot();         /* OST+软强制封波 / 安全关 IO / PWM 脚切输入（保留 SCIB） */

            FromAppRequestFlag = 0x55AA;    /* cmd 符号 0x128：Boot 判定来自 App 的升级请求 */

            asm("    MOV SP,#0");
            asm("    LB 0x080000");         /* 直接跳 Boot 入口，不返回（原始 PVBoostFlashMode） */
        }
    }
    else
    {
        stBootProto.u16ReadyCnt = 0U;       /* Ready 撤销：确认计数清零 */
    }
}
