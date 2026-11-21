/* F28P550 MCAN 驱动，经典 CAN 500 kbps。 */
#include "f28x_project.h"
#include "device.h"
#include "ConfigChip_McanInit.h"
#include <string.h>

#define DAB_CAN_BASE                MCANA_DRIVER_BASE

/* MCAN 时钟：SYSCLK 150 MHz / 6 = 25 MHz */
#define DAB_CAN_CLK_DIV             SYSCTL_MCANCLK_DIV_6

/* 500 kbps：25 MHz / (NBRP+1=5) / (1 + (NTSEG1+1=7) + (NTSEG2+1=2)) = 25/5/10 = 500 k */
#define DAB_CAN_NBRP                (4U)    /* 分频 = NBRP + 1 */
#define DAB_CAN_NTSEG1              (6U)    /* 段1 = 7 tq */
#define DAB_CAN_NTSEG2              (1U)    /* 段2 = 2 tq */
#define DAB_CAN_NSJW                DAB_CAN_NTSEG2  /* ★ SJW = 2 tq，与段2相同 */

/* 消息 RAM 分段（单位：字节偏移。扩展滤波器元素 8 B，收发元素 16 B） */
#define DAB_CAN_EXT_FILT_START      (0U)
#define DAB_CAN_EXT_FILT_NUM        (3U)    /* ★ 三条扩展滤波，每条占 8 B */
#define DAB_CAN_TX_BUF_START        (24U)
#define DAB_CAN_TX_BUF_NUM          (3U)
#define DAB_CAN_RX_FIFO0_START      (72U)   /* WATCHID / SCOPEID */
#define DAB_CAN_RX_FIFO1_START      (DAB_CAN_RX_FIFO0_START + CAN_RX_FIFO_DEPTH * 16U) /* ★ AVERID */
#define DAB_CAN_TX_EVT_START        (DAB_CAN_RX_FIFO1_START + CAN_RX_FIFO_DEPTH * 16U)
#define DAB_CAN_INIT_POLLS          (1000U)
#define DAB_CAN_ERR_UNEXPECTED_INIT (5U)
#define DAB_CAN_ERR_RESTRICTED_ASM  (6U)  /* ★ CCCR.ASM：能收不能发 */
/* DriverLib 把 MCAN_CCCR 定义为偏移宏，不能写 McanaRegs.MCAN_CCCR.bit.ASM */
#define DAB_CAN_CCCR_ASM_SET() \
    ((HWREG(DAB_CAN_BASE + MCAN_CCCR) & MCAN_CCCR_ASM_MASK) != 0UL)

/* ★ 0=正常，1=RAM，2=进入 INIT，3=配置，4=进入 Normal，
 *   5=运行中异常 INIT，6=受限模式 ASM。 */
Uint16 g_u16DabCanInitError = 1U;
volatile uint32_t g_u32DabCanUnexpectedInitCount;
volatile uint32_t g_u32DabCanRestrictedAsmCount;
volatile uint32_t g_au32DabCanRxLostEvents[2];

/* 发送缓冲编号（对应 eCAN 邮箱 11 / 12 / 13） */
Uint16 DabCan_TxReady(Uint16 u16BufIdx)
{
    MCAN_ProtocolStatus psr;

    if ((g_u16DabCanInitError != 0U) || (u16BufIdx >= DAB_CAN_TX_BUF_NUM) ||
        (MCAN_getOpMode(DAB_CAN_BASE) != MCAN_OPERATION_MODE_NORMAL) ||
        DAB_CAN_CCCR_ASM_SET())   /* ★ 受限模式禁止发送 */
    {
        return 0U;
    }
    MCAN_getProtocolStatus(DAB_CAN_BASE, &psr);
    if (psr.busOffStatus != 0U)
    {
        return 0U;
    }
    return ((MCAN_getTxBufReqPend(DAB_CAN_BASE) & (1UL << u16BufIdx)) == 0UL) ? 1U : 0U;
}

/**
 * @brief  MCAN 发送一帧经典扩展帧（8 字节）
 * @param  u16BufIdx 发送缓冲编号（DAB_CAN_TX_BUF_xxx，对应 eCAN 邮箱 11~13）
 * @note   该缓冲上一次发送未完成时直接丢弃本帧：与 eCAN 版"邮箱忙则跳过"策略一致。
 */
Uint16 DabCan_Send(Uint16 u16BufIdx, uint32_t u32Id, const Uint16 *pu16Data)
{
    MCAN_TxBufElement txMsg;
    Uint16 i;

    if ((pu16Data == 0) || (DabCan_TxReady(u16BufIdx) == 0U))
    {
        return 0U;
    }

    memset(&txMsg, 0, sizeof(txMsg));
    txMsg.id  = u32Id & 0x1FFFFFFFUL;
    txMsg.rtr = 0U;
    txMsg.xtd = 1U;                 /* 29 位扩展帧 */
    txMsg.esi = 0U;
    txMsg.dlc = 8U;                 /* 经典 CAN，有效 8 字节 */
    txMsg.brs = 0U;                 /* 不切换波特率（经典 CAN） */
    txMsg.fdf = 0U;                 /* 经典帧，非 CAN-FD */
    txMsg.efc = 0U;
    txMsg.mm  = 0xAAU;

    for (i = 0U; i < 8U; i++)
    {
        txMsg.data[i] = (Uint16)(pu16Data[i] & 0xFFU);
    }

    MCAN_writeMsgRam(DAB_CAN_BASE, MCAN_MEM_TYPE_BUF, (uint32_t)u16BufIdx, &txMsg);
    return MCAN_txBufAddReq(DAB_CAN_BASE, (uint32_t)u16BufIdx) == 0;
}

/* ★ 请求取消也会清除待发状态，成功只认 TXBTO。 */
Uint16 DabCan_TxSucceeded(Uint16 u16BufIdx)
{
    if ((g_u16DabCanInitError != 0U) || (u16BufIdx >= DAB_CAN_TX_BUF_NUM))
    {
        return 0U;
    }
    return (MCAN_getTxBufTransmissionStatus(DAB_CAN_BASE) & (1UL << u16BufIdx)) != 0UL;
}

void DabCan_CancelTx(Uint16 u16BufIdx)
{
    if ((g_u16DabCanInitError == 0U) && (u16BufIdx < DAB_CAN_TX_BUF_NUM) &&
        ((MCAN_getTxBufReqPend(DAB_CAN_BASE) & (1UL << u16BufIdx)) != 0UL))
    {
        (void)MCAN_txBufCancellationReq(DAB_CAN_BASE, (uint32_t)u16BufIdx);
    }
}

/**
 * @brief  从指定 RX FIFO 取一帧
 * @param  u32FifoNum MCAN_RX_FIFO_NUM_0（WATCHID/SCOPEID）/ MCAN_RX_FIFO_NUM_1（AVERID）
 * @return FIFO 无帧、取到有效帧或已丢弃一帧无效数据
 */
DAB_CAN_RX_RESULT DabCan_Recv(uint32_t u32FifoNum, DATACanType *pMsg)
{
    MCAN_RxFIFOStatus st;
    MCAN_RxBufElement rxMsg;
    Uint16 i;

    if ((g_u16DabCanInitError != 0U) || (pMsg == 0) || (u32FifoNum > MCAN_RX_FIFO_NUM_1))
    {
        return DAB_CAN_RX_EMPTY;
    }
    /* driverlib 靠 num 字段选择 RXF0S / RXF1S，必须先赋值 */
    memset(&st, 0, sizeof(st));
    st.num = u32FifoNum;
    MCAN_getRxFIFOStatus(DAB_CAN_BASE, &st);
    /* ★ 即使 FIFO 已空也结算溢出标志，只清当前 FIFO 的标志。 */
    if (st.msgLost != 0U)
    {
        MCAN_clearIntrStatus(DAB_CAN_BASE, (u32FifoNum == MCAN_RX_FIFO_NUM_0) ?
                             MCAN_INTR_SRC_RX_FIFO0_MSG_LOST : MCAN_INTR_SRC_RX_FIFO1_MSG_LOST);
        ++g_au32DabCanRxLostEvents[u32FifoNum];
    }
    /* ★ INIT 期间暂停访问 Message RAM，等待恢复。 */
    if ((st.fillLvl == 0U) || (MCAN_getOpMode(DAB_CAN_BASE) != MCAN_OPERATION_MODE_NORMAL))
    {
        return DAB_CAN_RX_EMPTY;
    }

    MCAN_readMsgRam(DAB_CAN_BASE, MCAN_MEM_TYPE_FIFO, 0U, u32FifoNum, &rxMsg);
    MCAN_writeRxFIFOAck(DAB_CAN_BASE, u32FifoNum, st.getIdx);

    if ((rxMsg.xtd == 0U) || (rxMsg.rtr != 0U) || (rxMsg.fdf != 0U) || (rxMsg.dlc != 8U))
    {
        return DAB_CAN_RX_INVALID;
    }

    pMsg->CanId.all = rxMsg.id & 0x1FFFFFFFUL;
    for (i = 0U; i < 8U; i++)
    {
        pMsg->CanData.Dat[i] = (Uint16)(rxMsg.data[i] & 0xFFU);
    }
    return DAB_CAN_RX_VALID;
}

/* ============================================================================
 * MCAN 硬件初始化（经典 CAN 500 kbps，双 RX FIFO + 3 个发送缓冲）
 * ============================================================================ */
/* ★ 启动与重试均使用有界轮询。 */
static Uint16 DabCan_WaitMode(uint32_t mode)
{
    Uint16 polls;

    for (polls = 0U; polls < DAB_CAN_INIT_POLLS; polls++)
    {
        if (MCAN_getOpMode(DAB_CAN_BASE) == mode)
        {
            return 1U;
        }
        DEVICE_DELAY_US(1U);
    }
    return 0U;
}

void init_Drv_can(void)
{
    MCAN_InitParams             initParams;
    MCAN_ConfigParams           configParams;
    MCAN_MsgRAMConfigParams     ramParams;
    MCAN_BitTimingParams        bitTimes;
    MCAN_ExtMsgIDFilterElement  extFilter[DAB_CAN_EXT_FILT_NUM];
    Uint16 polls = DAB_CAN_INIT_POLLS;

    g_u16DabCanInitError = 1U;

    memset(&initParams,   0, sizeof(initParams));
    memset(&configParams, 0, sizeof(configParams));
    memset(&ramParams,    0, sizeof(ramParams));
    memset(&bitTimes,     0, sizeof(bitTimes));
    memset(extFilter,     0, sizeof(extFilter));

    /* 1. MCAN 模块时钟 = SYSCLK / 6 = 25 MHz */
    SysCtl_setMCANClk(SYSCTL_MCANA, DAB_CAN_CLK_DIV);

    /* 2. 控制器模式：经典 CAN（非 FD），保留自动重传（与 eCAN 一致） */
    initParams.fdMode          = false;
    initParams.brsEnable       = false;
    initParams.txpEnable       = false;
    initParams.efbi            = false;
    initParams.pxhddisable     = false;
    initParams.darEnable       = false;
    initParams.wkupReqEnable   = false;
    initParams.autoWkupEnable  = false;
    initParams.emulationEnable = 1;
    initParams.tdcEnable       = false;
    initParams.wdcPreload      = 255U;

    /* 3. 全局过滤：不匹配的帧一律丢弃（等价 eCAN 掩码邮箱，避免挤占 RX FIFO） */
    configParams.monEnable         = 0U;
    configParams.asmEnable         = 0U;
    configParams.tsPrescalar       = 15U;
    configParams.tsSelect          = 0U;
    configParams.timeoutSelect     = 0U;
    configParams.timeoutPreload    = 0xFFFFU;
    configParams.timeoutCntEnable  = 0U;
    configParams.filterConfig.rrfe = 1U;    /* 拒绝远程扩展帧 */
    configParams.filterConfig.rrfs = 1U;    /* 拒绝远程标准帧 */
    configParams.filterConfig.anfe = 2U;    /* 10 = 不匹配的扩展帧丢弃 */
    configParams.filterConfig.anfs = 2U;    /* 10 = 不匹配的标准帧丢弃 */

    /* 4. 消息 RAM：双 RX FIFO + 3 个发送缓冲 */
    ramParams.flssa                = 0U;                        /* 无标准帧滤波器 */
    ramParams.lss                  = 0U;
    ramParams.flesa                = DAB_CAN_EXT_FILT_START;
    ramParams.lse                  = DAB_CAN_EXT_FILT_NUM;
    ramParams.txStartAddr          = DAB_CAN_TX_BUF_START;
    ramParams.txBufNum             = DAB_CAN_TX_BUF_NUM;        /* 3 个专用发送缓冲 */
    ramParams.txFIFOSize           = 0U;                        /* 不使用发送 FIFO */
    ramParams.txBufMode            = 0U;
    ramParams.txBufElemSize        = 0U;                        /* 8 字节元素 */
    ramParams.txEventFIFOStartAddr = DAB_CAN_TX_EVT_START;
    ramParams.txEventFIFOSize      = 0U;
    ramParams.txEventFIFOWaterMark = 0U;
    ramParams.rxFIFO0startAddr     = DAB_CAN_RX_FIFO0_START;    /* WATCHID/SCOPEID */
    ramParams.rxFIFO0size          = CAN_RX_FIFO_DEPTH;
    ramParams.rxFIFO0waterMark     = 0U;
    ramParams.rxFIFO0OpMode        = 0U;                        /* 阻塞模式 */
    ramParams.rxFIFO0ElemSize      = 0U;
    ramParams.rxFIFO1startAddr     = DAB_CAN_RX_FIFO1_START;    /* AVERID */
    ramParams.rxFIFO1size          = CAN_RX_FIFO_DEPTH;
    ramParams.rxFIFO1waterMark     = 0U;
    ramParams.rxFIFO1OpMode        = 0U;
    ramParams.rxFIFO1ElemSize      = 0U;
    ramParams.rxBufStartAddr       = 0U;
    ramParams.rxBufElemSize        = 0U;

    /* 5. 扩展滤波器：Classic filter，EFID2 = 0x000000FF 仅比较低 8 位地址 */
    extFilter[0].efid1 = (uint32_t)WATCHID;     /* 滤波器 0：WATCHID */
    extFilter[0].efec  = 0x1U;                  /* 001 = 存入 RX FIFO0 */
    extFilter[0].efid2 = 0x000000FFUL;          /* 掩码：仅低 8 位 */
    extFilter[0].eft   = 0x2U;                  /* 10 = Classic filter */

    extFilter[1].efid1 = (uint32_t)AVERID;      /* 滤波器 1：AVERID */
    extFilter[1].efec  = 0x2U;                  /* 010 = 存入 RX FIFO1 */
    extFilter[1].efid2 = 0x000000FFUL;
    extFilter[1].eft   = 0x2U;

    /* ★ SCOPEID 按扩展 ID 低 8 位匹配，存入 FIFO0。 */
    extFilter[2].efid1 = (uint32_t)SCOPEID;
    extFilter[2].efec  = 0x1U;
    extFilter[2].efid2 = 0x000000FFUL;
    extFilter[2].eft   = 0x2U;

    /* 6. 位定时：500 kbps，采样点 80% */
    bitTimes.nomRatePrescalar   = DAB_CAN_NBRP;
    bitTimes.nomTimeSeg1        = DAB_CAN_NTSEG1;
    bitTimes.nomTimeSeg2        = DAB_CAN_NTSEG2;
    bitTimes.nomSynchJumpWidth  = DAB_CAN_NSJW;
    bitTimes.dataRatePrescalar  = 0U;
    bitTimes.dataTimeSeg1       = 0U;
    bitTimes.dataTimeSeg2       = 0U;
    bitTimes.dataSynchJumpWidth = 0U;

    /* 7. 初始化流程 */
    while (0U == MCAN_isMemInitDone(DAB_CAN_BASE))
    {
        if (polls == 0U)
        {
            return;
        }
        polls--;
        DEVICE_DELAY_US(1U);
    }

    MCAN_setOpMode(DAB_CAN_BASE, MCAN_OPERATION_MODE_SW_INIT);
    if (DabCan_WaitMode(MCAN_OPERATION_MODE_SW_INIT) == 0U)
    {
        g_u16DabCanInitError = 2U;
        return;
    }

    if ((MCAN_init(DAB_CAN_BASE, &initParams) != 0) ||
        (MCAN_config(DAB_CAN_BASE, &configParams) != 0) ||
        (MCAN_setBitTime(DAB_CAN_BASE, &bitTimes) != 0) ||
        (MCAN_msgRAMConfig(DAB_CAN_BASE, &ramParams) != 0))
    {
        g_u16DabCanInitError = 3U;
        return;
    }
    MCAN_addExtMsgIDFilter(DAB_CAN_BASE, 0U, &extFilter[0]);    /* WATCHID → FIFO0 */
    MCAN_addExtMsgIDFilter(DAB_CAN_BASE, 1U, &extFilter[1]);    /* AVERID  → FIFO1 */
    MCAN_addExtMsgIDFilter(DAB_CAN_BASE, 2U, &extFilter[2]);    /* SCOPEID → FIFO0 */
    MCAN_setExtIDAndMask(DAB_CAN_BASE, 0x1FFFFFFFUL);

    MCAN_lpbkModeEnable(DAB_CAN_BASE, MCAN_LPBK_MODE_EXTERNAL, false);

    MCAN_setOpMode(DAB_CAN_BASE, MCAN_OPERATION_MODE_NORMAL);
    if (DabCan_WaitMode(MCAN_OPERATION_MODE_NORMAL) == 0U)
    {
        g_u16DabCanInitError = 4U;
        return;
    }
    g_u16DabCanInitError = 0U;
}

/* ★ 故障重试仅复位 MCANA，再重建 Message RAM 配置。 */
void DabCan_Reinit(void)
{
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_MCANA);
    init_Drv_can();
}

void DabCan_CheckState(void)
{
    MCAN_ProtocolStatus psr;

    if (g_u16DabCanInitError != 0U)
    {
        return;
    }

    /* ★ ASM 受限模式：INIT 仍为 0（看起来像 Normal），但停止发送、仍可接收。
     *   必须在“仅看 OpMode==Normal 就 return”之前检出，否则永久能收不能发。 */
    if (DAB_CAN_CCCR_ASM_SET())
    {
        g_u16DabCanInitError = DAB_CAN_ERR_RESTRICTED_ASM;
        ++g_u32DabCanRestrictedAsmCount;
        return;
    }

    if (MCAN_getOpMode(DAB_CAN_BASE) == MCAN_OPERATION_MODE_NORMAL)
    {
        return;
    }

    MCAN_getProtocolStatus(DAB_CAN_BASE, &psr);
    if ((psr.busOffStatus != 0U) &&
        ((MCAN_getIntrStatus(DAB_CAN_BASE) & MCAN_INTR_SRC_BIT_ERR_UNCORRECTED) == 0U))
    {
        /* ★ 先清 INIT，硬件随后检测总线空闲并恢复。 */
        MCAN_setOpMode(DAB_CAN_BASE, MCAN_OPERATION_MODE_NORMAL);
    }
    else
    {
        /* ★ 异常 INIT（含不可纠正位错误）交给周期任务重初始化。 */
        g_u16DabCanInitError = DAB_CAN_ERR_UNEXPECTED_INIT;
        ++g_u32DabCanUnexpectedInitCount;
    }
}

