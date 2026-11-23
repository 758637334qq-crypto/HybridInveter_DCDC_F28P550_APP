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
#define DAB_CAN_EXT_FILT_NUM        (2U)    /* ★ WATCH+AVER；SCOPE 仅上行，不建滤波（anfe 丢弃） */
#define DAB_CAN_TX_BUF_START        (24U)   /* 仍按原 3×8 槽位起点，前 16 B 滤波、余 8 B 空闲 */
#define DAB_CAN_TX_BUF_NUM          (3U)
#define DAB_CAN_RX_FIFO0_START      (72U)   /* WATCHID */
#define DAB_CAN_RX_FIFO1_START      (DAB_CAN_RX_FIFO0_START + CAN_RX_FIFO_DEPTH * 16U) /* ★ AVERID */
#define DAB_CAN_TX_EVT_START        (DAB_CAN_RX_FIFO1_START + CAN_RX_FIFO_DEPTH * 16U)
#define DAB_CAN_BOOT_INIT_POLLS     (1000U) /* 上电同步等待：1us × 1000 ≈ 1ms/阶段 */
#define DAB_CAN_TASK_INIT_POLLS     (20U)   /* 5ms 运行态：5ms × 20 = 100ms/阶段 */
#define DAB_CAN_ERR_UNEXPECTED_INIT (5U)
#define DAB_CAN_ERR_RESTRICTED_ASM  (6U)  /* ★ CCCR.ASM：能收不能发 */
/* DriverLib 把 MCAN_CCCR 定义为偏移宏，不能写 McanaRegs.MCAN_CCCR.bit.ASM */
#define DAB_CAN_CCCR_ASM_SET() \
    ((HWREG(DAB_CAN_BASE + MCAN_CCCR) & MCAN_CCCR_ASM_MASK) != 0UL)
/* 直读 PSR.BO，避免 MCAN_ProtocolStatus（~22 字）压栈 */
#define DAB_CAN_PSR_BO_SET() \
    ((HWREG(DAB_CAN_BASE + MCAN_PSR) & MCAN_PSR_BO_MASK) != 0UL)

/* ★ 0=正常，1=RAM，2=进入 INIT，3=配置，4=进入 Normal，
 *   5=运行中异常 INIT，6=受限模式 ASM。 */
Uint16 g_u16DabCanInitError = 1U;
volatile uint32_t g_u32DabCanUnexpectedInitCount;
volatile uint32_t g_u32DabCanRestrictedAsmCount;
volatile uint32_t g_au32DabCanRxLostEvents[2];

/* 收发元素含 data[64]（CAN-FD 布局），放文件静态区，避免 1ms 任务栈峰值；
 * 任务串行调用且 ISR 不进 Send/Recv，无重入。 */
static MCAN_TxBufElement s_canTxMsg;
static MCAN_RxBufElement s_canRxMsg;

/* 发送缓冲编号（对应 eCAN 邮箱 11 / 12 / 13） */
Uint16 DabCan_TxReady(Uint16 u16BufIdx)
{
    if ((g_u16DabCanInitError != 0U) || (u16BufIdx >= DAB_CAN_TX_BUF_NUM) ||
        (MCAN_getOpMode(DAB_CAN_BASE) != MCAN_OPERATION_MODE_NORMAL) ||
        DAB_CAN_CCCR_ASM_SET())   /* ★ 受限模式禁止发送 */
    {
        return 0U;
    }
    if (DAB_CAN_PSR_BO_SET())
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
    Uint16 i;

    if ((pu16Data == 0) || (DabCan_TxReady(u16BufIdx) == 0U))
    {
        return 0U;
    }

    memset(&s_canTxMsg, 0, sizeof(s_canTxMsg));
    s_canTxMsg.id  = u32Id & 0x1FFFFFFFUL;
    s_canTxMsg.rtr = 0U;
    s_canTxMsg.xtd = 1U;                 /* 29 位扩展帧 */
    s_canTxMsg.esi = 0U;
    s_canTxMsg.dlc = 8U;                 /* 经典 CAN，有效 8 字节 */
    s_canTxMsg.brs = 0U;                 /* 不切换波特率（经典 CAN） */
    s_canTxMsg.fdf = 0U;                 /* 经典帧，非 CAN-FD */
    s_canTxMsg.efc = 0U;
    s_canTxMsg.mm  = 0xAAU;

    for (i = 0U; i < 8U; i++)
    {
        s_canTxMsg.data[i] = (Uint16)(pu16Data[i] & 0xFFU);
    }

    MCAN_writeMsgRam(DAB_CAN_BASE, MCAN_MEM_TYPE_BUF, (uint32_t)u16BufIdx, &s_canTxMsg);
    /* ★ 直写 TXBAR（写 1 置位），避免 DriverLib 读-改-写与多缓冲/ISR 竞态 */
    HWREG(DAB_CAN_BASE + MCAN_TXBAR) = (1UL << u16BufIdx);
    return 1U;
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
        /* ★ 直写 TXBCR（写 1 置位，写 0 无效），免读-改-写 */
        HWREG(DAB_CAN_BASE + MCAN_TXBCR) = (1UL << u16BufIdx);
    }
}

/**
 * @brief  从指定 RX FIFO 取一帧
 * @param  u32FifoNum MCAN_RX_FIFO_NUM_0（WATCHID）/ MCAN_RX_FIFO_NUM_1（AVERID）
 * @return FIFO 无帧、取到有效帧或已丢弃一帧无效数据
 */
DAB_CAN_RX_RESULT DabCan_Recv(uint32_t u32FifoNum, DATACanType *pMsg)
{
    MCAN_RxFIFOStatus st;
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

    MCAN_readMsgRam(DAB_CAN_BASE, MCAN_MEM_TYPE_FIFO, 0U, u32FifoNum, &s_canRxMsg);
    MCAN_writeRxFIFOAck(DAB_CAN_BASE, u32FifoNum, st.getIdx);

    if ((s_canRxMsg.xtd == 0U) || (s_canRxMsg.rtr != 0U) ||
        (s_canRxMsg.fdf != 0U) || (s_canRxMsg.dlc != 8U))
    {
        return DAB_CAN_RX_INVALID;
    }

    pMsg->CanId.all = s_canRxMsg.id & 0x1FFFFFFFUL;
    for (i = 0U; i < 8U; i++)
    {
        pMsg->CanData.Dat[i] = (Uint16)(s_canRxMsg.data[i] & 0xFFU);
    }
    return DAB_CAN_RX_VALID;
}

/* ============================================================================
 * MCAN 硬件初始化（经典 CAN 500 kbps，双 RX FIFO + 3 个发送缓冲）
 *
 * if + 基数：各等待阶段只查一次就绪并 ++s_initPolls，超时置错后回到 IDLE；
 * 无 DEVICE_DELAY_US 忙等。上电由 init_Drv_can 同步推完；运行中 Reinit 只启动，
 * 5ms 经 DabCan_InitPoll 推进，避免堵住主循环其它节拍。
 * ============================================================================ */

/* 初始化参数常驻 Flash/常量区，init/Reinit 不再把 ~120 字结构体压任务栈 */
static const MCAN_InitParams s_canInitParams =
{
    .fdMode          = 0U,
    .brsEnable       = 0U,
    .txpEnable       = 0U,
    .efbi            = 0U,
    .pxhddisable     = 0U,
    .darEnable       = 0U,
    .wkupReqEnable   = 0U,
    .autoWkupEnable  = 0U,
    .emulationEnable = 1U,
    .wdcPreload      = 255U,
    .tdcEnable       = 0U
};

static const MCAN_ConfigParams s_canConfigParams =
{
    .monEnable        = 0U,
    .asmEnable        = 0U,
    .tsPrescalar      = 15U,
    .tsSelect         = 0U,
    .timeoutSelect    = 0U,
    .timeoutPreload   = 0xFFFFU,
    .timeoutCntEnable = 0U,
    .filterConfig     =
    {
        .rrfe = 1U,    /* 拒绝远程扩展帧 */
        .rrfs = 1U,    /* 拒绝远程标准帧 */
        .anfe = 2U,    /* 10 = 不匹配的扩展帧丢弃 */
        .anfs = 2U     /* 10 = 不匹配的标准帧丢弃 */
    }
};

static const MCAN_MsgRAMConfigParams s_canRamParams =
{
    .flssa                = 0U,
    .lss                  = 0U,
    .flesa                = DAB_CAN_EXT_FILT_START,
    .lse                  = DAB_CAN_EXT_FILT_NUM,
    .txStartAddr          = DAB_CAN_TX_BUF_START,
    .txBufNum             = DAB_CAN_TX_BUF_NUM,
    .txFIFOSize           = 0U,
    .txBufMode            = 0U,
    .txBufElemSize        = 0U,
    .txEventFIFOStartAddr = DAB_CAN_TX_EVT_START,
    .txEventFIFOSize      = 0U,
    .txEventFIFOWaterMark = 0U,
    .rxFIFO0startAddr     = DAB_CAN_RX_FIFO0_START,
    .rxFIFO0size          = CAN_RX_FIFO_DEPTH,
    .rxFIFO0waterMark     = 0U,
    .rxFIFO0OpMode        = 0U,
    .rxFIFO0ElemSize      = 0U,
    .rxFIFO1startAddr     = DAB_CAN_RX_FIFO1_START,
    .rxFIFO1size          = CAN_RX_FIFO_DEPTH,
    .rxFIFO1waterMark     = 0U,
    .rxFIFO1OpMode        = 0U,
    .rxFIFO1ElemSize      = 0U,
    .rxBufStartAddr       = 0U,
    .rxBufElemSize        = 0U
};

static const MCAN_BitTimingParams s_canBitTimes =
{
    .nomRatePrescalar   = DAB_CAN_NBRP,
    .nomTimeSeg1        = DAB_CAN_NTSEG1,
    .nomTimeSeg2        = DAB_CAN_NTSEG2,
    .nomSynchJumpWidth  = DAB_CAN_NSJW,
    .dataRatePrescalar  = 0U,
    .dataTimeSeg1       = 0U,
    .dataTimeSeg2       = 0U,
    .dataSynchJumpWidth = 0U
};

/* Classic filter：EFID2=0x7FFFF 精确匹配 ADDR 19 位；SCOPE 不建项，由 anfe 丢弃 */
static const MCAN_ExtMsgIDFilterElement s_canExtFilter[DAB_CAN_EXT_FILT_NUM] =
{
    { (uint32_t)WATCHID, 0x1U, 0x0007FFFFUL, 0x2U },  /* → FIFO0 */
    { (uint32_t)AVERID,  0x2U, 0x0007FFFFUL, 0x2U }   /* → FIFO1 */
};

enum
{
    CAN_INIT_ST_IDLE = 0,
    CAN_INIT_ST_WAIT_RAM,
    CAN_INIT_ST_WAIT_INIT,
    CAN_INIT_ST_WAIT_NORMAL
};

static Uint16 s_initState = CAN_INIT_ST_IDLE;
static Uint16 s_initPolls;

static void DabCan_InitBegin(void)
{
    g_u16DabCanInitError = 1U;
    s_initPolls = 0U;
    SysCtl_setMCANClk(SYSCTL_MCANA, DAB_CAN_CLK_DIV);
    s_initState = CAN_INIT_ST_WAIT_RAM;
}

/* ★ 配置段一次写完；失败返回 0。 */
static Uint16 DabCan_InitApplyConfig(void)
{
    if ((MCAN_init(DAB_CAN_BASE, &s_canInitParams) != 0) ||
        (MCAN_config(DAB_CAN_BASE, &s_canConfigParams) != 0) ||
        (MCAN_setBitTime(DAB_CAN_BASE, &s_canBitTimes) != 0) ||
        (MCAN_msgRAMConfig(DAB_CAN_BASE, &s_canRamParams) != 0))
    {
        return 0U;
    }
    MCAN_addExtMsgIDFilter(DAB_CAN_BASE, 0U, &s_canExtFilter[0]); /* WATCHID → FIFO0 */
    MCAN_addExtMsgIDFilter(DAB_CAN_BASE, 1U, &s_canExtFilter[1]); /* AVERID  → FIFO1 */
    MCAN_setExtIDAndMask(DAB_CAN_BASE, 0x1FFFFFFFUL);
    return 1U;
}

static void DabCan_InitStep(Uint16 u16PollLimit)
{
    if (s_initState == CAN_INIT_ST_IDLE)
    {
        return;
    }

    if (s_initState == CAN_INIT_ST_WAIT_RAM)
    {
        if (MCAN_isMemInitDone(DAB_CAN_BASE) != 0U)
        {
            MCAN_setOpMode(DAB_CAN_BASE, MCAN_OPERATION_MODE_SW_INIT);
            s_initPolls = 0U;
            s_initState = CAN_INIT_ST_WAIT_INIT;
        }
        else if (++s_initPolls >= u16PollLimit)
        {
            g_u16DabCanInitError = 1U;
            s_initState = CAN_INIT_ST_IDLE;
        }
        return;
    }

    if (s_initState == CAN_INIT_ST_WAIT_INIT)
    {
        if (MCAN_getOpMode(DAB_CAN_BASE) == MCAN_OPERATION_MODE_SW_INIT)
        {
            if (DabCan_InitApplyConfig() == 0U)
            {
                g_u16DabCanInitError = 3U;
                s_initState = CAN_INIT_ST_IDLE;
                return;
            }
            MCAN_setOpMode(DAB_CAN_BASE, MCAN_OPERATION_MODE_NORMAL);
            s_initPolls = 0U;
            s_initState = CAN_INIT_ST_WAIT_NORMAL;
        }
        else if (++s_initPolls >= u16PollLimit)
        {
            g_u16DabCanInitError = 2U;
            s_initState = CAN_INIT_ST_IDLE;
        }
        return;
    }

    if (s_initState == CAN_INIT_ST_WAIT_NORMAL)
    {
        if (MCAN_getOpMode(DAB_CAN_BASE) == MCAN_OPERATION_MODE_NORMAL)
        {
            g_u16DabCanInitError = 0U;
            s_initState = CAN_INIT_ST_IDLE;
        }
        else if (++s_initPolls >= u16PollLimit)
        {
            g_u16DabCanInitError = 4U;
            s_initState = CAN_INIT_ST_IDLE;
        }
    }
}

void DabCan_InitPoll(void)
{
    DabCan_InitStep(DAB_CAN_TASK_INIT_POLLS);
}

Uint16 DabCan_InitBusy(void)
{
    return (s_initState != CAN_INIT_ST_IDLE) ? 1U : 0U;
}

void init_Drv_can(void)
{
    Uint16 guard;

    DabCan_InitBegin();
    /* 上电：带 1us 间隔同步推完（≈1ms/阶段，覆盖繁忙总线等待 11 隐性位） */
    for (guard = 0U; guard < (DAB_CAN_BOOT_INIT_POLLS * 3U); guard++)
    {
        DabCan_InitStep(DAB_CAN_BOOT_INIT_POLLS);
        if (s_initState == CAN_INIT_ST_IDLE)
        {
            break;
        }
        DEVICE_DELAY_US(1U);
    }
}

/* ★ 故障重试：只复位 MCANA 并启动状态机，本拍立即返回。 */
void DabCan_Reinit(void)
{
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_MCANA);
    DabCan_InitBegin();
}

void DabCan_CheckState(void)
{
    /* ★ 初始化推进中：本拍只走 InitPoll，不与运行态诊断交叉。 */
    if (s_initState != CAN_INIT_ST_IDLE)
    {
        DabCan_InitPoll();
        return;
    }

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

    if (DAB_CAN_PSR_BO_SET() &&
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

