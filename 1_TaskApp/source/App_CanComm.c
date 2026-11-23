/*************************************************************************************************
 * App_CanComm.c  —  CAN 应用协议（唯一协议层）
 *
 * 底层收发及故障恢复通过 ConfigChip_McanInit 接口。
 * 调度：1ms Rx→Tx，5ms CheckState/Reinit；逻辑地址由 UART 分址（无 CAN 握手、无 100us RX）。
 *************************************************************************************************/
#include "Includes.h"
#include <stddef.h>

#define CAN_SHARE_PERIOD_MS     (81U)  /* ★ 独立均流发送周期：81ms × 200ms 均流窗 = 2.47 帧/窗，保底 2 帧 */
#define CAN_SHARE_TX_MAX_MS     (40U)  /* 发送超时取消 ≈ 半周期 */
#define CAN_TELEMETRY_PERIOD_MS (20U)  /* 0x11 遥测周期 20ms（50 Hz） */
#define CAN_INIT_RETRY_TICKS    (20U)  /* ★ 5ms × 20 = 100ms */

/* ============================================================================
 * 遥测通道调度表（3:1 交织）
 * ============================================================================ */
static const Uint16 c_FastChList[] =
{
    0x00, 0x03, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
    0x2B, 0x2C, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20
};
#define FAST_CH_NUM    (sizeof(c_FastChList) / sizeof(c_FastChList[0]))

static const Uint16 c_SlowChList[] =
{
    0x01, 0x02, 0x04, 0x05, 0x06, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x21, 0x22,
    0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A
};
#define SLOW_CH_NUM    (sizeof(c_SlowChList) / sizeof(c_SlowChList[0]))

/* ★ 模块内状态集中；全零初值落 .bss，Init 一句清零避免二次初始化残留。 */
typedef struct
{
    Uint16 txPendMask;      /* bitN = CAN_TX_BUF_N 在途 */
    Uint16 ratioCnt;
    Uint16 fastIdx;
    Uint16 slowIdx;
    Uint16 unum;
    Uint16 shareTicks;
    Uint16 scopeTicks;
    Uint16 initRetryTicks;
} CanCommState_t;

static CanCommState_t s_st;
volatile uint32_t g_u32CanShareTxOk;
volatile uint32_t g_u32CanShareTxMiss;

/* ============================================================================
 * [改] 结算在途 TX（Scope / Share 共用 bit 位图）
 * @return 0=未结算；1=已结算且 TXBTO 成功；2=已结算但未成功
 * ============================================================================ */
static Uint16 CanComm_TxSettle(Uint16 buf,
                               volatile uint32_t *ok,
                               volatile uint32_t *miss)
{
    Uint16 bit = (Uint16)(1U << buf);

    if ((s_st.txPendMask & bit) == 0U)
    {
        return 0U;
    }
    if (DabCan_TxReady(buf) == 0U)
    {
        return 0U;
    }
    s_st.txPendMask = (Uint16)(s_st.txPendMask & (Uint16)(~bit));
    if (DabCan_TxSucceeded(buf) != 0U)
    {
        if (ok != 0)
        {
            (*ok)++;
        }
        return 1U;
    }
    if (miss != 0)
    {
        (*miss)++;
    }
    return 2U;
}

void CanComm_Init(void)
{
    /* DataScope / uMdlAddMes / sCanMes 已由 init_global_data 清零；此处只置本模块态。 */
    memset(&s_st, 0, sizeof(s_st));
    g_u32CanShareTxOk = 0U;
    g_u32CanShareTxMiss = 0U;
    uMdlAddMes.bCalIDCmd = ADDRcmd_on;   /* 等 UART 首次有效分址 */
}

void CanComm_SetLocalAddr(uint16_t u16Addr)
{
    if ((uMdlAddMes.uIndex != u16Addr) ||
        (uMdlAddMes.bCalIDCmd != ADDRcmd_end))
    {
        DabCan_CancelTx(CAN_TX_BUF_CMD);
        uMdlAddMes.uIndex = u16Addr;
        uMdlAddMes.bCalIDCmd = ADDRcmd_end;
        CurShare_ResetPeers();
    }
}

/* ============================================================================
 * 示波器分包上传（数据源 = DataScope，存满 900 点锁定后直接读）
 * ★ TXBTO 确认成功后推进 uCurrent_Tran_subscript。请求结束但未成功时重发当前包。
 * ★ 包序号塞进 ID 的 ADDR 高 11 位（低 8 位恒为 SCOPEID）。
 * ============================================================================ */
static void CanComm_SendScope(void)
{
    Uint16 u16Point;
    Uint16 u16Slot;
    Uint16 u0, u1, u2, u3;
    Uint16 settle;
    Uint16 tx[8];
    uint16_t intm;

    if ((s_st.txPendMask & (Uint16)(1U << CAN_TX_BUF_SCOPE)) != 0U)
    {
        settle = CanComm_TxSettle(CAN_TX_BUF_SCOPE, 0, 0);
        if (settle == 0U)
        {
            return;
        }
        if (settle == 1U)
        {
            DataScope.uCurrent_Tran_subscript++;
            if (DataScope.uCurrent_Tran_subscript >= (Uint16)SCOPENUM)
            {
                /* ★ 关中断原子重布防，避免与 EPWM10_ISR 撕写下标 */
                intm = __disable_interrupts();
                DataScope.uCurrent_Tran_subscript = 0U;
                DataScope.uCurrent_Save_subscript = 0U;
                DataScope.uPreTriggerIdx          = 0U;
                DataScope.uTran_CMD               = 0U;
                __restore_interrupts(intm);
                return;
            }
        }
    }

    u16Point = DataScope.uCurrent_Tran_subscript;
    u16Slot  = u16Point;
    if (u16Point < (Uint16)PRE_TRIGGER_NUM)
    {
        u16Slot = (Uint16)((DataScope.uPreTriggerIdx + u16Point) & PRE_TRIGGER_MASK);
    }

    u0 = (Uint16)DataScope.uDataScope0[u16Slot];
    u1 = (Uint16)DataScope.uDataScope1[u16Slot];
    u2 = (Uint16)DataScope.uDataScope2[u16Slot];
    u3 = (Uint16)DataScope.uDataScope3[u16Slot];

    CAN_DAT_PUT_BE16(tx, 0, u0);
    CAN_DAT_PUT_BE16(tx, 2, u1);
    CAN_DAT_PUT_BE16(tx, 4, u2);
    CAN_DAT_PUT_BE16(tx, 6, u3);

    if (DabCan_Send(CAN_TX_BUF_SCOPE,
                    CAN_ID_MAKE(SCOPEID | ((uint32_t)u16Point << 8),
                                uMdlAddMes.uIndex, BORAD_TO, ADDRcmd_end),
                    tx) == 0U)
    {
        return;
    }
    s_st.txPendMask = (Uint16)(s_st.txPendMask | (Uint16)(1U << CAN_TX_BUF_SCOPE));
}

/* ============================================================================
 * 0x20 在线参数整定
 * ============================================================================ */
static void CanComm_Cmd_0x20(const Uint16 *dat)
{
    volatile Uint16 *pDevCfgParam = (volatile Uint16 *)(volatile void *)&CfgParam;
    Uint16 DevParamSize  = (Uint16)(sizeof(CfgParam) / sizeof(Uint16));
    Uint16 uiRevindex    = CAN_DAT_BE16(dat, 1);
    Uint16 uiRevvalue;
    uint16_t intm;

    if (uiRevindex >= DevParamSize)
    {
        return;
    }
    uiRevvalue = CAN_DAT_BE16(dat, 6);
    pDevCfgParam[uiRevindex] = uiRevvalue;

    if (uiRevindex == 0U)
    {
        SM_SyncSysCmd();
    }

    /* ScopeTrigCmd：写非零请求一次录波，命令自清；忙时忽略，写 0 无动作。 */
    if (CfgParam.ScopeTrigCmd != 0U)
    {
        CfgParam.ScopeTrigCmd = 0U;
        intm = __disable_interrupts();
        if (DataScope.uTran_CMD == 0U)
        {
            DataScope.uCurrent_Save_subscript = PRE_TRIGGER_NUM;
            DataScope.uTran_CMD               = 1U;
        }
        __restore_interrupts(intm);
    }
}

/* ============================================================================
 * 0x11 遥测 —— offsetof 字偏移表（Flash 紧凑；载荷与原指针表/switch 一致）
 * 编码：bits[15:12]=kind，bits[11:0]=字偏移（相对 g_CanMon，或 Soft 下标）
 * PAIR：读 off 与 off+1（字段必须相邻）；PAIR0：hi 固定发 0
 * ============================================================================ */
#define TELE_PAIR   (0U)
#define TELE_PAIR0  (1U)
#define TELE_U32    (2U)
#define TELE_U16Z   (3U)
#define TELE_F32    (4U)
#define TELE_SOFT   (5U)

#define TELE_WOFF(field) \
    ((Uint16)(offsetof(CanMonitorData_t, field) / sizeof(Uint16)))
#define TE(kind, field) \
    ((Uint16)(((Uint16)(kind) << 12) | (TELE_WOFF(field) & 0x0FFFU)))
#define TES(idx) \
    ((Uint16)(((Uint16)TELE_SOFT << 12) | ((Uint16)(idx) & 0x0FFFU)))

static const uint32_t c_TeleSoft[2] =
{
    (uint32_t)SOFTER_TIME,
    (uint32_t)SOFTER_VER
};

/* 邻接静态检查：PAIR 通道依赖 lo/hi 连续布局 */
typedef char tele_adj_00[(TELE_WOFF(ADC.vbusPcs_dV) == TELE_WOFF(ADC.vbat_dV) + 1U) ? 1 : -1];
typedef char tele_adj_1d[(TELE_WOFF(RealAlarm.alarm) == TELE_WOFF(RealAlarm.fastfault) + 1U) ? 1 : -1];
typedef char tele_adj_20[(TELE_WOFF(SaveAlarm.savealarm) == TELE_WOFF(SaveAlarm.savefastfault) + 1U) ? 1 : -1];

static const Uint16 c_TeleCh[] =
{
    TE(TELE_PAIR,  ADC.vbat_dV),                 /* 0x00 */
    TE(TELE_PAIR,  ADC.vbus1_dV),                /* 0x01 */
    TE(TELE_PAIR,  ADC.vmBus_dV),                /* 0x02 */
    TE(TELE_PAIR,  ADC.iout1_cA),                /* 0x03 */
    TE(TELE_PAIR,  ADC.pout1_W),                 /* 0x04 */
    TE(TELE_PAIR,  ADC.tempInductor_dC),         /* 0x05 */
    TE(TELE_PAIR,  ADC.tempHeatsink_dC),         /* 0x06 */
    TE(TELE_PAIR,  LoopCtrl.vLoop1_Err_cV),      /* 0x07 */
    TE(TELE_PAIR,  LoopCtrl.vLoop2_Err_cV),      /* 0x08 */
    TE(TELE_PAIR,  LoopCtrl.vLoop1_DutyFf),      /* 0x09 */
    TE(TELE_PAIR,  LoopCtrl.iLoop1_Err_cA),      /* 0x0A */
    TE(TELE_PAIR,  LoopCtrl.iLoop2_Err_cA),      /* 0x0B */
    TE(TELE_PAIR,  LoopCtrl.iLoop1_DutyFf),      /* 0x0C */
    TE(TELE_PAIR,  Setpoint.voutSet_dV),         /* 0x0D */
    TE(TELE_PAIR,  Setpoint.vbus1Ref_dV),        /* 0x0E */
    TE(TELE_PAIR0, Setpoint.vbus1Cmd_dV),        /* 0x0F */
    TE(TELE_PAIR,  Setpoint.vbus2Ref_dV),        /* 0x10 */
    TE(TELE_PAIR0, Setpoint.vbus2Cmd_dV),        /* 0x11 */
    TE(TELE_PAIR,  Setpoint.iCharSet_cA),        /* 0x12 */
    TE(TELE_PAIR,  Setpoint.iReCharSet_cA),      /* 0x13 */
    TE(TELE_PAIR,  Setpoint.droopCur_dv),        /* 0x14 */
    TE(TELE_PAIR,  PowerLimit.bmsCharPowSet_W),  /* 0x15 */
    TE(TELE_PAIR,  PowerLimit.sysMaxChgPow_W),   /* 0x16 */
    TE(TELE_PAIR,  PowerLimit.tempLimChar_W),    /* 0x17 */
    TE(TELE_PAIR,  PowerLimit.ibatLimChar_cA),   /* 0x18 */
    TE(TELE_PAIR,  Status.runFlag.all),          /* 0x19 */
    TE(TELE_PAIR,  Status.canRecCnt),            /* 0x1A */
    TE(TELE_U32,   RealAlarm.slowfault1),        /* 0x1B */
    TE(TELE_U16Z,  RealAlarm.alarm),             /* 0x1C */
    TE(TELE_PAIR,  RealAlarm.fastfault),         /* 0x1D */
    TE(TELE_U32,   SaveAlarm.saveslowfault1),    /* 0x1E */
    TE(TELE_U16Z,  SaveAlarm.savealarm),         /* 0x1F */
    TE(TELE_PAIR,  SaveAlarm.savefastfault),     /* 0x20 */
    TES(0),                                      /* 0x21 SOFTER_TIME */
    TES(1),                                      /* 0x22 SOFTER_VER */
    TE(TELE_F32,   utest1),                      /* 0x23 */
    TE(TELE_F32,   utest2),                      /* 0x24 */
    TE(TELE_F32,   utest3),                      /* 0x25 */
    TE(TELE_F32,   utest4),                      /* 0x26 */
    TE(TELE_F32,   utest5),                      /* 0x27 */
    TE(TELE_F32,   utest6),                      /* 0x28 */
    TE(TELE_F32,   utest7),                      /* 0x29 */
    TE(TELE_F32,   utest8),                      /* 0x2A */
    TE(TELE_PAIR,  LoopCtrl.vLoop_Ref_dV),       /* 0x2B */
    TE(TELE_PAIR,  LoopCtrl.iLoop1_Ff_cA)        /* 0x2C */
};

#define TELE_CH_NUM  (sizeof(c_TeleCh) / sizeof(c_TeleCh[0]))

static uint32_t CanComm_PackTele(Uint16 ch)
{
    const Uint16 *base = (const Uint16 *)(const void *)&g_CanMon;
    Uint16 enc;
    Uint16 kind;
    Uint16 off;
    CHANGTO_FLOAT cf;

    cf.f_32 = 0U;
    if (ch >= (Uint16)TELE_CH_NUM)
    {
        return 0U;
    }

    enc  = c_TeleCh[ch];
    kind = (Uint16)(enc >> 12);
    off  = (Uint16)(enc & 0x0FFFU);

    switch (kind)
    {
        case TELE_PAIR:
            cf.u16.lo = base[off];
            cf.u16.hi = base[off + 1U];
            break;
        case TELE_PAIR0:
            cf.u16.lo = base[off];
            break;
        case TELE_U32:
            cf.f_32 = *(const uint32_t *)(const void *)&base[off];
            break;
        case TELE_U16Z:
            cf.f_32 = (uint32_t)base[off];
            break;
        case TELE_F32:
            cf.all = *(const float *)(const void *)&base[off];
            break;
        case TELE_SOFT:
            if (off < 2U)
            {
                cf.f_32 = c_TeleSoft[off];
            }
            break;
        default:
            break;
    }
    return cf.f_32;
}

static void CanComm_Cmd_0x11(void)
{
    Uint16 uiRevValueTYP;
    uint32_t u32Payload;
    Uint16 tx[8];

    if (DabCan_TxReady(CAN_TX_BUF_WATCH) == 0U)
    {
        return;
    }

    if ((s_st.ratioCnt & 0x03U) != 0x03U)
    {
        uiRevValueTYP = c_FastChList[s_st.fastIdx++];
        if (s_st.fastIdx >= FAST_CH_NUM)
        {
            s_st.fastIdx = 0U;
        }
    }
    else
    {
        uiRevValueTYP = c_SlowChList[s_st.slowIdx++];
        if (s_st.slowIdx >= SLOW_CH_NUM)
        {
            s_st.slowIdx = 0U;
        }
    }
    s_st.ratioCnt++;

    u32Payload = CanComm_PackTele(uiRevValueTYP);

    tx[0] = CAN_TELEMETRY_CMD;
    tx[1] = (Uint16)((uiRevValueTYP >> 8) & 0xFFU);
    tx[2] = (Uint16)(uiRevValueTYP & 0xFFU);
    tx[3] = 0x00U;
    tx[4] = (Uint16)((u32Payload >> 24) & 0xFFU);
    tx[5] = (Uint16)((u32Payload >> 16) & 0xFFU);
    tx[6] = (Uint16)((u32Payload >> 8)  & 0xFFU);
    tx[7] = (Uint16)(u32Payload & 0xFFU);

    (void)DabCan_Send(CAN_TX_BUF_WATCH,
                      CAN_ID_MAKE(WATCHID, uMdlAddMes.uIndex, BORAD_TO, ADDRcmd_end),
                      tx);
}

/* ============================================================================
 * 单帧协议解析（硬件已 19 位精确滤 WATCH/AVER）
 * ============================================================================ */
static void CanComm_ParseFrame(const DATACanType *pMsg)
{
    if (pMsg->CanId.bit.ADDR == WATCHID)
    {
        if (((pMsg->CanId.bit.PTP == BORAD_ALL) ||
             ((pMsg->CanId.bit.PTP == BORAD_TO) &&
              (pMsg->CanId.bit.LogADDR == uMdlAddMes.uIndex))) &&
            (pMsg->CanData.Dat[0] == 0x20U))
        {
            CanComm_Cmd_0x20(pMsg->CanData.Dat);
        }
    }
    else if ((pMsg->CanId.bit.ADDR == AVERID) && (pMsg->CanData.Dat[0] == 0x41U))
    {
        sCanMes.uCanRec_count += CurShare_Rx41((Uint16)pMsg->CanId.bit.LogADDR,
                                               pMsg->CanData.Dat);
    }
}

void CanComm_Task_Tx_1ms(void)
{
    uint32_t u32ShareId;
    Uint16   tx[8];

    /* 有界回零：严格 20 ms，无 % 除法、无 Uint16 回绕抖动 */
    if (++s_st.unum >= CAN_TELEMETRY_PERIOD_MS)
    {
        s_st.unum = 0U;
        CanComm_Cmd_0x11();
    }

    if ((DataScope.uTran_CMD != 0U) &&
        (DataScope.uCurrent_Save_subscript >= (Uint16)SCOPENUM))
    {
        if (++s_st.scopeTicks >= 3U)
        {
            s_st.scopeTicks = 0U;
            CanComm_SendScope();
        }
    }

    (void)CanComm_TxSettle(CAN_TX_BUF_CMD,
                           &g_u32CanShareTxOk, &g_u32CanShareTxMiss);

    ++s_st.shareTicks;
    if (s_st.shareTicks == CAN_SHARE_TX_MAX_MS)
    {
        DabCan_CancelTx(CAN_TX_BUF_CMD);
    }
    if (s_st.shareTicks >= CAN_SHARE_PERIOD_MS)
    {
        s_st.shareTicks = 0U;
        if (CurShare_FillTx41(tx) != 0U)
        {
            u32ShareId = CAN_ID_MAKE(AVERID, uMdlAddMes.uIndex, BORAD_ALL, ADDRcmd_end);
            if (((s_st.txPendMask & (Uint16)(1U << CAN_TX_BUF_CMD)) == 0U) &&
                (DabCan_Send(CAN_TX_BUF_CMD, u32ShareId, tx) != 0U))
            {
                s_st.txPendMask = (Uint16)(s_st.txPendMask | (Uint16)(1U << CAN_TX_BUF_CMD));
            }
            else
            {
                g_u32CanShareTxMiss++;
            }
        }
    }
}

static void CanComm_PollRxFifo(uint32_t fifo)
{
    Uint16 i;
    DATACanType frame;
    DAB_CAN_RX_RESULT result;

    for (i = 0U; i < CAN_RX_FIFO_DEPTH; i++)
    {
        result = DabCan_Recv(fifo, &frame);
        if (result == DAB_CAN_RX_EMPTY)
        {
            break;
        }
        if (result == DAB_CAN_RX_VALID)
        {
            CanComm_ParseFrame(&frame);
        }
    }
}

void CanComm_Task_Rx(void)
{
    CanComm_PollRxFifo(CAN_RX_FIFO_WATCH);  /* FIFO0：WATCHID */
    CanComm_PollRxFifo(CAN_RX_FIFO_AVER);   /* FIFO1：AVERID */
}

void CanComm_Task_5ms(void)
{
    DabCan_CheckState();

    if (g_u16DabCanInitError == 0U)
    {
        s_st.initRetryTicks = 0U;
        return;
    }
    if (DabCan_InitBusy() != 0U)
    {
        s_st.initRetryTicks = 0U;
        return;
    }
    if (++s_st.initRetryTicks >= CAN_INIT_RETRY_TICKS)
    {
        s_st.initRetryTicks = 0U;
        DabCan_Reinit();
    }
}
