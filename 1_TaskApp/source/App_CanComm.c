/*************************************************************************************************
 * App_CanComm.c  —  CAN 应用协议（唯一协议层）
 *
 * 底层收发及故障恢复通过 ConfigChip_McanInit 接口。
 *************************************************************************************************/
#include "Includes.h"

#define CAN_SHARE_PERIOD_MS     (81U)  /* ★ 独立均流发送周期：81ms × 200ms 均流窗 = 2.47 帧/窗，保底 2 帧 */
#define CAN_SHARE_TX_MAX_MS     (40U)  /* 发送超时取消 ≈ 半周期 */
#define CAN_INIT_RETRY_TICKS    (20U)  /* ★ 5ms × 20 = 100ms */

static DATACanType CanRecData;
static Uint16 s_txMsgData[8];

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

/* ============================================================================
 * 协议状态复位
 * ============================================================================ */
static void CanComm_ResetProtoData(void)
{
    Uint16 i;

    memset(&uMdlAddMes, 0, sizeof(uMdlAddMes));
    memset((void *)&sCanMes, 0, sizeof(sCanMes));
    memset((void *)&DataScope, 0, sizeof(DataScope)); /* 显式去掉 volatile 限定，仅初始化用 */

    uMdlAddMes.uIndex       = 0U;
    uMdlAddMes.MDADDRID_num = 0U;
    for (i = 0U; i <= (Uint16)recnum; i++)
    {
        uMdlAddMes.MDADDRID[i] = 0U;
    }

    uMdlAddMes.uSelfID                  = 0x1234567UL;
    uMdlAddMes.uTranEND_num             = 0U;
    sCanMes.uCalADDR_Cyle               = 0U;
    sCanMes.uCanRec_count               = 0U;
    uMdlAddMes.uCanState.all            = 0U;
    uMdlAddMes.uCanState.bits.bCalIDCmd = ADDRcmd_on;   /* ★ 等待首次有效地址 */
    uMdlAddMes.uReqIDnum                = 0U;
    uMdlAddMes.uCanState.bits.bRecAddr  = ADDRcmd_end;
}

/* ★ 示波器包与均流帧分别跟踪各自的在途请求。 */
static Uint16 s_scopeTxPend = 0U;
static Uint16 s_shareTxPend = 0U;
volatile uint32_t g_u32CanShareTxOk;
volatile uint32_t g_u32CanShareTxMiss;

void CanComm_Init(void)
{
    CanComm_ResetProtoData();
    s_scopeTxPend = 0U;
    s_shareTxPend = 0U;
    g_u32CanShareTxOk = 0U;
    g_u32CanShareTxMiss = 0U;
}

/* ============================================================================
 * 示波器分包上传（数据源 = DataScope，存满 900 点锁定后直接读）
 * ★ TXBTO 确认成功后推进 uCurrent_Tran_subscript。请求结束但未成功时重发当前包。
 * ★ 包序号塞进 ID 的 ADDR 高 11 位（低 8 位恒为 SCOPEID 供滤波）：
 *   上位机可按 (ID>>8)&0x7FF 重组/检丢；不解析序号的上位机完全兼容。
 * ============================================================================ */
static void CanComm_SendScope(void)
{
    Uint16 u16Idx;
    Uint16 UDATA0, UDATA1, UDATA2, UDATA3;

    /* 1. 结算上一帧：确认成功才推进，满 SCOPENUM 即结束并重新布防 */
    if (s_scopeTxPend != 0U)
    {
        if (DabCan_TxReady(CAN_TX_BUF_SCOPE) == 0U)
        {
            return;
        }
        s_scopeTxPend = 0U;
        if (DabCan_TxSucceeded(CAN_TX_BUF_SCOPE) != 0U)
        {
            DataScope.uCurrent_Tran_subscript++;
            if (DataScope.uCurrent_Tran_subscript >= (Uint16)SCOPENUM)
            {
                DataScope.uCurrent_Tran_subscript = 0U;
                DataScope.uTran_CMD               = 0U;   /* 传完：清锁定，重新布防 */
                DataScope.uCurrent_Save_subscript = 0U;
                DataScope.uPreTriggerIdx          = 0U;
                return;
            }
        }
    }

    /* 2. 录波未存满或发包邮箱未就绪：本拍不发 */
    if ((DataScope.uTran_CMD == 0U) ||
        (DataScope.uCurrent_Save_subscript < (Uint16)SCOPENUM) ||
        (DabCan_TxReady(CAN_TX_BUF_SCOPE) == 0U))
    {
        return;
    }

    /* 3. 组包发送：4 通道大端，点序号塞进 ID。
     * ★ 点序 → 槽位映射（经典 scopeGet 零拷贝）：预触发段点 i 的实际槽位 =
     *   (uPreTriggerIdx + i) & 63（槽 uPreTriggerIdx 为最旧样本）；触发后段线性
     *   存放，槽位即点序。缓冲不做物理旋转，上传顺序即时间序。 */
    u16Idx = DataScope.uCurrent_Tran_subscript;
    if (u16Idx < (Uint16)PRE_TRIGGER_NUM)
    {
        u16Idx = (Uint16)((DataScope.uPreTriggerIdx + u16Idx) & PRE_TRIGGER_MASK);
    }

    UDATA0 = (Uint16)DataScope.uDataScope0[u16Idx];
    UDATA1 = (Uint16)DataScope.uDataScope1[u16Idx];
    UDATA2 = (Uint16)DataScope.uDataScope2[u16Idx];
    UDATA3 = (Uint16)DataScope.uDataScope3[u16Idx];

    s_txMsgData[0] = (UDATA0 >> 8) & 0xFFU;
    s_txMsgData[1] = (UDATA0     ) & 0xFFU;
    s_txMsgData[2] = (UDATA1 >> 8) & 0xFFU;
    s_txMsgData[3] = (UDATA1     ) & 0xFFU;
    s_txMsgData[4] = (UDATA2 >> 8) & 0xFFU;
    s_txMsgData[5] = (UDATA2     ) & 0xFFU;
    s_txMsgData[6] = (UDATA3 >> 8) & 0xFFU;
    s_txMsgData[7] = (UDATA3     ) & 0xFFU;

    if (DabCan_Send(CAN_TX_BUF_SCOPE,
                    CAN_ID_MAKE(SCOPEID | ((uint32_t)u16Idx << 8),
                                uMdlAddMes.uIndex, BORAD_TO, ADDRcmd_end),
                    s_txMsgData) == 0U)
    {
        return;   /* 未提交：下标不动 */
    }
    s_scopeTxPend = 1U;
}

/* ============================================================================
 * 0x20 在线参数整定
 * ============================================================================ */
static void CanComm_Cmd_0x20(void)
{
    volatile Uint16 *pDevCfgParam = (volatile Uint16 *)(volatile void *)&CfgParam;
    Uint16 DevParamSize  = (Uint16)(sizeof(CfgParam) / sizeof(Uint16));
    Uint16 uiRevindex    = CAN_DAT_BE16(CanRecData.CanData.Dat, 1);
    Uint16 uiRevvalue;

    if (uiRevindex >= DevParamSize)
    {
        return;
    }
    uiRevvalue = CAN_DAT_BE16(CanRecData.CanData.Dat, 6);
    pDevCfgParam[uiRevindex] = uiRevvalue;  /* ★ 保留对共享配置的 volatile 写入 */

    /* 索引 0 = SysCmd 字：与串口一致，当拍同步到 SysCmd（强充/强放、onoff） */
    if (uiRevindex == 0U)
    {
        SM_SyncSysCmd();
    }

    /* ScopeTrigCmd：上位机写非 0 手动触发一次录波，读回自清零；
       与 ISR 故障触发同一路径：置锁定标志，从第 64 点起线性记录 */
    if (CfgParam.ScopeTrigCmd != 0U)
    {
        CfgParam.ScopeTrigCmd = 0U;
//        if (DataScope.uTran_CMD == 0U)
        {
            DataScope.uTran_CMD               = 1U;
            DataScope.uCurrent_Save_subscript = PRE_TRIGGER_NUM;
        }
    }
}

/* ============================================================================
 * 0x11 遥测
 * ============================================================================ */
static void CanComm_Cmd_0x11(void)
{
    static Uint16 u16RatioCnt = 0U;
    static Uint16 u16FastIdx  = 0U;
    static Uint16 u16SlowIdx  = 0U;

    Uint16 uiRevValueTYP;
    CHANGTO_FLOAT cf;

    if (DabCan_TxReady(CAN_TX_BUF_WATCH) == 0U)
    {
        return;
    }

    if ((u16RatioCnt & 0x03U) != 0x03U)
    {
        uiRevValueTYP = c_FastChList[u16FastIdx++];
        if (u16FastIdx >= FAST_CH_NUM)
        {
            u16FastIdx = 0U;
        }
    }
    else
    {
        uiRevValueTYP = c_SlowChList[u16SlowIdx++];
        if (u16SlowIdx >= SLOW_CH_NUM)
        {
            u16SlowIdx = 0U;
        }
    }
    u16RatioCnt++;

    cf.f_32 = 0U;

    switch (uiRevValueTYP)
    {
        case 0x00:
            cf.u16.lo = g_CanMon.ADC.vbat_dV;
            cf.u16.hi = g_CanMon.ADC.vbusPcs_dV;
            break;
        case 0x01:
            cf.u16.lo = g_CanMon.ADC.vbus1_dV;
            cf.u16.hi = g_CanMon.ADC.vbus2_dV;
            break;
        case 0x02:
            cf.u16.lo = g_CanMon.ADC.vmBus_dV;
            cf.i16.hi = g_CanMon.ADC.sampleIr_cA;
            break;
        case 0x03:
            cf.i16.lo = g_CanMon.ADC.iout1_cA;
            cf.i16.hi = g_CanMon.ADC.iout2_cA;
            break;
        case 0x04:
            cf.i16.lo = g_CanMon.ADC.pout1_W;
            cf.i16.hi = g_CanMon.ADC.pout2_W;
            break;
        case 0x05:
            cf.i16.lo = g_CanMon.ADC.tempInductor_dC;
            cf.i16.hi = g_CanMon.ADC.tempAmbient_dC;
            break;
        case 0x06:
            cf.i16.lo = g_CanMon.ADC.tempHeatsink_dC;
            cf.i16.hi = g_CanMon.ADC.tempMos_dC;
            break;
        case 0x07:
            cf.i16.lo = g_CanMon.LoopCtrl.vLoop1_Err_cV;
            cf.i16.hi = g_CanMon.LoopCtrl.vLoop1_Out;
            break;
        case 0x08:
            cf.i16.lo = g_CanMon.LoopCtrl.vLoop2_Err_cV;
            cf.i16.hi = g_CanMon.LoopCtrl.vLoop2_Out;
            break;
        case 0x09:
            cf.u16.lo = g_CanMon.LoopCtrl.vLoop1_DutyFf;
            cf.u16.hi = g_CanMon.LoopCtrl.vLoop2_DutyFf;
            break;
        case 0x0A:
            cf.i16.lo = g_CanMon.LoopCtrl.iLoop1_Err_cA;
            cf.i16.hi = g_CanMon.LoopCtrl.iLoop1_Out;
            break;
        case 0x0B:
            cf.i16.lo = g_CanMon.LoopCtrl.iLoop2_Err_cA;
            cf.i16.hi = g_CanMon.LoopCtrl.iLoop2_Out;
            break;
        case 0x0C:
            cf.u16.lo = g_CanMon.LoopCtrl.iLoop1_DutyFf;
            cf.u16.hi = g_CanMon.LoopCtrl.iLoop2_DutyFf;
            break;
        case 0x0D:
            cf.u16.lo = g_CanMon.Setpoint.voutSet_dV;
            cf.i16.hi = g_CanMon.Setpoint.vbusDroop_dV;
            break;
        case 0x0E:  /* lo=控制 ref，hi=下发设定 */
            cf.u16.lo = g_CanMon.Setpoint.vbus1Ref_dV;
            cf.u16.hi = g_CanMon.Setpoint.vbus1Set_dV;
            break;
        case 0x0F:  /* lo=步进设定 Ramp.v */
            cf.u16.lo = g_CanMon.Setpoint.vbus1Cmd_dV;
            cf.u16.hi = 0U;
            break;
        case 0x10:  /* 母线2 同外环：ref / 下发设定 */
            cf.u16.lo = g_CanMon.Setpoint.vbus2Ref_dV;
            cf.u16.hi = g_CanMon.Setpoint.vbus2Set_dV;
            break;
        case 0x11:  /* 母线2 步进设定 */
            cf.u16.lo = g_CanMon.Setpoint.vbus2Cmd_dV;
            cf.u16.hi = 0U;
            break;
        case 0x12:
            cf.i16.lo = g_CanMon.Setpoint.iCharSet_cA;
            cf.i16.hi = g_CanMon.Setpoint.ibus1ref_cA;
            break;
        case 0x13:
            cf.i16.lo = g_CanMon.Setpoint.iReCharSet_cA;
            cf.i16.hi = g_CanMon.Setpoint.ibus2ref_cA;
            break;
        case 0x14:  /* lo=下垂 droop[0.1V]，hi=均流 voltAdj[0.1V] */
            cf.i16.lo = g_CanMon.Setpoint.droopCur_dv;
            cf.i16.hi = g_CanMon.Setpoint.canAverCur_dv;
            break;
        case 0x15:
            cf.u16.lo = g_CanMon.PowerLimit.bmsCharPowSet_W;
            cf.u16.hi = g_CanMon.PowerLimit.bmsrecharPowSet_W;
            break;
        case 0x16:
            cf.u16.lo = g_CanMon.PowerLimit.sysMaxChgPow_W;
            cf.u16.hi = g_CanMon.PowerLimit.sysMaxRechPow_W;
            break;
        case 0x17:
            cf.u16.lo = g_CanMon.PowerLimit.tempLimChar_W;
            cf.u16.hi = g_CanMon.PowerLimit.tempLimRech_W;
            break;
        case 0x18:
            cf.u16.lo = g_CanMon.PowerLimit.ibatLimChar_cA;
            cf.u16.hi = g_CanMon.PowerLimit.vbatLimRech_W;
            break;
        case 0x19:
            cf.u16.lo = g_CanMon.Status.runFlag.all;
            cf.u16.hi = g_CanMon.Status.dabRunSta;
            break;
        case 0x1A:
            cf.u16.lo = g_CanMon.Status.canRecCnt;
            cf.u16.hi = g_CanMon.Status.resv;
            break;
        case 0x1B:
            cf.f_32 = g_CanMon.RealAlarm.slowfault1;
            break;
        case 0x1C:
            cf.f_32 = (Uint32)g_CanMon.RealAlarm.alarm; /* 原 slowfault2 槽：alm 零扩展 */
            break;
        case 0x1D:
            cf.u16.lo = g_CanMon.RealAlarm.fastfault;
            cf.u16.hi = g_CanMon.RealAlarm.alarm;
            break;
        case 0x1E:
            cf.f_32 = g_CanMon.SaveAlarm.saveslowfault1;
            break;
        case 0x1F:
            cf.f_32 = (Uint32)g_CanMon.SaveAlarm.savealarm; /* 原 saveslowfault2 槽 */
            break;
        case 0x20:
            cf.u16.lo = g_CanMon.SaveAlarm.savefastfault;
            cf.u16.hi = g_CanMon.SaveAlarm.savealarm;
            break;
        case 0x21:
            cf.f_32 = (uint32_t)SOFTER_TIME;
            break;
        case 0x22:
            cf.f_32 = (uint32_t)SOFTER_VER;
            break;
        case 0x23: cf.all = g_CanMon.utest1; break;
        case 0x24: cf.all = g_CanMon.utest2; break;
        case 0x25: cf.all = g_CanMon.utest3; break;
        case 0x26: cf.all = g_CanMon.utest4; break;
        case 0x27: cf.all = g_CanMon.utest5; break;
        case 0x28: cf.all = g_CanMon.utest6; break;
        case 0x29: cf.all = g_CanMon.utest7; break;
        case 0x2A: cf.all = g_CanMon.utest8; break;
        case 0x2B:
            cf.i16.lo = g_CanMon.LoopCtrl.vLoop_Ref_dV;
            cf.i16.hi = g_CanMon.LoopCtrl.vLoop_Ff_cA;
            break;
        case 0x2C:
            cf.i16.lo = g_CanMon.LoopCtrl.iLoop1_Ff_cA;
            cf.i16.hi = g_CanMon.LoopCtrl.iLoop2_Ff_cA;
            break;
        default:
            cf.f_32 = 0U;
            break;
    }

    s_txMsgData[0] = CAN_TELEMETRY_CMD;
    s_txMsgData[1] = (Uint16)((uiRevValueTYP >> 8) & 0xFFU);
    s_txMsgData[2] = (Uint16)(uiRevValueTYP & 0xFFU);
    s_txMsgData[3] = 0x00U;
    s_txMsgData[4] = (Uint16)((cf.f_32 >> 24) & 0xFFU);
    s_txMsgData[5] = (Uint16)((cf.f_32 >> 16) & 0xFFU);
    s_txMsgData[6] = (Uint16)((cf.f_32 >> 8)  & 0xFFU);
    s_txMsgData[7] = (Uint16)(cf.f_32 & 0xFFU);

    (void)DabCan_Send(CAN_TX_BUF_WATCH,
                      CAN_ID_MAKE(WATCHID, uMdlAddMes.uIndex, BORAD_TO, ADDRcmd_end),
                      s_txMsgData);
}

/* ============================================================================
 * 单帧协议解析
 * ============================================================================ */
static void CanComm_ParseFrame(void)
{
    Uint16 uiRevCMDTYP;
    Uint32 u32Addr = CanRecData.CanId.bit.ADDR & 0xFFUL;

    /* ★ SCOPEID 帧可从 FIFO0 取出，当前协议仅定义波形上传，暂无接收命令。 */
    if ((u32Addr != (Uint32)WATCHID) && (u32Addr != (Uint32)AVERID) &&
        (u32Addr != (Uint32)SCOPEID))
    {
        return;
    }

    if (CanRecData.CanId.bit.ADDR == WATCHID)
    {
        if ((CanRecData.CanId.bit.PTP == BORAD_ALL) ||
            ((CanRecData.CanId.bit.PTP == BORAD_TO) &&
             (CanRecData.CanId.bit.LogADDR == uMdlAddMes.uIndex)))
        {
            uiRevCMDTYP = CanRecData.CanData.Dat[0];
            switch (uiRevCMDTYP)
            {
                case 0x20U:
                    CanComm_Cmd_0x20();
                    break;
                default:
                    break;
            }
        }
    }
    else if ((CanRecData.CanId.bit.ADDR == AVERID) && (CanRecData.CanData.Dat[0] == 0x41U))
    {
        /* ★ 只统计已更新对端状态的 0x41。 */
        sCanMes.uCanRec_count += CurShare_Rx41((Uint16)CanRecData.CanId.bit.LogADDR,
                                               CanRecData.CanData.Dat);
    }
}

/* ============================================================================
 * 周期任务
 * ============================================================================ */
void CanComm_Task_Tx_1ms(void)
{
    static Uint16 unum = 0U;
    static Uint16 shareTicks = 0U;
    static Uint16 scopeTicks = 0U;
    uint32_t      u32ShareId;

    unum++;

    if (((unum % 10U) == 0U) && ((unum % 4U) != 0U))
    {
        CanComm_Cmd_0x11();
    }

    /* 示波器上传：存满锁定期间每 3ms 一包（900 帧 ≈ 2.7s；未存满节拍停摆） */
    if ((DataScope.uTran_CMD != 0U) &&
        (DataScope.uCurrent_Save_subscript >= (Uint16)SCOPENUM))
    {
        if (++scopeTicks >= 3U)
        {
            scopeTicks = 0U;
            CanComm_SendScope();
        }
    }

    /* ★ 先结算上一帧，再提交下一帧。提交新请求会清除旧 TXBTO。 */
    if ((s_shareTxPend != 0U) && (DabCan_TxReady(CAN_TX_BUF_CMD) != 0U))
    {
        s_shareTxPend = 0U;
        if (DabCan_TxSucceeded(CAN_TX_BUF_CMD) != 0U)
        {
            g_u32CanShareTxOk++;
        }
        else
        {
            g_u32CanShareTxMiss++;
        }
    }

    /* ★ 专用 TX Buffer 独立调度，避免跳拍形成 162ms 空档。 */
    ++shareTicks;
    if (shareTicks == CAN_SHARE_TX_MAX_MS)
    {
        DabCan_CancelTx(CAN_TX_BUF_CMD);  /* ★ 清理未发出的旧均流状态 */
    }
    if (shareTicks >= CAN_SHARE_PERIOD_MS)
    {
        shareTicks = 0U;
        if (CurShare_FillTx41(s_txMsgData) != 0U)
        {
            /* ★ 0x41 收发统一走 AVERID。 */
            u32ShareId = CAN_ID_MAKE(AVERID, uMdlAddMes.uIndex, BORAD_ALL, ADDRcmd_end);
            if ((s_shareTxPend == 0U) &&
                (DabCan_Send(CAN_TX_BUF_CMD, u32ShareId, s_txMsgData) != 0U))
            {
                s_shareTxPend = 1U;
            }
            else
            {
                g_u32CanShareTxMiss++;
            }
        }
    }
}

/* ★ 每个 FIFO 每拍最多出队一个 FIFO 深度，给另一组留出处理机会。 */
static void CanComm_PollRxFifo(uint32_t fifo)
{
    Uint16 i;
    DAB_CAN_RX_RESULT result;

    for (i = 0U; i < CAN_RX_FIFO_DEPTH; i++)
    {
        result = DabCan_Recv(fifo, &CanRecData);
        if (result == DAB_CAN_RX_EMPTY)
        {
            break;
        }
        if (result == DAB_CAN_RX_VALID)
        {
            CanComm_ParseFrame();
        }
    }
}

void CanComm_Task_Rx(void)
{
    CanComm_PollRxFifo(CAN_RX_FIFO_WATCH);  /* ★ FIFO0：WATCHID (0xFF) 和 SCOPEID (0x1F) */
    CanComm_PollRxFifo(CAN_RX_FIFO_AVER);   /* ★ FIFO1：AVERID (0x10) */
}

void CanComm_Task_5ms(void)
{
    static Uint16 initRetryTicks;

    DabCan_CheckState();
    /* ★ 上电失败 / 异常 INIT / ASM 受限模式，均按 100ms 周期重试。 */
    if (g_u16DabCanInitError != 0U)
    {
        if (++initRetryTicks >= CAN_INIT_RETRY_TICKS)
        {
            initRetryTicks = 0U;
            DabCan_Reinit();
        }
    }

    sCanMes.uCalADDR_Cyle++;
    if (sCanMes.uCalADDR_Cyle >= uCalADDR_Cyle_15S)
    {
        sCanMes.uCalADDR_Cyle = uCalADDR_Cyle_15S;
    }

}
