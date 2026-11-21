/* App_UartProto.c — P550 串口协议 V1.3
 * 帧：AA 55 | ADDR | CMD | LEN | DATA[N] | CRC_L CRC_H（小端 CRC16）
 * 1ms 拼帧 → 5ms 执行（地址/CRC 在收帧入口过滤）→ 每 20ms 轮发一帧遥测。
 * 升级链路统一由 App_BootProto（ARM 0xD2 帧）处理，原 0x2F 握手已移除 */

#include "Includes.h"

UartProtoCtrl_t g_UartProto;

/* BMS 下发数据的暂存 */
static UartBmsSide_t s_bms;

/* Modbus CRC16 查表，多项式 0xA001，初值 0xFFFF */
static const uint16_t g_crc16_table[256] =
{
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/* 查表法：crc = (crc>>8) ^ table[(crc ^ byte) & 0xFF] */
static uint16_t Fast_CRC16(const uint16_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    for (i = 0U; i < len; i++)
    {
        crc = (uint16_t)((crc >> 8) ^
              g_crc16_table[(crc ^ (buf[i] & 0xFFU)) & 0xFFU]);
    }
    return crc;
}

/* 16 位数按小端拆成两个协议字节（只存 uint16 低 8 位） */
static void PutU16(uint16_t *p, uint16_t v)
{
    p[0] = (uint16_t)(v & 0xFFU);
    p[1] = (uint16_t)((v >> 8) & 0xFFU);
}

/* C2000 上 uint32 = 两个字，hi=0/1 取低/高字 */
static uint16_t U32Word(const uint32_t *p, uint16_t hi)
{
    const uint16_t *w = (const uint16_t *)p;
    return w[hi];
}

/* float 四舍五入并限幅 0~65535，供 0x51/0x52 上报 */
static uint16_t F32ToU16(float v)
{
    float t;

    if (v != v) { return 0U; }   /* NaN（自身比较恒假）：兜底 0，避免 (uint16_t)NaN 未定义结果 */

    t = v + ((v >= 0.0f) ? 0.5f : -0.5f);
    if (t > 65535.0f) { t = 65535.0f; }
    if (t < 0.0f)     { t = 0.0f; }
    return (uint16_t)t;
}

/* 组帧并整包写入 TX 环；长度超限或空指针直接返回 false */
bool UartProto_SendPacket(uint16_t addr, uint16_t cmd,
                          const uint16_t *pData, uint16_t len)
{
    uint16_t pack[PROTOCOL_FRAME_SIZE];
    uint16_t crc;
    uint16_t i;

    if ((len > PROTOCOL_MAX_PAYLOAD) || ((len != 0U) && (pData == 0)))
    {
        return false;
    }

    pack[0] = PROTOCOL_SOF1;
    pack[1] = PROTOCOL_SOF2;
    pack[2] = addr & 0xFFU;
    pack[3] = cmd & 0xFFU;
    pack[4] = len & 0xFFU;
    for (i = 0U; i < len; i++)
    {
        pack[5U + i] = pData[i] & 0xFFU;
    }
    /* CRC 覆盖帧头 5 字节 + 数据，低字节在前 */
    crc = Fast_CRC16(pack, (uint16_t)(5U + len));
    pack[5U + len] = (uint16_t)(crc & 0xFFU);
    pack[6U + len] = (uint16_t)((crc >> 8) & 0xFFU);

    return CustomUart_Write(pack, (uint16_t)(7U + len));   /* 整帧 7 + N 字节 */
}

/* 上电清零，状态机回到等 0xAA */
void UartProto_Init(void)
{
    memset(&g_UartProto, 0, sizeof(g_UartProto));
    memset(&s_bms, 0, sizeof(s_bms));
    g_UartProto.eRxState = eUartRxWaitSof1;
    g_UartProto.u32LastRxByteTick = Sys_GetTick1ms();
}

/* 遥测映射：命令码 + 4 个变量地址；g_CanMon 有空洞，不能整段 memcpy */
static const UartTxMap_t s_txMap[] =
{
    { 0x40U, { &g_CanMon.ADC.vbat_dV, &g_CanMon.ADC.vbusPcs_dV,
               &g_CanMon.ADC.vbus1_dV, &g_CanMon.ADC.vbus2_dV } },
    { 0x41U, { &g_CanMon.ADC.vmBus_dV, (const uint16_t *)&g_CanMon.ADC.sampleIr_cA,
               (const uint16_t *)&g_CanMon.ADC.iout1_cA, (const uint16_t *)&g_CanMon.ADC.iout2_cA } },
    { 0x42U, { (const uint16_t *)&g_CanMon.ADC.pout1_W, (const uint16_t *)&g_CanMon.ADC.pout2_W,
               (const uint16_t *)&g_CanMon.ADC.tempInductor_dC, (const uint16_t *)&g_CanMon.ADC.tempAmbient_dC } },
    { 0x43U, { (const uint16_t *)&g_CanMon.ADC.tempHeatsink_dC, (const uint16_t *)&g_CanMon.ADC.tempMos_dC,
               0, 0 } },
    { 0x44U, { &g_CanMon.Setpoint.voutSet_dV, (const uint16_t *)&g_CanMon.Setpoint.vbusDroop_dV,
               &g_CanMon.Setpoint.vbus1Ref_dV, &g_CanMon.Setpoint.vbus1Set_dV } },
    { 0x45U, { &g_CanMon.Setpoint.vbus1Cmd_dV, &g_CanMon.Setpoint.vbus2Ref_dV,
               &g_CanMon.Setpoint.vbus2Set_dV, &g_CanMon.Setpoint.vbus2Cmd_dV } },
    { 0x46U, { (const uint16_t *)&g_CanMon.Setpoint.iCharSet_cA, (const uint16_t *)&g_CanMon.Setpoint.ibus1ref_cA,
               (const uint16_t *)&g_CanMon.Setpoint.iReCharSet_cA, (const uint16_t *)&g_CanMon.Setpoint.ibus2ref_cA } },
    { 0x47U, { (const uint16_t *)&g_CanMon.Setpoint.droopCur_dv, (const uint16_t *)&g_CanMon.Setpoint.canAverCur_dv,
               &g_CanMon.PowerLimit.bmsCharPowSet_W, &g_CanMon.PowerLimit.bmsrecharPowSet_W } },
    { 0x48U, { &g_CanMon.PowerLimit.sysMaxChgPow_W, &g_CanMon.PowerLimit.sysMaxRechPow_W,
               &g_CanMon.PowerLimit.tempLimChar_W, &g_CanMon.PowerLimit.tempLimRech_W } },
    { 0x49U, { &g_CanMon.PowerLimit.ibatLimChar_cA, &g_CanMon.PowerLimit.vbatLimRech_W,
               &g_CanMon.Status.runFlag.all, &g_CanMon.Status.dabRunSta } },
    /* 0x4A/0x4C 是 32 位告警、0x51/0x52 是 float：表内 w 留空，发送时特判 */
    { 0x4AU, { 0, 0, 0, 0 } },
    { 0x4BU, { &g_CanMon.RealAlarm.fastfault, &g_CanMon.RealAlarm.alarm,
               &g_CanMon.SaveAlarm.savefastfault, &g_CanMon.SaveAlarm.savealarm } },
    { 0x4CU, { 0, 0, 0, 0 } },
    { 0x4DU, { (const uint16_t *)&g_CanMon.LoopCtrl.vLoop1_Err_cV, (const uint16_t *)&g_CanMon.LoopCtrl.vLoop1_Out,
               (const uint16_t *)&g_CanMon.LoopCtrl.vLoop2_Err_cV, (const uint16_t *)&g_CanMon.LoopCtrl.vLoop2_Out } },
    { 0x4EU, { &g_CanMon.LoopCtrl.vLoop1_DutyFf, &g_CanMon.LoopCtrl.vLoop2_DutyFf,
               (const uint16_t *)&g_CanMon.LoopCtrl.iLoop1_Err_cA, (const uint16_t *)&g_CanMon.LoopCtrl.iLoop1_Out } },
    { 0x4FU, { (const uint16_t *)&g_CanMon.LoopCtrl.iLoop2_Err_cA, (const uint16_t *)&g_CanMon.LoopCtrl.iLoop2_Out,
               &g_CanMon.LoopCtrl.iLoop1_DutyFf, &g_CanMon.LoopCtrl.iLoop2_DutyFf } },
    { 0x50U, { (const uint16_t *)&g_CanMon.LoopCtrl.vLoop_Ref_dV, (const uint16_t *)&g_CanMon.LoopCtrl.vLoop_Ff_cA,
               (const uint16_t *)&g_CanMon.LoopCtrl.iLoop1_Ff_cA, (const uint16_t *)&g_CanMon.LoopCtrl.iLoop2_Ff_cA } },
    { 0x51U, { 0, 0, 0, 0 } },
    { 0x52U, { 0, 0, 0, 0 } },
};

#define UART_TX_MAP_NUM  ((uint16_t)(sizeof(s_txMap) / sizeof(s_txMap[0])))

/* 按 s_txMap 轮发一帧；特殊命令在上文特判之外单独组装 */
static void UartProto_SendReport(void)
{
    uint16_t tx[9];
    uint16_t i;
    uint16_t idx = g_UartProto.u16TxCmdIdx;
    const UartTxMap_t *m;
    uint16_t cmd;

    if (idx >= UART_TX_MAP_NUM)
    {
        idx = 0U;
    }
    m = &s_txMap[idx];
    cmd = m->cmd;

    /* 默认 4 个字（小端）+ 1 字节保留 */
    for (i = 0U; i < 4U; i++)
    {
        PutU16(&tx[i * 2U], (m->w[i] != 0) ? (*m->w[i]) : 0U);
    }
    tx[8] = 0U;

    if (cmd == 0x40U)
    {
        tx[8] = 0x01U;   /* 数据有效 */
    }
    else if (cmd == 0x4AU)   /* 32 位实时告警拆成 4 个字 */
    {
        PutU16(&tx[0], U32Word(&g_CanMon.RealAlarm.slowfault1, 0U));
        PutU16(&tx[2], U32Word(&g_CanMon.RealAlarm.slowfault1, 1U));
        PutU16(&tx[4], g_CanMon.RealAlarm.alarm); /* 原 slowfault2 低字：alm */
        PutU16(&tx[6], 0U);                       /* 原 slowfault2 高字恒 0 */
    }
    else if (cmd == 0x4CU)   /* 锁存告警同理 */
    {
        PutU16(&tx[0], U32Word(&g_CanMon.SaveAlarm.saveslowfault1, 0U));
        PutU16(&tx[2], U32Word(&g_CanMon.SaveAlarm.saveslowfault1, 1U));
        PutU16(&tx[4], g_CanMon.SaveAlarm.savealarm);
        PutU16(&tx[6], 0U);
    }
    else if (cmd == 0x51U)   /* 4 个 float 转 uint16 */
    {
        PutU16(&tx[0], F32ToU16(g_CanMon.utest1));
        PutU16(&tx[2], F32ToU16(g_CanMon.utest2));
        PutU16(&tx[4], F32ToU16(g_CanMon.utest3));
        PutU16(&tx[6], F32ToU16(g_CanMon.utest4));
    }
    else if (cmd == 0x52U)   /* 另 4 个 float */
    {
        PutU16(&tx[0], F32ToU16(g_CanMon.utest5));
        PutU16(&tx[2], F32ToU16(g_CanMon.utest6));
        PutU16(&tx[4], F32ToU16(g_CanMon.utest7));
        PutU16(&tx[6], F32ToU16(g_CanMon.utest8));
    }

    if (UartProto_SendPacket(PROTOCOL_BROADCAST, cmd, tx, 9U) == false)
    {
        g_UartProto.u16TxDropCnt++;   /* TX 环满记丢帧 */
    }

    g_UartProto.u16TxCmdIdx = (uint16_t)(idx + 1U);
    if (g_UartProto.u16TxCmdIdx >= UART_TX_MAP_NUM)
    {
        g_UartProto.u16TxCmdIdx = 0U;
    }
}

/* 0x00 运行控制：
 * 0~1 SOC(0.1%) | 2~3 充电电流设定(0.1A) | 4~5 放电电流设定(0.1A)
 * 6 BMS 地址 | 7 控制位 | 8 运行模式 mode */
static void Cmd00_Run(const uint16_t *d)
{
    UnionSysCmd cmd;
    uint16_t ctrl;
    uint16_t mode;

    CfgParam.BatChgCurSet = d[1];
    CfgParam.BatDischgCurSet = d[2];
    s_bms.bmsAddr = d[3] & 0xFFU;          /* Byte6：BMS 地址 */

    /* ★ 首次有效分址也接受地址 0；变更地址时先撤掉未发出的旧 0x41，
     *   再换 uIndex / 清对端票，避免 CMD 缓冲里带着旧 LogADDR 发出去。 */
    if ((uMdlAddMes.uIndex != s_bms.bmsAddr) ||
        (uMdlAddMes.uCanState.bits.bCalIDCmd != ADDRcmd_end))
    {
        DabCan_CancelTx(CAN_TX_BUF_CMD);
        uMdlAddMes.uIndex = s_bms.bmsAddr;
        uMdlAddMes.uCanState.bits.bCalIDCmd = ADDRcmd_end;
        CurShare_ResetPeers();
    }
    ctrl          = (d[3] >> 8) & 0xFFU;   /* Byte7：控制位 */
    mode          = d[4];                 /* Byte8：运行模式 */

    /* Byte7 控制位（协议 4.1.1）：
     * bit0 onoff | bit1 HotCMD | bit2~3 ReCharForce | bit4 standby
     * bit5 phase | bit6 bClearFault | bit7 bStopReChar
     * SysCmd 先在局部变量拼好整字再一次写入：逐位域写是多次读改写，
     * 中间态可能被 2ms 状态机 / ZERO ISR 读到（如 onoff 已置 1 而放电使能未更新） */
    cmd.all               = CfgParam.SysCmd.all;   /* 保留本帧未涉及的位（CharDcdcOn/faultcmd 等） */
    cmd.bit.onoffcmd      = ctrl & 0x01U;
    s_bms.hotCmd          = (ctrl >> 1) & 0x01U;
    cmd.bit.ReCharForce   = (ctrl >> 2) & 0x03U;
    cmd.bit.standby       = (ctrl >> 4) & 0x01U;
    cmd.bit.clearfaultcmd = (ctrl >> 6) & 0x01U;
    cmd.bit.ReCharDcdcOn  = ((ctrl >> 7) & 0x01U) ? 0U : 1U;   /* bit7 禁止放电：1 → 关放电使能 */

    /* 运行模式 mode（协议 4.1.2）：
     * bit0 TestCmd | bit1 BmsMode | bit2 bClearFault | bit3 bAdi，其余保留填 0 */
    cmd.bit.openTest = mode & 0x0001U;
    if ((mode & 0x0004U) != 0U)   /* mode.bit2 清故障与 Byte7.bit6 任一为 1 即清 */
    {
        cmd.bit.clearfaultcmd = 1U;
    }
    CfgParam.SysCmd.all = cmd.all;   /* 单次 16 位写，原子生效 */

    /* 当拍同步 SysCmd，同帧 onoff=0+clear=1 时清故障能读到新 shutdown */
    SM_SyncSysCmd();
}

/* 0x05 并联/母线设定，LEN=9 */
static void Cmd05_PackBus(const uint16_t *d)
{
    const UartBmsBusFrame_t *p = (const UartBmsBusFrame_t *)d;

    s_bms.PackSum            = p->packSum;
    CfgParam.BusVolSet       = p->busVolSet_dV;
    CfgParam.BusDischgCurSet = p->busDischgCurSet_dA;
    CfgParam.OpenLoopDuty    = p->openLoopDuty;
}

uint16_t UartProto_GetPackSum(void)
{
    return s_bms.PackSum;
}

uint16_t UartProto_GetHotCmd(void)
{
    return s_bms.hotCmd;
}

/* 只处理 0x00 与 0x05；0x01~0x04 校准帧数据无使用方，与其余命令一样静默丢弃。 */
static void UartProto_HandleCmd(uint16_t cmd, const uint16_t *data)
{
    switch (cmd)
    {
    case PROTOCOL_CMD_RUN_0:          Cmd00_Run(data);     break;
    case PROTOCOL_CMD_SET_PACK_BUS_5: Cmd05_PackBus(data); break;
    default: break;
    }
}

/* 1ms 写帧队列；满则丢最旧帧保最新命令 */
static void UartProto_PushFrame(const uint16_t *pFrame)
{
    uint16_t wp = g_UartProto.u16RxWritePos;
    uint16_t np = (uint16_t)((wp + 1U) & UART_PROTO_RX_SLOT_MASK);
    uint16_t i;

    if (np == g_UartProto.u16RxReadPos)
    {
        /* 队列满：消费端（5ms）停滞说明旧命令已积压过期，
         * 丢最旧、保最新设定值，防止"执行旧命令、丢弃新命令"；
         * 与 PopFrame 同处主循环协作上下文，无并发 */
        g_UartProto.u16RxReadPos =
            (uint16_t)((g_UartProto.u16RxReadPos + 1U) & UART_PROTO_RX_SLOT_MASK);
        if (g_UartProto.u16RxFrameDropCnt < 60000U)
        {
            g_UartProto.u16RxFrameDropCnt++;
        }
    }

    for (i = 0U; i < PROTOCOL_RX_FRAME_SIZE; i++)
    {
        g_UartProto.rxFrameQueue[wp][i] = pFrame[i];
    }
    g_UartProto.u16RxWritePos  = np;
}

/* 5ms 读帧队列；空返回 false */
static bool UartProto_PopFrame(uint16_t *pBuf)
{
    uint16_t rp = g_UartProto.u16RxReadPos;
    uint16_t i;

    if (rp == g_UartProto.u16RxWritePos) { return false; }

    for (i = 0U; i < PROTOCOL_RX_FRAME_SIZE; i++)
    {
        pBuf[i] = g_UartProto.rxFrameQueue[rp][i];
    }
    g_UartProto.u16RxReadPos = (uint16_t)((rp + 1U) & UART_PROTO_RX_SLOT_MASK);
    return true;
}

/* 收帧状态机：等 AA → 等 55 → 收 3 字节头 → 收数据+CRC */
static void UartProto_RxByte(uint16_t byte)
{
    uint16_t b = byte & 0xFFU;
    uint16_t addr;
    uint16_t calc_crc;
    uint16_t recv_crc;

    switch (g_UartProto.eRxState)
    {
    case eUartRxWaitSof1:
        if (b == PROTOCOL_SOF1)
        {
            g_UartProto.rxFrameBuf[0] = b;
            g_UartProto.u16RxFramePos = 1U;
            g_UartProto.eRxState = eUartRxWaitSof2;
        }
        break;

    case eUartRxWaitSof2:
        if (b == PROTOCOL_SOF2)
        {
            g_UartProto.rxFrameBuf[1] = b;
            g_UartProto.u16RxFramePos = 2U;
            g_UartProto.eRxState = eUartRxHead;
        }
        /* 连续 AA 时重新对齐，不丢第二个 AA */
        else if (b == PROTOCOL_SOF1)
        {
            g_UartProto.rxFrameBuf[0] = b;
            g_UartProto.u16RxFramePos = 1U;
        }
        else
        {
            g_UartProto.eRxState = eUartRxWaitSof1;
            g_UartProto.u16RxFramePos = 0U;
        }
        break;

    case eUartRxHead:
        g_UartProto.rxFrameBuf[g_UartProto.u16RxFramePos++] = b;
        /* LEN 只在收帧入口校验一次，命令统一按 9 字节解析。 */
        if (g_UartProto.u16RxFramePos >= 5U)
        {
            if (b != PROTOCOL_BMS_PAYLOAD_LEN)
            {
                g_UartProto.eRxState = eUartRxWaitSof1;
                g_UartProto.u16RxFramePos = 0U;
            }
            else
            {
                g_UartProto.eRxState = eUartRxBody;
            }
        }
        break;

    case eUartRxBody:
        g_UartProto.rxFrameBuf[g_UartProto.u16RxFramePos++] = b;
        /* 收齐 16 字入队，状态机复位 */
        if (g_UartProto.u16RxFramePos >= PROTOCOL_RX_FRAME_SIZE)
        {
            /* 本机/广播且 CRC 正确的帧才占用队列槽位。 */
            calc_crc = Fast_CRC16(g_UartProto.rxFrameBuf, (uint16_t)(PROTOCOL_RX_FRAME_SIZE - 2U));
            recv_crc = (uint16_t)(g_UartProto.rxFrameBuf[PROTOCOL_RX_FRAME_SIZE - 2U] & 0xFFU) |
                       (uint16_t)((g_UartProto.rxFrameBuf[PROTOCOL_RX_FRAME_SIZE - 1U] & 0xFFU) << 8);
            addr = g_UartProto.rxFrameBuf[2] & 0xFFU;
            if (((addr == PROTOCOL_MY_ADDR) || (addr == PROTOCOL_BROADCAST)) &&
                (calc_crc == recv_crc))
            {
                UartProto_PushFrame(g_UartProto.rxFrameBuf);
            }

            g_UartProto.eRxState = eUartRxWaitSof1;
            g_UartProto.u16RxFramePos = 0U;
        }
        break;

    default:
        g_UartProto.eRxState = eUartRxWaitSof1;
        g_UartProto.u16RxFramePos = 0U;
        break;
    }
}

/* 1ms：rxRing → 逐字节拼帧；须排在 CustomUart_Task_Rx 之后 */
void UartProto_RxTask(void)
{
    uint16_t byte;

    /* 半帧超时 3ms：丢弃残帧，重新等 0xAA。
     * 必须在消费新字节之前判断：若新帧首字节先刷新 u32LastRxByteTick，
     * 残帧状态未复位就继续拼帧，会把新帧拼进旧帧（对齐 BootProto 的静默前置检查）。 */
    if (g_UartProto.eRxState != eUartRxWaitSof1)
    {
        if ((Sys_GetTick1ms() - g_UartProto.u32LastRxByteTick) >= UART_PROTO_FRAME_GAP_MS)
        {
            g_UartProto.eRxState = eUartRxWaitSof1;
            g_UartProto.u16RxFramePos = 0U;
        }
    }

    while (g_CustomUart.rxTail != g_CustomUart.rxHead)
    {
        byte = g_CustomUart.rxRing[g_CustomUart.rxTail] & 0xFFU;
        g_CustomUart.rxTail = (uint16_t)((g_CustomUart.rxTail + 1U) & CUSTOM_UART_RX_RING_MASK);
        g_UartProto.u32LastRxByteTick = Sys_GetTick1ms();
        g_UartProto.u16RxIdleCnt = 0U;
        UartProto_RxByte(byte);
    }
}

/* 一字节一格的帧数据 → 小端拼成 16 位字，供结构体指针访问 */
static void UartProto_PackData(const uint16_t *src, uint16_t *dst, uint16_t byteLen)
{
    uint16_t num = (uint16_t)((byteLen + 1U) / 2U);
    uint16_t i;
    uint16_t lo;
    uint16_t hi;

    for (i = 0U; i < num; i++)
    {
        lo = src[i * 2U] & 0xFFU;
        hi = ((uint16_t)(i * 2U + 1U) < byteLen) ? (src[i * 2U + 1U] & 0xFFU) : 0U;   /* 奇数长度时高字节补 0 */
        dst[i] = (uint16_t)(lo | (hi << 8));
    }
}

/* 5ms：取帧 → 执行命令；并轮发遥测。
 * 地址/CRC 已在收帧入口（UartProto_RxByte eUartRxBody）过滤后才入队，
 * 队列为协作上下文单生产者单消费者，出队侧无需复检。 */
void UartProto_Task_5ms(void)
{
    uint16_t buf[PROTOCOL_RX_FRAME_SIZE];
    uint16_t data[(PROTOCOL_BMS_PAYLOAD_LEN + 1U) / 2U];
    uint16_t cmd;
    bool runCmdReceived = false;

    while (UartProto_PopFrame(buf) == true)
    {
        cmd = buf[3] & 0xFFU;
        UartProto_PackData(&buf[5], data, PROTOCOL_BMS_PAYLOAD_LEN);
        UartProto_HandleCmd(cmd, data);
        if (cmd == PROTOCOL_CMD_RUN_0)
        {
            runCmdReceived = true;
        }
    }

    /* 接收和运行控制超时分别计数。 */
    if (g_UartProto.u16RxIdleCnt < 60000U)
    {
        g_UartProto.u16RxIdleCnt++;
    }
    if (runCmdReceived)
    {
        g_UartProto.u16RunCmdIdleCnt = 0U;
    }
    else if (g_UartProto.u16RunCmdIdleCnt < 60000U)
    {
        g_UartProto.u16RunCmdIdleCnt++;
    }

    /* 4×5ms 轮发一帧遥测 */
    g_UartProto.u16TelemetryTick++;
    if (g_UartProto.u16TelemetryTick >= UART_PROTO_TELEMETRY_TICKS)
    {
        g_UartProto.u16TelemetryTick = 0U;
        UartProto_SendReport();
    }
}
