/* App_UartProto.c — P550 串口协议 V1.3
 * 帧：AA 55 | ADDR | CMD | LEN | DATA[N] | CRC_L CRC_H（小端 CRC16）
 * 1ms 驱动 FIFO 直递拼帧 + 校验 + 当拍执行（字节同步喂 Boot）→ 5ms 只做遥测轮发。
 * 升级链路统一由 App_BootProto（ARM 0xD2 帧）处理，原 0x2F 握手已移除 */

#include "Includes.h"

ST_UART_PROTO_CTRL stUartProto;   /* 类型与 extern 声明见 App_UartProto.h（分区：常量/类型/变量/API） */

/* BMS 侧暂存（类型见 .h） */
static ST_UART_BMS_DATA sstBmsData;

/* Modbus CRC16 查表，多项式 0xA001，初值 0xFFFF */
static const uint16_t su16Crc16Table[256] =
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
static uint16_t sFastCrc16(const uint16_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    for (i = 0U; i < len; i++)
    {
        crc = (uint16_t)((crc >> 8) ^
              su16Crc16Table[(crc ^ (buf[i] & 0xFFU)) & 0xFFU]);
    }
    return crc;
}

/* 16 位数按小端拆成两个协议字节（只存 uint16 低 8 位） */
static void sPutU16(uint16_t *p, uint16_t v)
{
    p[0] = (uint16_t)(v & 0xFFU);
    p[1] = (uint16_t)((v >> 8) & 0xFFU);
}

/* float 四舍五入并限幅 0~65535，供 0x51/0x52 上报 */
static uint16_t sF32ToU16(float v)
{
    float t;

    if (v != v) { return 0U; }   /* NaN（自身比较恒假）：兜底 0，避免 (uint16_t)NaN 未定义结果 */

    t = v + ((v >= 0.0f) ? 0.5f : -0.5f);
    if (t > 65535.0f) { t = 65535.0f; }
    if (t < 0.0f)     { t = 0.0f; }
    return (uint16_t)t;
}

/* 组帧并整包写入 TX 环；载荷上限 24 由 TX 环容量决定（可用 31 字节 - 帧开销 7），
 * 超限或空指针直接返回 false */
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
    crc = sFastCrc16(pack, (uint16_t)(5U + len));
    pack[5U + len] = (uint16_t)(crc & 0xFFU);
    pack[6U + len] = (uint16_t)((crc >> 8) & 0xFFU);

    return CustomUart_Write(pack, (uint16_t)(7U + len));   /* 整帧 7 + N 字节 */
}

/* 上电清零，收帧位置回到等 0xAA；两个时间戳取当前时刻，给满额启动宽限 */
void UartProto_Init(void)
{
    memset(&stUartProto, 0, sizeof(stUartProto));
    memset(&sstBmsData, 0, sizeof(sstBmsData));
    stUartProto.u32LastRxByteTick = Sys_GetTick1ms();
    stUartProto.u32LastCmdTick = Sys_GetTick1ms();
}

/* 遥测映射：命令码 + 4 个变量地址；g_CanMon 有空洞，不能整段 memcpy */
static const ST_UART_TX_MAP sstTxMap[] =
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
    /* 0x4A/0x4C 是 32 位告警、0x51/0x52 是 float：表内 pu16Data 留空，发送时特判 */
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

#define UART_TX_MAP_NUM  ((uint16_t)(sizeof(sstTxMap) / sizeof(sstTxMap[0])))

/* 按 sstTxMap 轮发一帧；特殊命令在上文特判之外单独组装 */
static void sSendReport(void)
{
    uint16_t tx[9];
    uint16_t i;
    uint16_t idx = stUartProto.u16TxCmdIdx;
    const ST_UART_TX_MAP *m;
    uint16_t cmd;

    if (idx >= UART_TX_MAP_NUM)
    {
        idx = 0U;
    }
    m = &sstTxMap[idx];
    cmd = m->u16Cmd;

    /* 默认 4 个字（小端）+ 1 字节保留 */
    for (i = 0U; i < 4U; i++)
    {
        sPutU16(&tx[i * 2U], (m->pu16Data[i] != 0) ? (*m->pu16Data[i]) : 0U);
    }
    tx[8] = 0U;

    if (cmd == 0x40U)
    {
        tx[8] = 0x01U;   /* 数据有效 */
    }
    else if (cmd == 0x4AU)   /* 32 位实时告警拆成 4 个字 */
    {
        const uint32_t slowFault = g_CanMon.RealAlarm.slowfault1;
        sPutU16(&tx[0], (uint16_t)slowFault);
        sPutU16(&tx[2], (uint16_t)(slowFault >> 16U));
        sPutU16(&tx[4], g_CanMon.RealAlarm.alarm); /* 原 slowfault2 低字：alm */
        sPutU16(&tx[6], 0U);                       /* 原 slowfault2 高字恒 0 */
    }
    else if (cmd == 0x4CU)   /* 锁存告警同理 */
    {
        const uint32_t slowFault = g_CanMon.SaveAlarm.saveslowfault1;
        sPutU16(&tx[0], (uint16_t)slowFault);
        sPutU16(&tx[2], (uint16_t)(slowFault >> 16U));
        sPutU16(&tx[4], g_CanMon.SaveAlarm.savealarm);
        sPutU16(&tx[6], 0U);
    }
    else if (cmd == 0x51U)   /* 4 个 float 转 uint16 */
    {
        sPutU16(&tx[0], sF32ToU16(g_CanMon.utest1));
        sPutU16(&tx[2], sF32ToU16(g_CanMon.utest2));
        sPutU16(&tx[4], sF32ToU16(g_CanMon.utest3));
        sPutU16(&tx[6], sF32ToU16(g_CanMon.utest4));
    }
    else if (cmd == 0x52U)   /* 另 4 个 float */
    {
        sPutU16(&tx[0], sF32ToU16(g_CanMon.utest5));
        sPutU16(&tx[2], sF32ToU16(g_CanMon.utest6));
        sPutU16(&tx[4], sF32ToU16(g_CanMon.utest7));
        sPutU16(&tx[6], sF32ToU16(g_CanMon.utest8));
    }

    if (UartProto_SendPacket(PROTOCOL_BROADCAST, cmd, tx, 9U) == false)
    {
        stUartProto.u16TxDropCnt++;   /* TX 环满记丢帧 */
    }

    stUartProto.u16TxCmdIdx = (uint16_t)(idx + 1U);
    if (stUartProto.u16TxCmdIdx >= UART_TX_MAP_NUM)
    {
        stUartProto.u16TxCmdIdx = 0U;
    }
}

/* 0x00 运行控制：
 * 0~1 SOC(0.1%) | 2~3 充电电流设定(0.1A) | 4~5 放电电流设定(0.1A)
 * 6 BMS 地址 | 7 控制位 | 8 运行模式 mode */
static void sCmd00Run(const uint16_t *d)
{
    UnionSysCmd cmd;
    uint16_t ctrl;
    uint16_t mode;
    uint16_t bmsAddr;

    sstBmsData.u16SocDPct = d[0];                 /* Byte0~1：SOC 0.1% */
    CfgParam.BatChgCurSet = d[1];
    CfgParam.BatDischgCurSet = d[2];
    bmsAddr = d[3] & 0xFFU;                /* Byte6：BMS 地址（仅本函数使用） */

    /* ★ 首次有效分址也接受地址 0；变更时撤旧 0x41 → 换址 → ResetPeers（见 CanComm_SetLocalAddr）。 */
    CanComm_SetLocalAddr(bmsAddr);
    ctrl          = (d[3] >> 8) & 0xFFU;   /* Byte7：控制位 */
    mode          = d[4];                 /* Byte8：运行模式 */

    /* Byte7 控制位（协议 4.1.1）：
     * bit0 onoff | bit1 HotCMD | bit2~3 ReCharForce | bit4 standby
     * bit5 phase | bit6 bClearFault | bit7 bStopReChar
     * SysCmd 先在局部变量拼好整字再一次写入：逐位域写是多次读改写，
     * 中间态可能被 2ms 状态机 / ZERO ISR 读到（如 onoff 已置 1 而放电使能未更新） */
    cmd.all               = CfgParam.SysCmd.all;   /* 保留本帧未涉及的位（CharDcdcOn/faultcmd 等） */
    cmd.bit.onoffcmd      = ctrl & 0x01U;
    sstBmsData.u16HotCmd          = (ctrl >> 1) & 0x01U;
    cmd.bit.ReCharForce   = (ctrl >> 2) & 0x03U;
    cmd.bit.standby       = (ctrl >> 4) & 0x01U;
    sstBmsData.u16Phase           = (ctrl >> 5) & 0x01U;   /* 预留：0单相 1三相 */
    cmd.bit.clearfaultcmd = (ctrl >> 6) & 0x01U;
    cmd.bit.ReCharDcdcOn  = ((ctrl >> 7) & 0x01U) ? 0U : 1U;   /* bit7 禁止放电：1 → 关放电使能 */

    /* 运行模式 mode（协议 4.1.2）：
     * bit0 TestCmd | bit1 BmsMode | bit2 bClearFault | bit3 bAdj | bit4 upboot，其余保留填 0 */
    cmd.bit.openTest = mode & 0x0001U;
    sstBmsData.u16BmsMode    = (mode >> 1) & 0x0001U;
    if ((mode & 0x0004U) != 0U)   /* mode.bit2 清故障与 Byte7.bit6 任一为 1 即清 */
    {
        cmd.bit.clearfaultcmd = 1U;
    }
    sstBmsData.u16AdiMode       = (mode >> 3) & 0x0001U;
    CfgParam.SysCmd.all = cmd.all;   /* 单次 16 位写，原子生效 */

    /* bit4 upboot=1：进纯 Boot 接收态（关加热膜 + 封波）；满 30s 未跳 Boot 由 BootProto 超时退出 */
    if ((mode & 0x0010U) != 0U)
    {
        BootProto_EnterBootMode();
    }

    /* 当拍同步 SysCmd，同帧 onoff=0+clear=1 时清故障能读到新 shutdown */
    SM_SyncSysCmd();
}

/* 校准帧：前 8 字节是 k/b，最后 1 字节保留。 */
static void sCmdCalStore(const uint16_t *d, ST_UART_BMS_CAL_FRAME *dst)
{
    dst->u16K1   = d[0];
    dst->i16B1   = (int16_t)d[1];
    dst->u16K2   = d[2];
    dst->i16B2   = (int16_t)d[3];
    dst->u16Rsvd = d[4];
}

/* 0x05 并联/母线设定，LEN=9：packSum | busVolSet_dV | busDischgCurSet_dA | openLoopDuty | 保留 */
static void sCmd05PackBus(const uint16_t *d)
{
    sstBmsData.u16PackSum            = d[0];
    CfgParam.BusVolSet       = d[1];
    CfgParam.BusDischgCurSet = d[2];
    CfgParam.OpenLoopDuty    = d[3];
}

uint16_t UartProto_GetPackSum(void)
{
    return sstBmsData.u16PackSum;
}

uint16_t UartProto_GetHotCmd(void)
{
    return sstBmsData.u16HotCmd;
}

void UartProto_ClearHotCmd(void)
{
    sstBmsData.u16HotCmd = 0U;
}

/* 0x00 运行控制 | 0x01~0x04 校准存储 | 0x05 并联/母线设定；其余命令静默丢弃 */
static void sHandleCmd(uint16_t cmd, const uint16_t *data)
{
    switch (cmd)
    {
    case PROTOCOL_CMD_RUN_0:           sCmd00Run(data);                         break;
    case PROTOCOL_CMD_CAL_BUS12_1:     sCmdCalStore(data, &sstBmsData.stCal.stBus12);    break;
    case PROTOCOL_CMD_CAL_PCS_I1_2:    sCmdCalStore(data, &sstBmsData.stCal.stBusPcsI1); break;
    case PROTOCOL_CMD_CAL_I2_ITOT_3:   sCmdCalStore(data, &sstBmsData.stCal.stI2Itot);   break;
    case PROTOCOL_CMD_CAL_VBAT_IBAT_4: sCmdCalStore(data, &sstBmsData.stCal.stVbatIbat);  break;
    case PROTOCOL_CMD_SET_PACK_BUS_5:  sCmd05PackBus(data);                      break;
    default: break;
    }
}

/* 收帧位置：0=等 AA，1=等 55，2~4=头，5~15=数据和 CRC。
 * 收齐 16 字节并校验通过后，当拍执行有效命令。
 * 长度或 CRC 错误时丢弃当前帧，从下一个字节重新寻找帧头。
 * 入口统一规范化低 8 位；缓冲只存规范化字节，解包不再重复 &0xFF。 */
static void sRxByte(uint16_t byte)
{
    uint16_t pos = stUartProto.u16RxFramePos;
    uint16_t addr;
    uint16_t cmd;
    uint16_t calc_crc;
    uint16_t recv_crc;
    uint16_t data[(PROTOCOL_BMS_PAYLOAD_LEN + 1U) / 2U];
    uint16_t i;

    byte &= 0xFFU;

    if ((pos == 0U) && (byte != PROTOCOL_SOF1))
    {
        return;
    }

    stUartProto.u16RxFrameBuf[pos] = byte;
    pos++;

    if (pos == 2U)   /* 第二字节：必须 0x55；连续 AA 重新对齐不丢第二个 AA；其余作废 */
    {
        if (byte == PROTOCOL_SOF1)
        {
            pos = 1U;
        }
        else if (byte != PROTOCOL_SOF2)
        {
            pos = 0U;
        }
    }
    else if (pos == 5U)   /* 头收齐：LEN 只在入口校验一次，命令统一按 9 字节解析 */
    {
        if (byte != PROTOCOL_BMS_PAYLOAD_LEN)
        {
            if (byte == PROTOCOL_SOF1)
            {
                stUartProto.u16RxFrameBuf[0] = PROTOCOL_SOF1;
                pos = 1U;
            }
            else
            {
                pos = 0U;
            }
        }
    }
    else if (pos >= PROTOCOL_RX_FRAME_SIZE)   /* 16 字收齐：校验 + 当拍执行 */
    {
        calc_crc = sFastCrc16(stUartProto.u16RxFrameBuf, (uint16_t)(PROTOCOL_RX_FRAME_SIZE - 2U));
        recv_crc = (uint16_t)(stUartProto.u16RxFrameBuf[PROTOCOL_RX_FRAME_SIZE - 2U] |
                              (stUartProto.u16RxFrameBuf[PROTOCOL_RX_FRAME_SIZE - 1U] << 8));
        addr = stUartProto.u16RxFrameBuf[2];
        cmd  = stUartProto.u16RxFrameBuf[3];
        /* 本机/广播、CRC 正确且命令有处理动作（0x00~0x05 均有对应分支）才执行；
         * 其余命令静默丢弃，不占任何资源 */
        if (((addr == PROTOCOL_MY_ADDR) || (addr == PROTOCOL_BROADCAST)) &&
            (calc_crc == recv_crc) &&
            (cmd <= PROTOCOL_CMD_SET_PACK_BUS_5))
        {
            /* 数据区 9 字节（一字节一格）→ 小端拼 5 个 16 位字，奇数尾字高字节补 0 */
            for (i = 0U; i < (uint16_t)((PROTOCOL_BMS_PAYLOAD_LEN - 1U) / 2U); i++)
            {
                data[i] = (uint16_t)(stUartProto.u16RxFrameBuf[5U + (i * 2U)] |
                                     (stUartProto.u16RxFrameBuf[6U + (i * 2U)] << 8));
            }
            data[(PROTOCOL_BMS_PAYLOAD_LEN - 1U) / 2U] = stUartProto.u16RxFrameBuf[13];

            sHandleCmd(cmd, data);
            if (cmd == PROTOCOL_CMD_RUN_0)
            {
                stUartProto.u32LastCmdTick = Sys_GetTick1ms();   /* SCI 掉线判据在此刷新 */
            }
        }
        pos = 0U;
    }

    stUartProto.u16RxFramePos = pos;
}

/* 最近有效 0x00 距今毫秒数：App_Fault 以 FAULT_SCI_TIMEOUT_N×5ms 判 SCI 掉线 */
uint32_t UartProto_CmdIdleMs(void)
{
    return (uint32_t)(Sys_GetTick1ms() - stUartProto.u32LastCmdTick);
}

/* 1ms：驱动 FIFO 直递拼帧；同一字节流同步喂 Boot 升级协议。
 * Task_1ms 中 BootProto_Task_Rx（静默断帧检查）必须排在本函数之前。 */
void UartProto_RxTask(void)
{
    uint16_t byte;

    /* 半帧超时 3ms：丢弃残帧，重新等 0xAA。
     * 必须在消费新字节之前判断：若新帧首字节先刷新 u32LastRxByteTick，
     * 残帧状态未复位就继续拼帧，会把新帧拼进旧帧（对齐 BootProto 的静默前置检查）。 */
    if (stUartProto.u16RxFramePos != 0U)
    {
        if ((Sys_GetTick1ms() - stUartProto.u32LastRxByteTick) >= UART_PROTO_FRAME_GAP_MS)
        {
            stUartProto.u16RxFramePos = 0U;
        }
    }

    /* SCI 空闲钳位：空闲超过 60s 后把 u32LastCmdTick 钉在 now-60s，
     * 使 CmdIdleMs 恒为有界值、不跨 32 位回卷点（回卷周期 49.7 天，
     * 连续断联满一个周期会使差值回卷到 0，10s 内误判 SCI 恢复）。
     * 正常运行 0x00 间隔远小于 60s，本分支不触发，10s 掉线判据行为不变。 */
    if ((Sys_GetTick1ms() - stUartProto.u32LastCmdTick) > UART_PROTO_IDLE_CAP_MS)
    {
        stUartProto.u32LastCmdTick = Sys_GetTick1ms() - UART_PROTO_IDLE_CAP_MS;
    }

    /* 驱动错误检查/恢复：出错本拍丢弃整段 FIFO（含错误字节前已收到的字节），交由上位机重发；
     * 返回 true 时立即作废在收半帧——残帧由已被丢弃的字节拼成，若保留，
     * 紧随重发的首字节会拼进旧残帧造成错位（重发紧随时无 3ms 静默间隔可依赖） */
    if (CustomUart_RxService())
    {
        stUartProto.u16RxFramePos = 0U;
        BootProto_ResetRx();
    }

    /* 硬件 FIFO 逐字节路由：收到 upboot=1 前只进 AA55 协议，之后只进 Boot 协议，彻底解耦 */
    while (CustomUart_PopRxByte(&byte))
    {
        stUartProto.u32LastRxByteTick = Sys_GetTick1ms();
        if (stBootProto.u16BootModeEn != 0U)
        {
            BootProto_RxByte(byte);
        }
        else
        {
            sRxByte(byte);
        }
    }
}

/* 5ms：4×5ms 轮发一帧遥测；升级标志有效期间暂停发送，避免残余业务帧干扰 Boot 握手 */
void UartProto_Task_5ms(void)
{
    if (stBootProto.u16FlashUpdateFlag != 0U)
    {
        return;
    }

    stUartProto.u16TelemetryTick++;
    if (stUartProto.u16TelemetryTick >= PROTOCOL_TELEMETRY_DIV)
    {
        stUartProto.u16TelemetryTick = 0U;
        sSendReport();
    }
}
