/*************************************************************************************************
 * App_CurShare.c — 多机并联均流
 *
 * 0x41 每 81ms 广播方向/运行态/设定电流/实测电流。
 * 200ms：均流窗口用令牌法——收帧打票、算完销票，死机对端下一窗自动出局
 *        （81ms 发送 × 200ms 窗 = 2.47 帧/窗，任意窗口保底 2 帧，丢 1 帧仍有 1 帧）；
 *        同向在线模块按设定电流权重算 fAverCur，并积分慢速电压补偿。
 * 1ms：瞬时母线电流做下垂，发布 CurShare.voltAdj。
 * 0x42 接口保留，当前未接入 CAN 调度。
 *
 * 分工：FillTx41 地址锁定就发（待机/禁放/禁充仍广播，bit0=0 不参与均流）；
 *       AverCal / Update1ms 仅 CanRun 时动作。
 *************************************************************************************************/
#include "Includes.h"
#include "Interrupt_CtrlDab.h"       /* ISR 发布的充电发波状态（均流判据使用） */

/*------------------------------------------------------------------------------
 * 参数
 *----------------------------------------------------------------------------*/
/* 分段下垂（有符号电流，放正充负），限幅 ±8V；慢 trim 仍 ±10V
 *   |I|≤10A：Rd1=0.20 → 10A 时 2V（柔和，少互抢）
 *   10~20A：Rd2=0.60 → 再爬 6V，20A 满 8V（大功率快均流）
 *   折线连续：V(10)=Rd1·10，V(I)=V(10)+Rd2·(I-10) */
#define CUR_SHARE_DROOP_I_BREAK     (10.0f)
#define CUR_SHARE_DROOP_RD_LO       (0.20f)     /* Ω，≤10A */
#define CUR_SHARE_DROOP_RD_HI       (0.60f)     /* Ω，>10A */
#define CUR_SHARE_DROOP_MAX_V       (8.0f)
/* trim：I 环 @200ms；|err| 大时等效 Ki 加大（非线性积分，非经典 P） */
#define CUR_SHARE_TRIM_KI           (0.08f)     /* V/A / 200ms（0.08=0.04×2，保持原积分速率） */
#define CUR_SHARE_TRIM_KI_BOOST     (0.004f)    /* V/A²：×|err| */
#define CUR_SHARE_TRIM_MAX_V        (10.0f)     /* 电压修正 ±10V */
#define CUR_SHARE_I_MAX             (20.0f)
#define CUR_SHARE_ADJ_STEP_V        (0.05f)     /* V/1ms */
#define CUR_SHARE_AVER_DIV          (4U)        /* 50ms×4 = 200ms 均流窗口 */

#define CS_CLAMP(x, lo, hi)     (__fmin(__fmax((x), (lo)), (hi)))
#define CS_SLEW(now, tgt, step) ((now) + CS_CLAMP((tgt) - (now), -(step), (step)))

ST_CUR_SHARE CurShare;

/** 本机设定电流正数 [A] */
static float CurShare_LocalIsetPos(void)
{
    float x;

    if (RunState.workMode == (uint16_t)WORK_MODE_DISCHARGE)
    {
        x = RampTarget.iPos;
    }
    else if (RunState.workMode == (uint16_t)WORK_MODE_CHARGE)
    {
        x = g_stDabIsrState.f32ChgLimitA;  /* ★ 0x41 和本机权重共用有效充电能力 */
    }
    else
    {
        x = 0.0f;
    }
    return (x < 0.0f) ? -x : x;
}

/** 1=可均流控制：RUN + 允许运行 + 非换向 + 充/放合法且未禁该向 */
static uint16_t CurShare_CanRun(void)
{
    Uint16 mode = RunState.workMode;
    Uint16 seal = RunState.curLowSealing;

    /* ★ 均流参与判据同步实际封波，充电还需通过 PWM 发波门。 */
    if ((RunState.state != SM_STATE_RUN) || (SysCmd.bit.shutdown == 0U) ||
        FAULT_ANY() || (CfgParam.SysCmd.bit.faultcmd != 0U))
    {
        return 0U;
    }
    if ((mode == WORK_MODE_DISCHARGE) && ((seal & SM_SEAL_NO_DISCHARGE) == 0U))
    {
        return 1U;
    }
    if ((mode == WORK_MODE_CHARGE) && ((seal & SM_SEAL_NO_CHARGE) == 0U) &&
        (g_stDabIsrState.u16ChgPwmActive != 0U))
    {
        return 1U;
    }
    return 0U;
}

/** 0x41 状态字：bit0=可参与均流，bit1=放，bit2=充 */
static uint16_t CurShare_Status41(void)
{
    uint16_t st = 0U;

    if (RunState.workMode == (uint16_t)WORK_MODE_DISCHARGE)
    {
        st |= 2U;
    }
    else if (RunState.workMode == (uint16_t)WORK_MODE_CHARGE)
    {
        st |= 4U;
    }
    if (CurShare_CanRun() != 0U)
    {
        st |= 1U;   /* 待机/禁放/禁充：保持方向位，清 bit0 */
    }
    return st;
}

/*------------------------------------------------------------------------------
 * 补偿复位
 *----------------------------------------------------------------------------*/
void CurShare_ResetComp(void)
{
    CurShare.voltAdj = 0.0f;
    CurShare.droop   = 0.0f;
    CurShare.trim    = 0.0f;
    CurShare.trimOut = 0.0f;
    CurShare.workCount = 0U;   /* ★ 本机退出均流 */
    uMdlAddMes.fAverCur      = 0.0f;
}

/* ★ 本机地址变更时清除旧拓扑的对端样本和补偿。 */
void CurShare_ResetPeers(void)
{
    memset(uMdlAddMes.MDADDRID, 0, sizeof(uMdlAddMes.MDADDRID));
    CurShare_ResetComp();
}

/*==============================================================================
 * 接收
 *============================================================================*/

/**
 * 0x41 → 工程值；仅 bit0=1（对端可参与均流）时打票。
 * ★ 返回 1 表示已更新对端状态（含退出均流），0 表示拒收。
 */
uint16_t CurShare_Rx41(uint16_t u16Adr, const uint16_t *pu16Dat)
{
    uint16_t u16Raw;

    if ((pu16Dat == 0) || (u16Adr > (uint16_t)recnum) || (u16Adr == uMdlAddMes.uIndex))
    {
        return 0U;
    }

    uMdlAddMes.uRecMdl_onoff[u16Adr] = pu16Dat[1];
    /* pu16Dat[2]/[3] 为历史系数槽，本机发送固定 1，接收侧不再存 */

    u16Raw = (uint16_t)(((uint16_t)pu16Dat[4] << 8) | (pu16Dat[5] & 0xFFU));
    uMdlAddMes.uRecMdl_Curset[u16Adr] = (float)u16Raw * 0.01f;

    u16Raw = (uint16_t)(((uint16_t)pu16Dat[6] << 8) | (pu16Dat[7] & 0xFFU));
    uMdlAddMes.uRecMdl_Cur[u16Adr] = (float)u16Raw * 0.01f;

    /* ★ 令牌法打票：收帧即计；对端待机或禁向不发票。票据在 AverCal 算完销票。 */
    uMdlAddMes.MDADDRID[u16Adr] = ((pu16Dat[1] & 0x01U) != 0U) ? 1U : 0U;
    return 1U;
}

/**
 * 0x42：Vbat×100、主机标志；主机方向明确时写 RunState.canCmd 跟机
 * 接口保留，当前未接入 CAN 调度
 */
void CurShare_Rx42(uint16_t u16Adr, const uint16_t *pu16Dat)
{
    uint16_t u16ModeBits;

    if ((pu16Dat == 0) || (u16Adr > (uint16_t)recnum) || (u16Adr == uMdlAddMes.uIndex))
    {
        return;
    }

    /* 0x42 样本数组已从 MDADDRMES 删除；此处只解析主机跟机命令 */
    if (pu16Dat[4] == 0U)
    {
        return;
    }

    u16ModeBits = (uint16_t)(pu16Dat[1] & 0x06U);
    if ((u16ModeBits == 2U) && (RunState.workMode != (uint16_t)WORK_MODE_DISCHARGE))
    {
        RunState.canCmd = CAN_CMD_DISCHARGE;
    }
    else if ((u16ModeBits == 4U) && (RunState.workMode != (uint16_t)WORK_MODE_CHARGE))
    {
        RunState.canCmd = CAN_CMD_CHARGE;
    }
    else
    {
        RunState.canCmd = CAN_CMD_NONE;
    }
}

/*==============================================================================
 * 发送组包
 *============================================================================*/

/**
 * 0x41：地址锁定即发。待机/禁放/禁充仍广播，bit0=0 告知对端勿计入均流
 */
uint16_t CurShare_FillTx41(uint16_t *pu16Dat)
{
    uint16_t u16Raw;
    uint16_t st;
    float    f32Val;

    /* ★ 均流地址须与接收端的表下标范围一致。 */
    if ((pu16Dat == 0) || (uMdlAddMes.bCalIDCmd != ADDRcmd_end) ||
        (uMdlAddMes.uIndex > (uint16_t)recnum))
    {
        return 0U;
    }

    st = CurShare_Status41();
    pu16Dat[0] = 0x41U;
    pu16Dat[1] = st & 0xFFU;
    pu16Dat[2] = 1U;
    pu16Dat[3] = 1U;

    f32Val = CurShare_LocalIsetPos();
    u16Raw = (uint16_t)(f32Val * 100.0f);
    pu16Dat[4] = (u16Raw >> 8) & 0xFFU;
    pu16Dat[5] = u16Raw & 0xFFU;

    f32Val = stADC.BusCurr1.Filt.f32Filt100ms + stADC.BusCurr2.Filt.f32Filt100ms;
    f32Val = __fmax(((st & 0x06U) == 4U) ? -f32Val : f32Val, 0.0f);
    u16Raw = (uint16_t)(f32Val * 100.0f);
    pu16Dat[6] = (u16Raw >> 8) & 0xFFU;
    pu16Dat[7] = u16Raw & 0xFFU;

    return 1U;
}

/**
 * 0x42：[0]=0x42 [1]=状态 [2][3]=Vbat×100 [4]=主机 [5]=台数 [6]bit7=在跑 [7]=0
 * 接口保留，当前未接入 CAN 调度
 */
uint16_t CurShare_FillTx42(uint16_t *pu16Dat)
{
    uint16_t u16Raw;

    if ((pu16Dat == 0) || (uMdlAddMes.bCalIDCmd != ADDRcmd_end))
    {
        return 0U;
    }

    pu16Dat[0] = 0x42U;

    u16Raw = 1U;   /* bit0 固定 1（帧有效）；方向另或 */
    if (RunState.workMode == (uint16_t)WORK_MODE_DISCHARGE)
    {
        u16Raw |= 2U;
    }
    else if (RunState.workMode == (uint16_t)WORK_MODE_CHARGE)
    {
        u16Raw |= 4U;
    }
    pu16Dat[1] = u16Raw & 0xFFU;

    u16Raw = (uint16_t)(stADC.BatVolt.Filt.f32Filt10ms * 100.0f);
    pu16Dat[2] = (u16Raw >> 8) & 0xFFU;
    pu16Dat[3] = u16Raw & 0xFFU;

    pu16Dat[4] = RunState.isMaster & 0xFFU;
    pu16Dat[5] = UartProto_GetPackSum() & 0xFFU;
    pu16Dat[6] = ((SysCmd.bit.shutdown != 0U) && (RunState.state == SM_STATE_RUN)) ? 0x80U : 0U;
    pu16Dat[7] = 0U;

    return 1U;
}

/*==============================================================================
 * 周期任务
 *============================================================================*/

/**
 * ★ 均流平均（令牌法）：只统计本窗口内收到过帧的对端，同向计入 Σ；
 *   销票后对端必须在新窗口重新收帧，死机对端自然出局，无需超时检查。
 */
static void CurShare_AverCal(void)
{
    uint16_t i;
    uint16_t want;   /* 2=放，4=充 */
    uint16_t need;   /* 在跑|同向 */
    uint16_t peerCount = 0U;
    float    sumI = 0.0f;
    float    sumSet = 0.0f;
    float    iset = CurShare_LocalIsetPos();
    float    ibus;
    float    lim;
    float    err;

    uMdlAddMes.fAverCur = 0.0f;

    if (RunState.workMode == WORK_MODE_DISCHARGE)
    {
        want = 2U;
        lim  = Ramp.iPos;
    }
    else
    {
        want = 4U;
        lim  = iset;  /* ★ 充电目标使用同一能力快照限幅 */
    }

    need = (uint16_t)(want | 1U);
    for (i = 0U; i <= (uint16_t)recnum; i++)
    {
        if (uMdlAddMes.MDADDRID[i] == 0U)
        {
            continue;
        }
        /* ★ 见票即销：本窗数据只用一次，下一窗须重新收帧；
         *   存活对端 81ms 内必然重新打票（含本机封波/换向期间），
         *   死机对端下一窗自动出局。 */
        uMdlAddMes.MDADDRID[i] = 0U;
        if ((i == uMdlAddMes.uIndex) ||
            ((uMdlAddMes.uRecMdl_onoff[i] & 0x07U) != need) ||
            (uMdlAddMes.uRecMdl_Curset[i] <= 0.0f))
        {
            continue;
        }
        peerCount++;  /* ★ 与均流加权共用同一组有效对端 */
        sumSet += uMdlAddMes.uRecMdl_Curset[i];
        sumI   += uMdlAddMes.uRecMdl_Cur[i];
    }

    if (CurShare_CanRun() == 0U)
    {
        CurShare_ResetComp();
        return;
    }

    /* ★ 本机可运行时先计入本机，再发布有效均流台数。 */
    CurShare.workCount = 1U;
    if ((uMdlAddMes.bCalIDCmd != ADDRcmd_end) ||
        (uMdlAddMes.uIndex > (uint16_t)recnum) ||
        (peerCount == 0U) || (iset <= 0.0f) || (lim <= 0.0f))
    {
        CurShare.trim = 0.0f;
        return;
    }
    CurShare.workCount += peerCount;

    /* ibus 有符号：放为正、充为负；Σ 与 fAverCur 仍用幅值 */
    ibus = stADC.BusCurr1.Filt.f32Filt100ms + stADC.BusCurr2.Filt.f32Filt100ms;
    {
        float ibusMag = (want == 2U) ? ibus : -ibus;
        float iRef;
//        float eAbs;
//        float ki;

        if (ibusMag < 0.0f)
        {
            ibusMag = 0.0f;
        }

        uMdlAddMes.fAverCur = __fmin(__divf32(sumI + ibusMag, sumSet + iset) * iset, lim);

        /* err = I*有符号 − Ibus；非线性 I + ±10V */
        iRef = (want == 2U) ? uMdlAddMes.fAverCur : -uMdlAddMes.fAverCur;
        err  = iRef - ibus;
//        eAbs = (err >= 0.0f) ? err : -err;
//        if (eAbs > CUR_SHARE_I_MAX)
//        {
//            eAbs = CUR_SHARE_I_MAX;
//        }
//        ki = CUR_SHARE_TRIM_KI + CUR_SHARE_TRIM_KI_BOOST * eAbs;
        CurShare.trim += CUR_SHARE_TRIM_KI * err;
        CurShare.trim  = __fmin(__fmax(CurShare.trim, -CUR_SHARE_TRIM_MAX_V),
                               CUR_SHARE_TRIM_MAX_V);
    }
}

/** 50ms 入口：分频到 200ms 算均流（对齐 0x41 的 81ms 发送） */
void CurShare_Task_50ms(void)
{
    static uint16_t s_div;

    s_div++;
    if (s_div < CUR_SHARE_AVER_DIV)
    {
        return;
    }
    s_div = 0U;
    CurShare_AverCal();
}

/** 分段下垂：小电流柔、大电流陡，输出有符号 [V] */
static float CurShare_DroopFromI(float ibus)
{
    float iAbs = (ibus >= 0.0f) ? ibus : -ibus;
    float vAbs;

    if (iAbs <= CUR_SHARE_DROOP_I_BREAK)
    {
        vAbs = CUR_SHARE_DROOP_RD_LO * iAbs;
    }
    else
    {
        vAbs = CUR_SHARE_DROOP_RD_LO * CUR_SHARE_DROOP_I_BREAK +
               CUR_SHARE_DROOP_RD_HI * (iAbs - CUR_SHARE_DROOP_I_BREAK);
    }
    vAbs = CS_CLAMP(vAbs, 0.0f, CUR_SHARE_DROOP_MAX_V);
    return (ibus >= 0.0f) ? vAbs : -vAbs;
}

/** 限幅后按 ADJ_STEP 向目标爬坡 */
static void CurShare_SlewTo(float *pVal, float f32Tgt, float f32Min, float f32Max)
{
    f32Tgt = CS_CLAMP(f32Tgt, f32Min, f32Max);
    *pVal  = CS_SLEW(*pVal, f32Tgt, CUR_SHARE_ADJ_STEP_V);
    *pVal  = CS_CLAMP(*pVal, f32Min, f32Max);
}

/**
 * 1ms：下垂 + trim → voltAdj
 * @param f32AdjMin  voltAdj 下限 = Vmin − Vref（通常为负）
 * @param f32AdjMax  voltAdj 上限 = Vmax − Vref（通常为正）
 * voltAdj = trimOut − droop，且须落在 [AdjMin, AdjMax] 内保母线不越界
 */
void CurShare_Update1ms(float f32AdjMin, float f32AdjMax)
{
    float ibus;
    float droopTgt = 0.0f;
    float droopMin;
    float droopMax;
    float trimMin;
    float trimMax;

    if (CurShare_CanRun() == 0U)
    {
        CurShare_ResetComp();
        return;
    }

    ibus = stADC.BusCurr1.f32IsrResult + stADC.BusCurr2.f32IsrResult;
    /* ★ 只在有同向工作对端时下垂，最后一台退出后按原斜率回零。 */
    if (CurShare.workCount > 1U)
    {
        droopTgt = CurShare_DroopFromI(ibus);
    }

    /* voltAdj≈−droop（trim=0 时）→ droop 允许区间是 Adj 的镜像 */
    droopMin = -f32AdjMax;
    droopMax = -f32AdjMin;
    CurShare_SlewTo(&CurShare.droop, droopTgt, droopMin, droopMax);

    /* trimOut = voltAdj + droop，剩余裕度叠在 droop 上 */
    trimMin = f32AdjMin + CurShare.droop;
    trimMax = f32AdjMax + CurShare.droop;
    CurShare.trim = CS_CLAMP(CurShare.trim, trimMin, trimMax);
    CurShare_SlewTo(&CurShare.trimOut, CurShare.trim, trimMin, trimMax);

    CurShare.voltAdj = CurShare.trimOut - CurShare.droop;
}
