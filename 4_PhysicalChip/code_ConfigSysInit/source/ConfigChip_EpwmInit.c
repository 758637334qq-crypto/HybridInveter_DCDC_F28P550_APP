/*************************************************************************************************
 * ConfigChip_EpwmInit.c —— DAB PWM（最新硬件双桥）
 * 发波逻辑沿用原工程：1A=S1 1B=S2 | 2A=S3 2B=S4 | 3A=S5 3B=S6 | 4A=S7 4B=S8
 * 硬件映射见 ConfigChip_DabPwmMap.h：桥1=EPWM10/12/11/3，桥2=EPWM7/9/2/4 滞后 90°
 *************************************************************************************************/
#include "Includes.h"
#include "ConfigChip_DabPwmMap.h"

volatile Uint16 g_u16DabPwmTicks100us = 5U;

/*------------------------------------------------------------------------------------------------*/
/** @brief 单模块底座：时基/影子CMP/强制低/GLD
 *  @param selfLink   本模块 CMP 链接目标（桥内自链，禁止跨桥）
 *  @param gldMaster  GLDCTL2 主机：桥1=EPWM10，桥2=EPWM7（分开装载）
 */
static void SetupBase(volatile struct EPWM_REGS *p, Uint16 selfLink, Uint16 gldMaster)
{
    p->TZCTL.bit.TZA = TZ_FORCE_LO;
    p->TZCTL.bit.TZB = TZ_FORCE_LO;
    p->TZFRC.bit.OST = 1U;

    p->GLDCTL.all = 0U;
    p->GLDCFG.all = 0U;
    p->GLDCTL2.all = 0U;

    p->TBCTL.all = 0U;
    p->TBCTL.bit.CTRMODE = TB_FREEZE;
    p->TBCTL.bit.PRDLD = TB_IMMEDIATE;
    p->TBPRD = DAB_CHG_TBPRD;
    p->TBCTR = 0U;
    p->TBPHS.all = 0UL;
    p->EPWMSYNCINSEL.all = SYNC_IN_SRC_DISABLE_ALL;
    p->EPWMSYNCOUTEN.all = SYNC_OUT_SRC_DISABLE_ALL;

    p->CMPCTL.all = 0U;
    p->CMPA.all = ((Uint32)(DAB_CHG_TBPRD / 2U) << 16);
    p->CMPB.all = ((Uint32)(DAB_CHG_TBPRD / 2U) << 16);
    p->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    p->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    p->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    p->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    p->AQCTL.all = 0U;
    p->AQCTLA.all = 0U;
    p->AQCTLB.all = 0U;
    p->AQSFRC.bit.RLDCSF = 3U;
    p->AQCSFRC.all = 0x0005U;
    p->AQCTL.bit.SHDWAQAMODE = 1U;
    p->AQCTL.bit.SHDWAQBMODE = 1U;

    p->DBCTL.all = 0U;
    p->PCCTL.all = 0U;
    p->HRCNFG.all = 0U;
    p->HRPCTL.all = 0U;

    p->EPWMXLINK.bit.CMPALINK = selfLink;
    p->EPWMXLINK.bit.CMPBLINK = selfLink;
    p->EPWMXLINK.bit.GLDCTL2LINK = gldMaster;
    p->GLDCFG.bit.CMPA_CMPAHR = 1U;
    p->GLDCFG.bit.CMPB_CMPBHR = 1U;
    p->GLDCFG.bit.AQCTLA_AQCTLA2 = 1U;
    p->GLDCFG.bit.AQCTLB_AQCTLB2 = 1U;
    p->GLDCFG.bit.AQCSFRC = 1U;
    p->GLDCTL.bit.OSHTMODE = 1U;
    p->GLDCTL.bit.GLDMODE = 0U;
    p->GLDCTL.bit.GLDPRD = 1U;
    p->GLDCTL.bit.GLD = 1U;
}

/** 装充电 AQ 底座（与旧 1/2/3/4 逻辑一致） */
static void LoadChargeAq(volatile struct EPWM_REGS *p1,
                         volatile struct EPWM_REGS *p2,
                         volatile struct EPWM_REGS *p3,
                         volatile struct EPWM_REGS *p4)
{
    /* S1 */
    p1->AQCTLA.bit.ZRO = AQ_SET;
    p1->AQCTLA.bit.PRD = AQ_CLEAR;
    p1->AQCTLA.bit.CAU = AQ_CLEAR;
    p1->AQCTLA.bit.CAD = AQ_CLEAR;
    p1->AQCTLA.bit.CBU = AQ_NO_ACTION;
    p1->AQCTLA.bit.CBD = AQ_NO_ACTION;
    /* S2 */
    p1->AQCTLB.bit.ZRO = AQ_CLEAR;
    p1->AQCTLB.bit.PRD = AQ_SET;
    p1->AQCTLB.bit.CAU = AQ_NO_ACTION;
    p1->AQCTLB.bit.CAD = AQ_NO_ACTION;
    p1->AQCTLB.bit.CBU = AQ_CLEAR;
    p1->AQCTLB.bit.CBD = AQ_CLEAR;
    /* S3 */
    p2->AQCTLA.bit.ZRO = AQ_CLEAR;
    p2->AQCTLA.bit.PRD = AQ_SET;
    p2->AQCTLA.bit.CAU = AQ_CLEAR;
    p2->AQCTLA.bit.CAD = AQ_CLEAR;
    p2->AQCTLA.bit.CBU = AQ_NO_ACTION;
    p2->AQCTLA.bit.CBD = AQ_NO_ACTION;
    /* S4 */
    p2->AQCTLB.bit.ZRO = AQ_SET;
    p2->AQCTLB.bit.PRD = AQ_CLEAR;
    p2->AQCTLB.bit.CAU = AQ_NO_ACTION;
    p2->AQCTLB.bit.CAD = AQ_NO_ACTION;
    p2->AQCTLB.bit.CBU = AQ_CLEAR;
    p2->AQCTLB.bit.CBD = AQ_CLEAR;
    /* S5~S8 同 S1~S4 */
    p3->AQCTLA.bit.ZRO = AQ_SET;
    p3->AQCTLA.bit.PRD = AQ_CLEAR;
    p3->AQCTLA.bit.CAU = AQ_CLEAR;
    p3->AQCTLA.bit.CAD = AQ_CLEAR;
    p3->AQCTLA.bit.CBU = AQ_NO_ACTION;
    p3->AQCTLA.bit.CBD = AQ_NO_ACTION;

    p3->AQCTLB.bit.ZRO = AQ_CLEAR;
    p3->AQCTLB.bit.PRD = AQ_SET;
    p3->AQCTLB.bit.CAU = AQ_NO_ACTION;
    p3->AQCTLB.bit.CAD = AQ_NO_ACTION;
    p3->AQCTLB.bit.CBU = AQ_CLEAR;
    p3->AQCTLB.bit.CBD = AQ_CLEAR;

    p4->AQCTLA.bit.ZRO = AQ_CLEAR;
    p4->AQCTLA.bit.PRD = AQ_SET;
    p4->AQCTLA.bit.CAU = AQ_CLEAR;
    p4->AQCTLA.bit.CAD = AQ_CLEAR;
    p4->AQCTLA.bit.CBU = AQ_NO_ACTION;
    p4->AQCTLA.bit.CBD = AQ_NO_ACTION;

    p4->AQCTLB.bit.ZRO = AQ_SET;
    p4->AQCTLB.bit.PRD = AQ_CLEAR;
    p4->AQCTLB.bit.CAU = AQ_NO_ACTION;
    p4->AQCTLB.bit.CAD = AQ_NO_ACTION;
    p4->AQCTLB.bit.CBU = AQ_CLEAR;
    p4->AQCTLB.bit.CBD = AQ_CLEAR;
}

/* 保持 RLDCSF=3（立即模式）强制低，不切回 0：
 * 防止滞后 90° 的桥 2 已武装 OSHTLD 在 ZERO 时把旧影子 AQCSFRC(0x0000/0x0001) 重新装入解封；
 * 下次真正发波时由 Dab_Pwm_Commit() 在 u16Phase==0 下统一切回 RLDCSF=0 并重写影子。 */
static void ForceSoftOff(volatile struct EPWM_REGS *p)
{
    p->AQSFRC.bit.RLDCSF = 3U;
    p->AQCSFRC.all = 0x0005U;
}

/* 入参 prd = 完整载波周期 TBPRD（up-down 模式：一个载波周期 = 2×prd 个计数）
 * 90° = 1/4 周期 = prd/2，写入桥2 主模块 EPWM7 的 TBPHS；
 * 例如充电 TBPRD=1500 → TBPHS=750（不是 1500）。 */
static void ApplySyncPhase(Uint16 prd)
{
    Uint16 ph90 = DAB_PHASE_90_FROM_PRD(prd);

    /* 桥1：从机跟 EPWM10，相位 0 */
    DabEpwm2Regs.TBPHS.bit.TBPHS = 0U;
    DabEpwm3Regs.TBPHS.bit.TBPHS = 0U;
    DabEpwm4Regs.TBPHS.bit.TBPHS = 0U;
    DabEpwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    DabEpwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    DabEpwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    DabEpwm2Regs.EPWMSYNCINSEL.all = SYNC_IN_SRC_SYNCOUT_EPWM10;
    DabEpwm3Regs.EPWMSYNCINSEL.all = SYNC_IN_SRC_SYNCOUT_EPWM10;
    DabEpwm4Regs.EPWMSYNCINSEL.all = SYNC_IN_SRC_SYNCOUT_EPWM10;

    /* 桥2：仅 PWM1(EPWM7) 跟 EPWM10 且滞后 90°；其余跟本桥 EPWM7@0° */
    DabEpwm1bRegs.TBPHS.bit.TBPHS = ph90;
    DabEpwm1bRegs.TBCTL.bit.PHSEN = TB_ENABLE;
    DabEpwm1bRegs.EPWMSYNCINSEL.all = SYNC_IN_SRC_SYNCOUT_EPWM10;
    DabEpwm1bRegs.EPWMSYNCOUTEN.bit.ZEROEN = 1U;

    DabEpwm2bRegs.TBPHS.bit.TBPHS = 0U;
    DabEpwm3bRegs.TBPHS.bit.TBPHS = 0U;
    DabEpwm4bRegs.TBPHS.bit.TBPHS = 0U;
    DabEpwm2bRegs.TBCTL.bit.PHSEN = TB_ENABLE;
    DabEpwm3bRegs.TBCTL.bit.PHSEN = TB_ENABLE;
    DabEpwm4bRegs.TBCTL.bit.PHSEN = TB_ENABLE;
    DabEpwm2bRegs.EPWMSYNCINSEL.all = SYNC_IN_SRC_SYNCOUT_EPWM7;
    DabEpwm3bRegs.EPWMSYNCINSEL.all = SYNC_IN_SRC_SYNCOUT_EPWM7;
    DabEpwm4bRegs.EPWMSYNCINSEL.all = SYNC_IN_SRC_SYNCOUT_EPWM7;
}

/*------------------------------------------------------------------------------------------------*/
void DAB_PWM_Stop(void)
{
    uint16_t intm = __disable_interrupts();

    ForceSoftOff(&DabEpwm1Regs);
    ForceSoftOff(&DabEpwm2Regs);
    ForceSoftOff(&DabEpwm3Regs);
    ForceSoftOff(&DabEpwm4Regs);
    ForceSoftOff(&DabEpwm1bRegs);
    ForceSoftOff(&DabEpwm2bRegs);
    ForceSoftOff(&DabEpwm3bRegs);
    ForceSoftOff(&DabEpwm4bRegs);

    /* 同步作废两桥 AQ 动作缓存与相位，保证下次发波经 Dab_Pwm_Commit() 重新切回 RLDCSF=0 */
    g_stDabIsrState.stChgAqBr1.all = DAB_PWM_STATE_RESET;
    g_stDabIsrState.stChgAqBr2.all = DAB_PWM_STATE_RESET;

    __restore_interrupts(intm);
}

void DAB_PWM_OstStop(void)
{
    uint16_t intm = __disable_interrupts();

    EALLOW;
    DabEpwm1Regs.TZFRC.bit.OST = 1U;
    DabEpwm2Regs.TZFRC.bit.OST = 1U;
    DabEpwm3Regs.TZFRC.bit.OST = 1U;
    DabEpwm4Regs.TZFRC.bit.OST = 1U;
    DabEpwm1bRegs.TZFRC.bit.OST = 1U;
    DabEpwm2bRegs.TZFRC.bit.OST = 1U;
    DabEpwm3bRegs.TZFRC.bit.OST = 1U;
    DabEpwm4bRegs.TZFRC.bit.OST = 1U;
    EDIS;

    __restore_interrupts(intm);
}

void DAB_PWM_Start(void)
{
    const Uint16 u16Clr = (Uint16)(EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1);

    EALLOW;
    DabEpwm1Regs.TZCLR.all  = u16Clr;
    DabEpwm2Regs.TZCLR.all  = u16Clr;
    DabEpwm3Regs.TZCLR.all  = u16Clr;
    DabEpwm4Regs.TZCLR.all  = u16Clr;
    DabEpwm1bRegs.TZCLR.all = u16Clr;
    DabEpwm2bRegs.TZCLR.all = u16Clr;
    DabEpwm3bRegs.TZCLR.all = u16Clr;
    DabEpwm4bRegs.TZCLR.all = u16Clr;
    EDIS;
}

void DAB_PWM_SetMode(Uint16 workMode)
{
    Uint16 prd;

    prd = (workMode == (Uint16)WORK_MODE_CHARGE) ? DAB_CHG_TBPRD : DAB_DIS_TBPRD;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0U;

    DabEpwm1Regs.TBCTR = 0U;
    DabEpwm2Regs.TBCTR = 0U;
    DabEpwm3Regs.TBCTR = 0U;
    DabEpwm4Regs.TBCTR = 0U;
    DabEpwm1bRegs.TBCTR = 0U;
    DabEpwm2bRegs.TBCTR = 0U;
    DabEpwm3bRegs.TBCTR = 0U;
    DabEpwm4bRegs.TBCTR = 0U;

    DabEpwm1Regs.TBPRD = prd;
    DabEpwm2Regs.TBPRD = prd;
    DabEpwm3Regs.TBPRD = prd;
    DabEpwm4Regs.TBPRD = prd;
    DabEpwm1bRegs.TBPRD = prd;
    DabEpwm2bRegs.TBPRD = prd;
    DabEpwm3bRegs.TBPRD = prd;
    DabEpwm4bRegs.TBPRD = prd;

    g_u16DabPwmTicks100us = (prd == DAB_CHG_TBPRD) ? 5U : 6U;
    ApplySyncPhase(prd);

    /* AQ 模板：充电复用完整 LoadChargeAq，再镜像桥2；放电只清充电残留开通沿，发波由 DisUpdate 重写 */
    if (workMode == (Uint16)WORK_MODE_CHARGE)
    {
        LoadChargeAq(&DabEpwm1Regs, &DabEpwm2Regs, &DabEpwm3Regs, &DabEpwm4Regs);
    }
    else
    {
        DabEpwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
        DabEpwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
        DabEpwm1Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        DabEpwm1Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;

        DabEpwm2Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
        DabEpwm2Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        DabEpwm2Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;

        DabEpwm3Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
        DabEpwm3Regs.AQCTLB.bit.CBU = AQ_NO_ACTION;
        DabEpwm4Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
        DabEpwm4Regs.AQCTLB.bit.CBD = AQ_NO_ACTION;
    }
    DabPwm_MirrorShadowBridge2();

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1U;
    EDIS;
}

void DAB_PWM_Init(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0U;
    DAB_PWM_Stop();
    EALLOW;

    /* 桥1：CMP 自链，GLD 主机 = EPWM10（本桥 PWM1） */
    SetupBase(&DabEpwm1Regs, DAB_XLINK_EPWM10, DAB_XLINK_EPWM10);
    SetupBase(&DabEpwm2Regs, DAB_XLINK_EPWM12, DAB_XLINK_EPWM10);
    SetupBase(&DabEpwm3Regs, DAB_XLINK_EPWM11, DAB_XLINK_EPWM10);
    SetupBase(&DabEpwm4Regs, DAB_XLINK_EPWM3,  DAB_XLINK_EPWM10);

    /* 桥2：CMP 自链，GLD 主机 = EPWM7（本桥 PWM1）；与桥1 分开装载 */
    SetupBase(&DabEpwm1bRegs, DAB_XLINK_EPWM7, DAB_XLINK_EPWM7);
    SetupBase(&DabEpwm2bRegs, DAB_XLINK_EPWM9, DAB_XLINK_EPWM7);
    SetupBase(&DabEpwm3bRegs, DAB_XLINK_EPWM2, DAB_XLINK_EPWM7);
    SetupBase(&DabEpwm4bRegs, DAB_XLINK_EPWM4, DAB_XLINK_EPWM7);

    DabEpwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    DabEpwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    DabEpwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    DabEpwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    DabEpwm1bRegs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    DabEpwm2bRegs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    DabEpwm3bRegs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    DabEpwm4bRegs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

    /* 桥1 主机 EPWM10：ZERO → SYNCOUT + ISR + SOCA */
    DabEpwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    DabEpwm1Regs.EPWMSYNCOUTEN.bit.ZEROEN = 1U;

    DabEpwm1Regs.TBPRD = DAB_CHG_TBPRD;
    DabEpwm2Regs.TBPRD = DAB_CHG_TBPRD;
    DabEpwm3Regs.TBPRD = DAB_CHG_TBPRD;
    DabEpwm4Regs.TBPRD = DAB_CHG_TBPRD;
    DabEpwm1bRegs.TBPRD = DAB_CHG_TBPRD;
    DabEpwm2bRegs.TBPRD = DAB_CHG_TBPRD;
    DabEpwm3bRegs.TBPRD = DAB_CHG_TBPRD;
    DabEpwm4bRegs.TBPRD = DAB_CHG_TBPRD;

    g_u16DabPwmTicks100us = 5U;
    ApplySyncPhase(DAB_CHG_TBPRD);

    LoadChargeAq(&DabEpwm1Regs, &DabEpwm2Regs, &DabEpwm3Regs, &DabEpwm4Regs);
    LoadChargeAq(&DabEpwm1bRegs, &DabEpwm2bRegs, &DabEpwm3bRegs, &DabEpwm4bRegs);

    DAB_EPWM_MASTER_REGS.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    DAB_EPWM_MASTER_REGS.ETPS.bit.INTPRD = ET_1ST;
    DAB_EPWM_MASTER_REGS.ETSEL.bit.INTEN = 0U;   /* 留到 main() EINT 前开启，防校准期间挂起 */

    /* 双 ADC 触发：桥1@EPWM10 PRD、桥2@EPWM7 PRD（交错 90°） */
    DAB_EPWM_MASTER_REGS.ETSEL.bit.SOCASEL = ET_CTR_PRD;
    DAB_EPWM_MASTER_REGS.ETPS.bit.SOCAPRD = ET_1ST;
    DAB_EPWM_MASTER_REGS.ETSEL.bit.SOCAEN = 1U;

    DAB_EPWM_MASTER2_REGS.ETSEL.bit.SOCASEL = ET_CTR_PRD;
    DAB_EPWM_MASTER2_REGS.ETPS.bit.SOCAPRD = ET_1ST;
    DAB_EPWM_MASTER2_REGS.ETSEL.bit.SOCAEN = 1U;
    EDIS;

    DAB_PWM_Stop();
}

void InitChip_EpwmRegs(void)
{
    DAB_PWM_Init();
}

void EPWM_TzEnable(void)
{
    DAB_PWM_Stop();
    EALLOW;
    DabEpwm1Regs.TZFRC.bit.OST = 1U;
    DabEpwm2Regs.TZFRC.bit.OST = 1U;
    DabEpwm3Regs.TZFRC.bit.OST = 1U;
    DabEpwm4Regs.TZFRC.bit.OST = 1U;
    DabEpwm1bRegs.TZFRC.bit.OST = 1U;
    DabEpwm2bRegs.TZFRC.bit.OST = 1U;
    DabEpwm3bRegs.TZFRC.bit.OST = 1U;
    DabEpwm4bRegs.TZFRC.bit.OST = 1U;
    EDIS;
}

void EPWMStartTimer(void)
{
    DAB_PWM_Stop();
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0U;
    DabEpwm1Regs.TBCTR = 0U;
    DabEpwm2Regs.TBCTR = 0U;
    DabEpwm3Regs.TBCTR = 0U;
    DabEpwm4Regs.TBCTR = 0U;
    DabEpwm1bRegs.TBCTR = 0U;
    DabEpwm2bRegs.TBCTR = 0U;
    DabEpwm3bRegs.TBCTR = 0U;
    DabEpwm4bRegs.TBCTR = 0U;
    ApplySyncPhase(DabEpwm1Regs.TBPRD);
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1U;
    EDIS;
}
