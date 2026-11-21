/*************************************************************************************************
 * ConfigChip_AdcInit.c — ADC 上电 / SOC 配置（与 ConfigChip_AdcInit.h 槽位一致）
 *
 * 配置要点：
 *   1) 三模块均分：ADCA 5 + ADCB 5 + ADCC 6，避免单 ADC 串行过长
 *   2) 电流首采：ADCA.SOC0=IDD1(A7)，ADCB.SOC0=IDD2(B8)
 *   3) 双触发：EPWM10 SOCA→桥1 通道；EPWM7 SOCA→IDD2、VBUS2
 *   4) ACQPS=24；未用 SOC 清零，防止误触发
 *************************************************************************************************/
#include "Includes.h"
#include "ConfigChip_AdcInit.h"

/** 配置单个 SOC：通道 CHSEL + 触发 TRIGSEL + 采样窗 ACQPS */
static void Adc_SetupSoc(volatile struct ADC_REGS *adc,
                         Uint16 soc,
                         Uint16 chsel,
                         Uint16 trig,
                         Uint16 acqps)
{
    switch (soc)
    {
        case 0U:
            adc->ADCSOC0CTL.bit.CHSEL = chsel;
            adc->ADCSOC0CTL.bit.TRIGSEL = trig;
            adc->ADCSOC0CTL.bit.ACQPS = acqps;
            break;
        case 1U:
            adc->ADCSOC1CTL.bit.CHSEL = chsel;
            adc->ADCSOC1CTL.bit.TRIGSEL = trig;
            adc->ADCSOC1CTL.bit.ACQPS = acqps;
            break;
        case 2U:
            adc->ADCSOC2CTL.bit.CHSEL = chsel;
            adc->ADCSOC2CTL.bit.TRIGSEL = trig;
            adc->ADCSOC2CTL.bit.ACQPS = acqps;
            break;
        case 3U:
            adc->ADCSOC3CTL.bit.CHSEL = chsel;
            adc->ADCSOC3CTL.bit.TRIGSEL = trig;
            adc->ADCSOC3CTL.bit.ACQPS = acqps;
            break;
        case 4U:
            adc->ADCSOC4CTL.bit.CHSEL = chsel;
            adc->ADCSOC4CTL.bit.TRIGSEL = trig;
            adc->ADCSOC4CTL.bit.ACQPS = acqps;
            break;
        case 5U:
            adc->ADCSOC5CTL.bit.CHSEL = chsel;
            adc->ADCSOC5CTL.bit.TRIGSEL = trig;
            adc->ADCSOC5CTL.bit.ACQPS = acqps;
            break;
        case 6U:
            adc->ADCSOC6CTL.bit.CHSEL = chsel;
            adc->ADCSOC6CTL.bit.TRIGSEL = trig;
            adc->ADCSOC6CTL.bit.ACQPS = acqps;
            break;
        case 7U:
            adc->ADCSOC7CTL.bit.CHSEL = chsel;
            adc->ADCSOC7CTL.bit.TRIGSEL = trig;
            adc->ADCSOC7CTL.bit.ACQPS = acqps;
            break;
        default:
            break;
    }
}

/** 清零未使用的 SOC（fromSoc…7），避免残留 TRIGSEL 误启动转换 */
static void Adc_ClearUnusedSoc(volatile struct ADC_REGS *adc, Uint16 fromSoc)
{
    if (fromSoc <= 0U) { adc->ADCSOC0CTL.all = 0U; }
    if (fromSoc <= 1U) { adc->ADCSOC1CTL.all = 0U; }
    if (fromSoc <= 2U) { adc->ADCSOC2CTL.all = 0U; }
    if (fromSoc <= 3U) { adc->ADCSOC3CTL.all = 0U; }
    if (fromSoc <= 4U) { adc->ADCSOC4CTL.all = 0U; }
    if (fromSoc <= 5U) { adc->ADCSOC5CTL.all = 0U; }
    if (fromSoc <= 6U) { adc->ADCSOC6CTL.all = 0U; }
    if (fromSoc <= 7U) { adc->ADCSOC7CTL.all = 0U; }
}

void InitChip_AdcRegs(void)
{
    /* trig1：桥1 载波 PRD；trig2：桥2 载波 PRD（滞后 90°） */
    const Uint16 trig1 = (Uint16)ADC_TRIGGER_EPWM10_SOCA;
    const Uint16 trig2 = (Uint16)ADC_TRIGGER_EPWM7_SOCA;
    const Uint16 acq   = DAB_ADC_ACQPS;

    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_EXTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(ADCB_BASE, ADC_REFERENCE_EXTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(ADCC_BASE, ADC_REFERENCE_EXTERNAL, ADC_REFERENCE_3_3V);

    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 2U;
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1U;

    AdcbRegs.ADCCTL2.bit.PRESCALE = 2U;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1U;

    AdccRegs.ADCCTL2.bit.PRESCALE = 2U;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1U;
    EDIS;

    DEVICE_DELAY_US(1000); /* 等待模拟上电稳定 */

    EALLOW;

    /* -------- ADCA（5）：SOC0=IDD1 首采，其余跟 EPWM10 -------- */
    Adc_SetupSoc(&AdcaRegs, DAB_ADC_A_IDD1_ADC,      7U, trig1, acq); /* A7  IDD1_ADC ★首采 */
    Adc_SetupSoc(&AdcaRegs, DAB_ADC_A_VMBUS_ADC,    12U, trig1, acq); /* A12 VMBUS_ADC 继电器 VM */
    Adc_SetupSoc(&AdcaRegs, DAB_ADC_A_PACK_NTC_ADC,  6U, trig1, acq); /* A6  PACK_NTC_ADC */
    Adc_SetupSoc(&AdcaRegs, DAB_ADC_A_IDD1_OCP,      5U, trig1, acq); /* A5  IDD1_OCP */
    Adc_SetupSoc(&AdcaRegs, DAB_ADC_A_IDD2_OCP,      8U, trig1, acq); /* A8  IDD2_OCP */
    Adc_ClearUnusedSoc(&AdcaRegs, 5U);

    /* -------- ADCB（5）：SOC0=IDD2@B8 首采；电流+母线2 跟 EPWM7 -------- */
    Adc_SetupSoc(&AdcbRegs, DAB_ADC_B_IDD2_ADC,      8U, trig2, acq); /* B8  IDD2_ADC ★首采 */
    Adc_SetupSoc(&AdcbRegs, DAB_ADC_B_VBUS2_ADC,    11U, trig2, acq); /* B11 VBUS2_ADC */
    Adc_SetupSoc(&AdcbRegs, DAB_ADC_B_VPBUS_ADC,     4U, trig1, acq); /* B4  VPBUS_ADC PCS端电压 */
    Adc_SetupSoc(&AdcbRegs, DAB_ADC_B_VMBUS_OVP_ADC, 5U, trig1, acq); /* B5  VMBUS_OVP_ADC */
    Adc_SetupSoc(&AdcbRegs, DAB_ADC_B_MODEL1,        3U, trig1, acq); /* B3  MODEL1 */
    Adc_ClearUnusedSoc(&AdcbRegs, 5U);

    /* -------- ADCC（6）：并联脚走 C，分摊转换时间 -------- */
    Adc_SetupSoc(&AdccRegs, DAB_ADC_C_VBUS1_ADC,     1U, trig1, acq); /* C1  VBUS1_ADC */
    Adc_SetupSoc(&AdccRegs, DAB_ADC_C_VDC_ADC,      10U, trig1, acq); /* C10 VDC_ADC（同 A10） */
    Adc_SetupSoc(&AdccRegs, DAB_ADC_C_HV_NTC_ADC,    6U, trig1, acq); /* C6  HV_NTC（同 B2） */
    Adc_SetupSoc(&AdccRegs, DAB_ADC_C_BATT_NTC_ADC, 16U, trig1, acq); /* C16 BATT_NTC（同 A16） */
    Adc_SetupSoc(&AdccRegs, DAB_ADC_C_MODEL2,        9U, trig1, acq); /* C9  MODEL2（同 B6） */
    Adc_SetupSoc(&AdccRegs, DAB_ADC_C_MODEL3,        7U, trig1, acq); /* C7  MODEL3（同 B9） */
    Adc_ClearUnusedSoc(&AdccRegs, 6U);

    /* ADCINT1 可选：ADCA 末 SOC 完成脉冲（控制路径不依赖本中断） */
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1U;
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 1U;
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 4U; /* 对应 IDD2_OCP */

    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 0U;
    AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = 0U;
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0U;

    AdccRegs.ADCINTSEL1N2.bit.INT1E = 0U;
    AdccRegs.ADCINTSEL1N2.bit.INT1CONT = 0U;
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0U;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
    EDIS;

    DEVICE_DELAY_US(1000);
}

void InitChip_DMARegs(void)
{
}
