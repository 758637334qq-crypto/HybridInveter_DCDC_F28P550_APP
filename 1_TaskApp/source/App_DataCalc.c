/*************************************************************************************************
Copyright 2022, celink Corporation.
File name:       App_DataCalc.c
Description:     CAN 遥测汇总（把运行采样/设定/状态整理进 g_CanMon，供 0x11 上报）
Update Date:     2022/5/22
Writer:          zy
*************************************************************************************************/

#include "Includes.h"

/* 浮点饱和转 Int16：防止短路/大功率瞬态超 ±32767 触发未定义行为 */
static inline Int16 sF32ToI16Sat(float f)
{
    return (Int16)__fmin(__fmax(f, -32768.0f), 32767.0f);
}

/* 浮点工程量 -> 协议定标 */
#define F_TO_DV(f)   ((Uint16)((f) * 10.0f + 0.5f))   /* V  -> 0.1V  */
#define F_TO_CA(f)   sF32ToI16Sat((f) * 100.0f)        /* A  -> 0.01A */

/**
 * @brief  汇总 ADC 采样、环路控制量、设定/限制、运行状态与告警，写入 g_CanMon
 * @note   5ms 周期调用；定标统一为 dV=0.1V、cA=0.01A、dC=0.1℃、W=1W
 */
void Update_CanMon(void)
{
    float fPout;

    /* ----- ADC 模拟量 ----- */
    g_CanMon.ADC.vbat_dV         = F_TO_DV(stADC.BatVolt.f32IsrResult);    /* 电池电压 */
    g_CanMon.ADC.vbusPcs_dV      = F_TO_DV(stADC.PcsBusVolt.f32IsrResult); /* PCS 端电压 */
    g_CanMon.ADC.vbus1_dV        = F_TO_DV(stADC.BusVolt1.f32IsrResult);    /* 高压母线 1 */
    g_CanMon.ADC.vbus2_dV        = F_TO_DV(stADC.BusVolt2.f32IsrResult);    /* 高压母线 2 */
    g_CanMon.ADC.vmBus_dV        = F_TO_DV(stADC.VmBusVolt.f32IsrResult);  /* 继电器 VM */
    g_CanMon.ADC.sampleIr_cA     = F_TO_CA(stADC.BatCurr.f32IsrResult);    /* 低压侧电流粗估 */
    g_CanMon.ADC.iout1_cA        = F_TO_CA(stADC.BusCurr1.f32IsrResult);    /* 输出电流 1 */
    g_CanMon.ADC.iout2_cA        = F_TO_CA(stADC.BusCurr2.f32IsrResult);    /* 输出电流 2 */

    /* 输出功率 = 400ms 滤波后的电压 × 电流，避免上报值随开关纹波跳动 */
    fPout = stADC.BusVolt1.Filt.f32Filt400ms * stADC.BusCurr1.Filt.f32Filt400ms;
    g_CanMon.ADC.pout1_W         = (Int16)fPout;
    fPout = stADC.BusVolt2.Filt.f32Filt400ms * stADC.BusCurr2.Filt.f32Filt400ms;
    g_CanMon.ADC.pout2_W         = (Int16)fPout;

    /* 本板未接入 NTC 采样通道，温度暂时固定上报 0；接入后在此替换为实测温度 */
    g_CanMon.ADC.tempInductor_dC = 0;
    g_CanMon.ADC.tempAmbient_dC  = 0;
    g_CanMon.ADC.tempHeatsink_dC = 0;
    g_CanMon.ADC.tempMos_dC      = 0;

    /* ----- 环路控制量 ----- */
    /* 两路共用同一个母线电压外环，因此 vLoop1/vLoop2 上报同一组值（×100 → 0.01 单位）。
     * 注意：DAB 外环输出 f32Out 是电流校正量（A），字段名沿用原协议 vLoop_Out。 */
    g_CanMon.LoopCtrl.vLoop1_Err_cV = sF32ToI16Sat(Bus.f32Err * 100.0f);   /* 电压误差，0.01V */
    g_CanMon.LoopCtrl.vLoop1_Out    = (Int16)(Bus.f32Out * 100.0f);   /* 外环输出，0.01 单位 */
    g_CanMon.LoopCtrl.vLoop2_Err_cV = g_CanMon.LoopCtrl.vLoop1_Err_cV;
    g_CanMon.LoopCtrl.vLoop2_Out    = g_CanMon.LoopCtrl.vLoop1_Out;
    /* 电压给定约 360~450V，0.01V 放不进 Int16，这里用 0.1V。前馈是电流，单位 0.01A。 */
    g_CanMon.LoopCtrl.vLoop_Ref_dV  = (Int16)(Bus.f32Vref * 10.0f + 0.5f);
    g_CanMon.LoopCtrl.vLoop_Ff_cA   = 0;   /* 预留：电压环电流前馈已废除，变量已删除 */

    /* 内环误差取电流环保存的上一拍误差 f32Err1（单位 A，×100 → 0.01A） */
    g_CanMon.LoopCtrl.iLoop1_Err_cA = (Int16)(stDabPi0.f32Err1 * 100.0f);
    g_CanMon.LoopCtrl.iLoop2_Err_cA = (Int16)(stDabPi1.f32Err1 * 100.0f);
    /* 内环输出：充电模式为电流域 PI 输出（A）、放电模式为占空比修正量，按 0.01 单位原样上报 */
    g_CanMon.LoopCtrl.iLoop1_Out    = (Int16)(stDabPi0.f32Out * 100.0f);
    g_CanMon.LoopCtrl.iLoop2_Out    = (Int16)(stDabPi1.f32Out * 100.0f);
    /* 0x09 仍上报两路实际发波占空比，0.01% 定标（1.0 = 10000） */
    g_CanMon.LoopCtrl.vLoop1_DutyFf = (Uint16)(stDabDuty0.f32Duty * 10000.0f);
    g_CanMon.LoopCtrl.vLoop2_DutyFf = (Uint16)(stDabDuty1.f32Duty * 10000.0f);
    /* 电流环前馈就是占空比前馈：放电为软起后的 u_ff，充电为开方 d_ff */
    g_CanMon.LoopCtrl.iLoop1_DutyFf = (Uint16)(stDabPi0.f32DutyFf * 10000.0f);
    g_CanMon.LoopCtrl.iLoop2_DutyFf = (Uint16)(stDabPi1.f32DutyFf * 10000.0f);
    g_CanMon.LoopCtrl.iLoop1_Ff_cA  = 0;   /* 预留：电压环电流前馈已废除，变量已删除 */
    g_CanMon.LoopCtrl.iLoop2_Ff_cA  = 0;

    /* ----- 设定值 (CfgParam: 电压 0.1V, 电流 0.1A) ----- */
    g_CanMon.Setpoint.voutSet_dV    = CfgParam.BatVolSet;      /* 输出电压给定 = 电池电压设定 */
    /* 母线总调节电压 CurShare.voltAdj [V] → 0.1V，四舍五入 */
    g_CanMon.Setpoint.vbusDroop_dV  = (Int16)(CurShare.voltAdj * 10.0f +
                                            ((CurShare.voltAdj >= 0.0f) ? 0.5f : -0.5f));
    /* 母线 Ref/Set/Cmd：控制 ref / 下发设定 / 步进设定（两路共用外环，同源上报） */
    g_CanMon.Setpoint.vbus1Ref_dV   = F_TO_DV(Bus.f32Vref);     /* 控制 ref = Ramp.v + voltAdj */
    g_CanMon.Setpoint.vbus1Set_dV   = CfgParam.BusVolSet;       /* 上位机下发母线设定 [0.1V] */
    g_CanMon.Setpoint.vbus1Cmd_dV   = F_TO_DV(Ramp.v);          /* 斜坡步进后的电压设定 */
    g_CanMon.Setpoint.resv_0x0F_hi  = 0;                        /* 0x0F 高半字保留，显式清零 */
    g_CanMon.Setpoint.vbus2Ref_dV   = g_CanMon.Setpoint.vbus1Ref_dV;
    g_CanMon.Setpoint.vbus2Set_dV   = g_CanMon.Setpoint.vbus1Set_dV;
    g_CanMon.Setpoint.vbus2Cmd_dV   = g_CanMon.Setpoint.vbus1Cmd_dV;
    g_CanMon.Setpoint.resv_0x11_hi  = 0;                        /* 0x11 高半字保留，显式清零 */
    g_CanMon.Setpoint.iCharSet_cA   = (Int16)(Ramp.iNeg * 100.0f);   /* 实际充电电流给定，负值，A→0.01A */
    g_CanMon.Setpoint.ibus1ref_cA   = (Int16)(stDabPi0.f32Ref * 100.0f); /* 0 路电流环实际给定 */
    g_CanMon.Setpoint.iReCharSet_cA = (Int16)(Ramp.iPos * 100.0f);  /* 实际放电电流给定，A→0.01A */
    g_CanMon.Setpoint.ibus2ref_cA   = (Int16)(stDabPi1.f32Ref * 100.0f); /* 1 路电流环实际给定 */
    /* CAN 0x14 低半字：下垂调节电压 droop [V] → 0.1V */
    g_CanMon.Setpoint.droopCur_dv   = (Int16)(CurShare.droop * 10.0f +
                                            ((CurShare.droop >= 0.0f) ? 0.5f : -0.5f));
    /* CAN 0x14 高半字：均流调节电压 voltAdj [V] → 0.1V */
    g_CanMon.Setpoint.canAverCur_dv = (Int16)(CurShare.voltAdj * 10.0f +
                                            ((CurShare.voltAdj >= 0.0f) ? 0.5f : -0.5f));

    /* ----- 功率限制 ----- */
    /* 系统最大充/放电功率 = 电池电流设定 × 电池电压设定（0.1A × 0.1V → W） */
    g_CanMon.PowerLimit.sysMaxChgPow_W    = (Uint16)((float)CfgParam.BatChgCurSet * 0.1f *
                                                     (float)CfgParam.BatVolSet * 0.1f);
    g_CanMon.PowerLimit.sysMaxRechPow_W   = (Uint16)((float)CfgParam.BatDischgCurSet * 0.1f *
                                                     (float)CfgParam.BatVolSet * 0.1f);
    g_CanMon.PowerLimit.ibatLimChar_cA    = (Uint16)(CfgParam.BatChgCurSet * 10);  /* 0.1A -> 0.01A */
    g_CanMon.PowerLimit.bmsCharPowSet_W   = g_CanMon.PowerLimit.sysMaxChgPow_W;    /* 未接 BMS 时跟随系统上限 */
    g_CanMon.PowerLimit.bmsrecharPowSet_W = g_CanMon.PowerLimit.sysMaxRechPow_W;   /* 同上 */
    /* 本机型未做温度降额与电池限压降额，三项固定上报 0 */
    g_CanMon.PowerLimit.tempLimChar_W     = 0;
    g_CanMon.PowerLimit.tempLimRech_W     = 0;
    g_CanMon.PowerLimit.vbatLimRech_W     = 0;

    /* ----- 运行状态（CAN 0x19 低半字）-----
     * bit0~7  状态码 | bit8~9 工作模式
     * bit10   禁充 | bit11 禁放 | bit12 待机 standby
     * 一次整字写入，避免 CAN 中断读到中间态。 */
    g_CanMon.Status.runFlag.all = (Uint16)(
        ((Uint16)RunState.state & 0x00FFU) |
        (((Uint16)RunState.workMode & 0x0003U) << 8) |
        (((Uint16)RunState.curLowSealing & SM_SEAL_NO_CHARGE) ? (1U << 10) : 0U) |
        (((Uint16)RunState.curLowSealing & SM_SEAL_NO_DISCHARGE) ? (1U << 11) : 0U) |
        ((RunState.state == SM_STATE_STANDBY) ? (1U << 12) : 0U));
    g_CanMon.Status.dabRunSta = (Uint16)RunState.state; /* SmState_e，串口 0x49 / CAN 对齐 */
    g_CanMon.Status.canRecCnt = sCanMes.uCanRec_count;                    /* CAN 接收计数 */
    g_CanMon.Status.resv      = 0;                                        /* 保留字 */

    /* ----- 实时告警：内部只存一份 alm；CAN/SCI 打包再映射原 slowfault2 槽 ----- */
    g_CanMon.RealAlarm.slowfault1 = stFault.slow;
    g_CanMon.RealAlarm.fastfault  = stFault.fast;
    g_CanMon.RealAlarm.alarm      = stFault.alm;

    /* ----- 调试观测通道（浮点原值，不定标） ----- */
    g_CanMon.utest1 = Bus.f32Vref;           /* 母线电压给定 */
    g_CanMon.utest2 = Bus.f32Err;            /* 滤波后电压误差 */
    g_CanMon.utest3 = Bus.f32Out;            /* 电压外环输出 */
    g_CanMon.utest4 = Bus.f32Ref;            /* 总电流给定 */
    g_CanMon.utest5 = stDabDuty0.f32Duty;    /* 第 0 路发波占空比 */
    g_CanMon.utest6 = stDabDuty1.f32Duty;    /* 第 1 路发波占空比 */
    g_CanMon.utest7 = stDabParam.f32SrRatio; /* 同步整流匝比换算系数 */
    g_CanMon.utest8 = 0.0f;                  /* 预留：原 I0+I1 低通，变量已删除 */
}
