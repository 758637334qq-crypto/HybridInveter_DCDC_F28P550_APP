/*************************************************************************************************
 * App_AdcDataDeal.h — ADC 应用层：RESULT 别名 + 采样池 + 三档慢滤波系数
 *
 * 组成：
 *   1) ADRESULT_*  —— 结果寄存器读值别名，槽号绑定 ConfigChip_AdcInit.h 的 DAB_ADC_*；
 *   2) ST_ADC_POOL —— 统一采样池：ST_DC 采样数据 + ST_ADC_CURR_OFFSET 校准参数；
 *   3) ADC_DAB_FILT_* —— 1ms 三档低通系数（τ=10/100/400ms）。
 *
 * 数据流：SOC 转换 → ISR 读 ADRESULT_*（本拍瞬时）→ 1ms AdcDabFilt1ms（三档滤波）
 *         → Notch2f_Isr（陷波）→ 控制/保护/遥测读取 stADC。
 *************************************************************************************************/

#ifndef APP_ADCDATADEAL_H_
#define APP_ADCDATADEAL_H_

#include "ConfigChip_AdcInit.h"

/*
 * ---------------------------------------------------------------------------
 * ADRESULT_*：ADC 结果寄存器读值别名（Uint16 原始码，0~4095，12 位右对齐）
 *
 *   ADRESULT_xxx = DAB_ADC_RESULT(结果寄存器块, Bsp SOC 槽号)
 *
 * - 槽号定义在 ConfigChip_AdcInit.h（DAB_ADC_{A,B,C}_*），与《6.2~10KWH_DSP 资源配置表》一致；
 *   改底层 SOC 顺序只需改 Bsp 槽宏，本文件展开式自动跟随。
 * - 每个宏求值 = 一次 16 位结果寄存器读（volatile，不缓存、不运算）；
 *   物理量换算在 ISR 用比例宏完成：V = 码 × RATIO，I = (码 − 零偏) × RATIO。
 * - 触发源：默认 EPWM10 SOCA；ADCB 的 IDD2/VBUS2 跟 EPWM7 SOCA（桥 2 时基，滞后 90°）。
 * - “首采”= 本模块 SOC0，转换排队最早；两桥电流都放在各自 SOC0，
 *   使各桥电流在本桥时基 PRD 处最先转换（中点偏置校准也读这两路）。
 * - 未入 stADC 的槽（NTC / OCP / OVP / MODEL）保留别名，供诊断或后续扩展使用。
 * ---------------------------------------------------------------------------
 */

/* ---------- ADCA（5 路）：EPWM10 SOCA 触发 ---------- */
#define ADRESULT_IDD1_ADC       DAB_ADC_RESULT(AdcaResultRegs, DAB_ADC_A_IDD1_ADC)      /* SOC0 A7  | 桥1 电流 IDD1（首采）→ stADC.BusCurr1 */
#define ADRESULT_VMBUS_ADC      DAB_ADC_RESULT(AdcaResultRegs, DAB_ADC_A_VMBUS_ADC)     /* SOC1 A12 | 母线电压 VM（继电器侧，控制主反馈）→ stADC.VmBusVolt */
#define ADRESULT_PACK_NTC_ADC   DAB_ADC_RESULT(AdcaResultRegs, DAB_ADC_A_PACK_NTC_ADC)  /* SOC2 A6  | 电池包温度 NTC（未入池） */
#define ADRESULT_IDD1_OCP       DAB_ADC_RESULT(AdcaResultRegs, DAB_ADC_A_IDD1_OCP)      /* SOC3 A5  | 桥1 过流检测 IDD1_OCP（未入池） */
#define ADRESULT_IDD2_OCP       DAB_ADC_RESULT(AdcaResultRegs, DAB_ADC_A_IDD2_OCP)      /* SOC4 A8  | 桥2 过流检测 IDD2_OCP（未入池） */

/* ---------- ADCB（5 路）：IDD2/VBUS2 跟 EPWM7 SOCA（桥2 滞后 90°），其余 EPWM10 SOCA ---------- */
#define ADRESULT_IDD2_ADC       DAB_ADC_RESULT(AdcbResultRegs, DAB_ADC_B_IDD2_ADC)      /* SOC0 B8  | 桥2 电流 IDD2（首采）→ stADC.BusCurr2 */
#define ADRESULT_VBUS2_ADC      DAB_ADC_RESULT(AdcbResultRegs, DAB_ADC_B_VBUS2_ADC)     /* SOC1 B11 | 母线2 电压 → stADC.BusVolt2 */
#define ADRESULT_PCS_VBUS       DAB_ADC_RESULT(AdcbResultRegs, DAB_ADC_B_VPBUS_ADC)     /* SOC2 B4  | PCS 端母线 VPBUS → stADC.PcsBusVolt */
#define ADRESULT_VMBUS_OVP_ADC  DAB_ADC_RESULT(AdcbResultRegs, DAB_ADC_B_VMBUS_OVP_ADC) /* SOC3 B5  | 母线过压检测 VMBUS_OVP（未入池） */
#define ADRESULT_MODEL1         DAB_ADC_RESULT(AdcbResultRegs, DAB_ADC_B_MODEL1)        /* SOC4 B3  | 机型/配置识别脚 1（未入池） */

/* ---------- ADCC（6 路）：EPWM10 SOCA 触发（并联脚走 C，均分转换负载） ---------- */
#define ADRESULT_VBUS1_ADC      DAB_ADC_RESULT(AdccResultRegs, DAB_ADC_C_VBUS1_ADC)     /* SOC0 C1  | 母线1 电压 → stADC.BusVolt1 */
#define ADRESULT_VDC_ADC        DAB_ADC_RESULT(AdccResultRegs, DAB_ADC_C_VDC_ADC)       /* SOC1 C10 | 电池电压 VDC → stADC.BatVolt */
#define ADRESULT_HV_NTC_ADC     DAB_ADC_RESULT(AdccResultRegs, DAB_ADC_C_HV_NTC_ADC)    /* SOC2 C6  | 高压侧温度 NTC（未入池） */
#define ADRESULT_BATT_NTC_ADC   DAB_ADC_RESULT(AdccResultRegs, DAB_ADC_C_BATT_NTC_ADC)  /* SOC3 C16 | 电池温度 NTC（未入池） */
#define ADRESULT_MODEL2         DAB_ADC_RESULT(AdccResultRegs, DAB_ADC_C_MODEL2)        /* SOC4 C9  | 机型/配置识别脚 2（未入池） */
#define ADRESULT_MODEL3         DAB_ADC_RESULT(AdccResultRegs, DAB_ADC_C_MODEL3)        /* SOC5 C7  | 机型/配置识别脚 3（未入池） */

/*
 * 1ms 任务一阶低通：y += NEW * (x - y)（保证稳态直流增益严格为 1）
 * NEW = 1-exp(-Ts/τ)，OLD = 1-NEW，Ts = 1ms
 */
#define ADC_DAB_FILT_10MS_NEW    (0.09516f)   /* τ=10ms */
#define ADC_DAB_FILT_10MS_OLD    (0.90484f)
#define ADC_DAB_FILT_100MS_NEW   (0.009950f)  /* τ=100ms */
#define ADC_DAB_FILT_100MS_OLD   (0.990050f)
#define ADC_DAB_FILT_400MS_NEW   (0.002497f)  /* τ=400ms */
#define ADC_DAB_FILT_400MS_OLD   (0.997503f)

/** 三档一阶低通状态：1ms 任务用同一瞬时值并行推进 */
typedef struct {
    float32     f32Filt10ms;    /* τ=10ms  → 控制内环/快保护 */
    float32     f32Filt100ms;   /* τ=100ms → 均流/慢环 */
    float32     f32Filt400ms;   /* τ=400ms → 遥测/功率估算 */
} ST_ADC_LPF3;

/** 单路直流量：瞬时（ISR 写）+ 三档低通（1ms 写）+ 陷波（Notch2f 写） */
typedef struct
{
    float32     f32IsrResult;   /* 本拍物理量（已按 RATIO 换算，V 或 A） */
    ST_ADC_LPF3 Filt;           /* 三档滤波输出：后台任务与慢环/前馈读 */
    float32     f32Notch;       /* 去二倍频结果，仅供控制环读 */
} ST_DC;

/* ---------- 校准量（非采样数据）：电流零点偏置（ADC 码） ---------- */
/** 零流校准写入，ISR 用 (RESULT − offset) × RATIO；默认 BUS_CURR_MID_ADC */
typedef struct
{
    float32     f32Idd1;        /* ADRESULT_IDD1_ADC 零点（ADCA RESULT0） */
    float32     f32Idd2;        /* ADRESULT_IDD2_ADC 零点（ADCB RESULT0 / B8） */
} ST_ADC_CURR_OFFSET;

/* ---------- 统一 ADC 数据池 ---------- */
typedef struct
{
    /* —— 采样数据：ISR 写瞬时；1ms 写三档滤波；Notch2f 写陷波 —— */
    ST_DC       BusVolt1;       /* ADRESULT_VBUS1_ADC  母线1 */
    ST_DC       BusVolt2;       /* ADRESULT_VBUS2_ADC  母线2 */
    ST_DC       VmBusVolt;      /* ADRESULT_VMBUS_ADC  继电器 VM */
    ST_DC       PcsBusVolt;     /* ADRESULT_PCS_VBUS   PCS 端电压 */
    ST_DC       BusCurr1;       /* ADRESULT_IDD1_ADC   桥1 电流（首采） */
    ST_DC       BusCurr2;       /* ADRESULT_IDD2_ADC   桥2 电流（首采） */
    ST_DC       BatVolt;        /* ADRESULT_VDC_ADC    电池电压 */
    ST_DC       BatCurr;        /* 低压电流粗估 ibus*vbus/vbat */

    float32     f32BusPowerFilt; /* PCS×|Ibus| 400ms 功率粗估 */

    /* —— 校准参数：上电/零流校准写入，ISR 只读 —— */
    ST_ADC_CURR_OFFSET CurrOffset;
} ST_ADC_POOL;

extern volatile ST_ADC_POOL stADC;

#ifndef ADC_CURR_ZERO_N
#define ADC_CURR_ZERO_N             (100U)  /* 零点平均次数 */
#endif

extern void AdcCtrlInit(void);
extern void AdcDabFilt1ms(void);

/**
 * 电流过零点校准：PWM 时基已开、功率封波时调用。
 * 对 IDD1/IDD2 各采 ADC_CURR_ZERO_N 次取平均，写入 stADC.CurrOffset。
 */
extern void Adc_CalibCurrZero(void);

#endif /* APP_ADCDATADEAL_H_ */
