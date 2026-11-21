/*************************************************************************************************
 * ConfigChip_AdcInit.h — F28P550SG DAB ADC 底层 SOC 槽位
 *
 * 依据：《6.2~10KWH_DSP资源配置表_F28P550SG》管脚配置
 *
 * 【触发】
 *   trig1 = EPWM10 SOCA @ PRD  → 桥1 同步采样
 *   trig2 = EPWM7  SOCA @ PRD  → 桥2 同步采样（相对桥1 滞后 90°）
 *   采样窗口 ACQPS = DAB_ADC_ACQPS(24)
 *   控制 ISR 挂 EPWM10 ZERO：读 RESULT 时本拍转换已完成
 *
 * 【均分原则】16 路分到 ADCA/B/C，缩短单模块串行转换链
 *   ADCA 5 路 | ADCB 5 路 | ADCC 6 路
 *   可走并联脚的信号迁到 ADCC（资源表同网：A10↔C10、B2↔C6 等）
 *
 * 【电流首采 — 硬性要求】
 *   ADCA SOC0 = IDD1（A7，trig1）  ← 桥1 电流最先转
 *   ADCB SOC0 = IDD2（B8，trig2）  ← 桥2 电流最先转（资源表 A4/B8 并联，采 B8）
 *
 * 【SOC → 通道一览】
 *   ADCA: 0 A7  IDD1_ADC      1 A12 VMBUS_ADC   2 A6  PACK_NTC
 *         3 A5  IDD1_OCP      4 A8  IDD2_OCP
 *   ADCB: 0 B8  IDD2_ADC      1 B11 VBUS2_ADC   2 B4  VPBUS_ADC（PCS 端电压）
 *         3 B5  VMBUS_OVP     4 B3  MODEL1
 *   ADCC: 0 C1  VBUS1_ADC     1 C10 VDC_ADC     2 C6  HV_NTC_ADC
 *         3 C16 BATT_NTC_ADC  4 C9  MODEL2      5 C7  MODEL3
 *
 * 应用层读数请用 App_AdcDataDeal.h 的 ADRESULT_*（与本槽位绑定）。
 *************************************************************************************************/
#ifndef BSP_ADCINIT_H_
#define BSP_ADCINIT_H_

#ifndef DAB_ADC_ACQPS
#define DAB_ADC_ACQPS               (24U)   /* 采样窗：SYSCLK 计数 */
#endif

/* ========== ADCA RESULT 槽（SOC 序号 = 宏值）；SOC0 必须是 IDD1 ========== */
#define DAB_ADC_A_IDD1_ADC          (0U)    /* A7  DAB1 电流，EPWM10 首采 */
#define DAB_ADC_A_VMBUS_ADC         (1U)    /* A12 继电器侧 VM 母线 */
#define DAB_ADC_A_PACK_NTC_ADC      (2U)    /* A6  电池包 NTC */
#define DAB_ADC_A_IDD1_OCP          (3U)    /* A5  DAB1 过流监测 / CMPSS */
#define DAB_ADC_A_IDD2_OCP          (4U)    /* A8  DAB2 过流监测 / CMPSS */

/* ========== ADCB RESULT 槽；SOC0 必须是 IDD2@B8 ========== */
#define DAB_ADC_B_IDD2_ADC          (0U)    /* B8  DAB2 电流，EPWM7 首采 */
#define DAB_ADC_B_VBUS2_ADC         (1U)    /* B11 母线2（跟 EPWM7） */
#define DAB_ADC_B_VPBUS_ADC         (2U)    /* B4  VPBUS_ADC = PCS 端电压 */
#define DAB_ADC_B_VMBUS_OVP_ADC     (3U)    /* B5  母线快速电压 / CMPSS1 */
#define DAB_ADC_B_MODEL1            (4U)    /* B3  版本检测 */

/* ========== ADCC RESULT 槽：均分负载，并联脚取 C 通道 ========== */
#define DAB_ADC_C_VBUS1_ADC         (0U)    /* C1  母线1 */
#define DAB_ADC_C_VDC_ADC           (1U)    /* C10 电池电压（同网 A10） */
#define DAB_ADC_C_HV_NTC_ADC        (2U)    /* C6  高压侧温度（同网 B2） */
#define DAB_ADC_C_BATT_NTC_ADC      (3U)    /* C16 电池侧温度（同网 A16） */
#define DAB_ADC_C_MODEL2            (4U)    /* C9  版本检测（同网 B6） */
#define DAB_ADC_C_MODEL3            (5U)    /* C7  版本检测（同网 B9） */

/*
 * 读 ADC 结果寄存器：ADCRESULT0 + idx
 * - ADCRESULT0..15 在结果块内连续存放，C28x 为字寻址，+idx 即第 idx 个结果寄存器；
 * - idx 取上面 DAB_ADC_{A,B,C}_* 槽号（必须是 0~15 的编译期常量）；
 * - 每次展开 = 一次 volatile 16 位寄存器读，不做任何换算。
 */
#define DAB_ADC_RESULT(baseRegs, idx) \
    (*((volatile Uint16 *)((&(baseRegs).ADCRESULT0) + (Uint16)(idx))))

extern void InitChip_AdcRegs(void);
extern void InitChip_DMARegs(void);

#endif /* BSP_ADCINIT_H_ */
