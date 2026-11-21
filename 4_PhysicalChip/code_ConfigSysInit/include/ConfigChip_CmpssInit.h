/*************************************************************************************************
 * ConfigChip_CmpssInit.h  —  CMPSS 硬件快保护（F28P550SG）
 *
 * 参考：HTH2606_PVAC_new/App/App_BoardCfg.c → CMPSS_init / EPWMXBAR_init
 * 依据：《6.2~10KWH_DSP资源配置表_F28P550SG.xlsx》
 *   CMPSS1  B5 / CMP1_HP5      → VMBUS_OVP（单向上限：只防过压）
 *   CMPSS2  A5 / CMP2_HP5/LP5  → IDD1_OCP ±30 A（窗口：正负都防）
 *   CMPSS4  A8 / CMP4_HP4/LP4  → IDD2_OCP ±30 A（窗口：正负都防）
 *
 * 电流：中点偏置 → DACH=MID+Δ（正）、DACL=MID−Δ + Low 取反（负）；
 *       XBAR 用 CTRIPH_OR_L。DAC 暂按 BUS_CURR_RATIO 估算，上板重标。
 *       （"上板重标"= 阈值是理论算的，需用实际采样链路校准后再定稿）
 *************************************************************************************************/
#ifndef BSP_CMPSS_INIT_H_       // 头文件保护：防止被重复包含导致重定义
#define BSP_CMPSS_INIT_H_

#include "f28x_project.h"       // 芯片头文件：提供 Uint16 等类型定义

/* =============================================================================================
 * 一、母线过压阈值
 * 公式：DAC码 = V_trip / k_v ，k_v = 0.16716926 V/码（采样链路比例系数）
 * 计算：530 / 0.16716926 ≈ 3170 → 对应 DAC 引脚约 2.555 V
 * 含义：母线超过约 530 V，高比较器翻转 → 触发 TRIP4 → 封锁 PWM
 * 用 #ifndef 包裹：允许在工程设置或别处预定义来覆盖，方便上板调试不改头文件
 * ============================================================================================= */
/* 约 530 V / 0.16716926 ≈ 3170 */
#ifndef DAB_CMPSS_VBUS_OVP_DAC
#define DAB_CMPSS_VBUS_OVP_DAC      (3170U)     // 12位 DAC 码，范围 0~4095
#endif

/* =============================================================================================
 * 二、数字滤波参数（作用：滤掉毛刺，防误触发；代价：保护响应变慢）
 *
 * 原理（多数表决）：每隔 Ts 采样一次比较器输出，在最近 N(窗) 次里
 *                   有 ≥ M(门限) 次为"1"，才确认故障。
 *   Ts       = (prd+1) / 150MHz
 *   最快确认 = M × Ts ； 最慢确认 = N × Ts ； 必须 M > N/2
 *
 *   OVP：prd=100，窗32/门限24 → Ts≈0.67 µs，约 16~22 µs
 *        （母线电压变化慢，可以滤得稳一点）
 *   OCP：prd=40， 窗10/门限6  → Ts≈0.28 µs，约 1.6~2.8 µs
 *        （电流上升快，必须更快；但太快又会被开关噪声误触发）
 * ============================================================================================= */
#ifndef DAB_CMPSS_VBUS_OVP_FILT_PRD
#define DAB_CMPSS_VBUS_OVP_FILT_PRD (100U)      // OVP 采样分频：越大越慢越稳
#endif

#ifndef DAB_CMPSS_IDD_OCP_FILT_PRD
#define DAB_CMPSS_IDD_OCP_FILT_PRD  (40U)       // OCP 采样分频：小，响应快
#endif

/* =============================================================================================
 * 三、电流窗口阈值（双极性：正向 + 负向）
 *
 * 采样电路：0 A 时输出中点电压（约 1.65 V），电流正负分别在中点上下摆动
 *
 *   电压(DAC码)
 *     3398 ─────────  ← POS_DAC  (+30 A)  Vin > 此值 → 高比较器触发
 *     2048 ─ ─ ─ ─ ─  ← MID_DAC  ( 0 A )  中点
 *      698 ─────────  ← NEG_DAC  (−30 A)  Vin < 此值 → 低比较器(取反)触发
 *
 * 公式：Δ = I_trip / k_i ，k_i = 0.02221679 A/码
 *       POS = MID + Δ ； NEG = MID − Δ （对称）
 * ============================================================================================= */
#ifndef DAB_CMPSS_IDD_MID_DAC
#define DAB_CMPSS_IDD_MID_DAC       (2048U)     // 0 A 对应码值 = 4096/2（中点 ≈ 1.65 V）
#endif

/* 暂定 ±30 A；Δ ≈ 30 / 0.02221679 ≈ 1350 */
#ifndef DAB_CMPSS_IDD_OCP_DELTA_DAC
#define DAB_CMPSS_IDD_OCP_DELTA_DAC (1350U)     // 30 A 对应的码数；改保护电流只改这个
#endif

/* 正向阈值 = 中点 + Δ = 2048 + 1350 = 3398（约 2.74 V） */
#ifndef DAB_CMPSS_IDD_OCP_POS_DAC
#define DAB_CMPSS_IDD_OCP_POS_DAC \
    ((Uint16)(DAB_CMPSS_IDD_MID_DAC + DAB_CMPSS_IDD_OCP_DELTA_DAC))
#endif

/* 负向阈值 = 中点 − Δ = 2048 − 1350 = 698（约 0.56 V）
 * 因为 Low 比较器配了 INV 取反，所以 Vin < NEG_DAC 时才输出 1 */
#ifndef DAB_CMPSS_IDD_OCP_NEG_DAC
#define DAB_CMPSS_IDD_OCP_NEG_DAC \
    ((Uint16)(DAB_CMPSS_IDD_MID_DAC - DAB_CMPSS_IDD_OCP_DELTA_DAC))
#endif

/* =============================================================================================
 * 四、故障源位定义（每路一位，可按位或，应用层用 & 判断）
 * 因为三路比较器"或"到同一个 TRIP4，ePWM 侧分不清是谁触发的，
 * 所以清故障前读 CMPSS 状态，把触发源记在这 5 个位里：
 *
 *   bit4  bit3  bit2  bit1  bit0
 *   I2负  I2正  I1负  I1正  VBUS
 * ============================================================================================= */
#define DAB_CMPSS_SRC_VBUS_OVP      (1U << 0)   /* CMPSS1 高比较器：母线过压 */
#define DAB_CMPSS_SRC_IDD1_POS      (1U << 1)   /* CMPSS2 高比较器：电流1 正过流 */
#define DAB_CMPSS_SRC_IDD1_NEG      (1U << 2)   /* CMPSS2 低比较器：电流1 负过流 */
#define DAB_CMPSS_SRC_IDD2_POS      (1U << 3)   /* CMPSS4 高比较器：电流2 正过流 */
#define DAB_CMPSS_SRC_IDD2_NEG      (1U << 4)   /* CMPSS4 低比较器：电流2 负过流 */

/* =============================================================================================
 * 五、对外接口
 * ============================================================================================= */
void InitChip_CmpssRegs(void);      // 总初始化：比较器 + XBAR + 8路ePWM挂接，上电调用一次

/**
 * 一次读三路 CMPSS STS，返回 DAB_CMPSS_SRC_* 位图（不清硬件锁存）。
 * Fault 5ms 登记与起机复核共用。
 */
Uint16 DabCmpss_ReadTripSrc(void);

/**
 * 清滤波锁存与 ePWM TZ。
 * 调用前应确认模拟量已恢复，否则清完可能立刻再跳。
 */
void DabCmpss_ClearTrip(void);

/** 仅清 CMPSS 滤波锁存（不碰 ePWM TZ）；全路 */
void DabCmpss_ClearFilterLatch(void);

/** 按 DAB_CMPSS_SRC_* 位图只清对应路锁存（分路恢复用，避免误清他路） */
void DabCmpss_ClearFilterLatchSrc(Uint16 u16Src);

#endif /* BSP_CMPSS_INIT_H_ */
