/*************************************************************************************************
 * ConfigChip_DabPwmMap.h  —  DAB 逻辑 PWM1~4 ↔ 硬件 ePWM / GPIO（F28P550SG 最新硬件）
 *
 * 依据：
 *   - 《6.2~10KWH_DSP资源配置表_F28P550SG.xlsx》管脚配置
 *   - 手绘拓扑：桥1 领先；桥2 同配、滞后桥1 90°
 *   - 工程发波逻辑不变：PWM1=BUS左(S1/S2) PWM2=BUS右(S3/S4)
 *                      PWM3=BAT左(S5/S6) PWM4=BAT右(S7/S8)
 *
 * 桥1(0°) : EPWM10 / 12 / 11 / 3   ← GLD 主机 EPWM10（本桥 PWM1）
 * 桥2(90°): EPWM7  / 9  / 2  / 4   ← GLD 主机 EPWM7 （本桥 PWM1）；相位由 EPWM7←EPWM10@90°
 *
 * 禁止两桥共用一个 GLD：CMP/AQ 必须在各自载波 ZERO 装载，否则 90° 错位易直通。
 *************************************************************************************************/
#ifndef BSP_DAB_PWM_MAP_H_
#define BSP_DAB_PWM_MAP_H_

#include "f28x_project.h"

/* ---------- 桥1：逻辑 PWM1~4（ISR 先写这四组，再镜像到桥2） ---------- */
#define DabEpwm1Regs            EPwm10Regs      /* PWM1 BUS左  S1/S2 */
#define DabEpwm2Regs            EPwm12Regs      /* PWM2 BUS右  S3/S4 */
#define DabEpwm3Regs            EPwm11Regs      /* PWM3 BAT左  S5/S6 */
#define DabEpwm4Regs            EPwm3Regs       /* PWM4 BAT右  S7/S8 */

/* ---------- 桥2：同角色，相位落后 90° ---------- */
#define DabEpwm1bRegs           EPwm7Regs       /* 桥2 PWM1 / GLD 主机 */
#define DabEpwm2bRegs           EPwm9Regs
#define DabEpwm3bRegs           EPwm2Regs
#define DabEpwm4bRegs           EPwm4Regs

/* XLINK 编号：EPWM1=0 … EPWM12=11（仅桥内自链 / GLD 分组，禁止跨桥 CMP 链） */
#define DAB_XLINK_EPWM10        (9U)
#define DAB_XLINK_EPWM12        (11U)
#define DAB_XLINK_EPWM11        (10U)
#define DAB_XLINK_EPWM3         (2U)
#define DAB_XLINK_EPWM7         (6U)
#define DAB_XLINK_EPWM9         (8U)
#define DAB_XLINK_EPWM2         (1U)
#define DAB_XLINK_EPWM4         (3U)

/* 桥1 主模块：ISR / SOCA / 桥1 SYNCOUT / 桥1 GLD */
#define DAB_EPWM_MASTER_REGS    DabEpwm1Regs
/* 桥2 主模块：桥2 SYNCOUT / 桥2 GLD */
#define DAB_EPWM_MASTER2_REGS   DabEpwm1bRegs
#define DAB_ADC_TRIG_SOCA       ((Uint16)ADC_TRIGGER_EPWM10_SOCA) /* 桥1 */
#define DAB_ADC_TRIG_SOCA_B2    ((Uint16)ADC_TRIGGER_EPWM7_SOCA)  /* 桥2 */

/* 同步源 */
#ifndef SYNC_IN_SRC_SYNCOUT_EPWM10
#define SYNC_IN_SRC_SYNCOUT_EPWM10   (0xAU)
#endif
#ifndef SYNC_IN_SRC_SYNCOUT_EPWM7
#define SYNC_IN_SRC_SYNCOUT_EPWM7    (0x7U)
#endif

/* 90°：up-down 下一载波周期=2*TBPRD，四分之一周期=TBPRD/2 */
#define DAB_PHASE_90_FROM_PRD(prd)   ((Uint16)((prd) >> 1))

/* ---------- 桥1 GPIO ---------- */
#define DAB_GPIO_DH1_LH         (62U)   /* EPWM10_A PWM_DR_DH1_LH */
#define DAB_GPIO_DH1_LL         (63U)   /* EPWM10_B */
#define DAB_GPIO_DH1_RH         (20U)   /* EPWM12_A */
#define DAB_GPIO_DH1_RL         (21U)   /* EPWM12_B */
#define DAB_GPIO_DL1_LH         (8U)    /* EPWM11_A */
#define DAB_GPIO_DL1_LL         (4U)    /* EPWM11_B */
#define DAB_GPIO_DL1_RH         (0U)    /* EPWM3_A  */
#define DAB_GPIO_DL1_RL         (1U)    /* EPWM3_B  */

/* ---------- 桥2 GPIO ---------- */
#define DAB_GPIO_DH2_LH         (12U)   /* EPWM7_A */
#define DAB_GPIO_DH2_LL         (13U)   /* EPWM7_B */
#define DAB_GPIO_DH2_RH         (16U)   /* EPWM9_A */
#define DAB_GPIO_DH2_RL         (17U)   /* EPWM9_B */
#define DAB_GPIO_DL2_LH         (41U)   /* EPWM2_A */
#define DAB_GPIO_DL2_LL         (40U)   /* EPWM2_B */
#define DAB_GPIO_DL2_RH         (22U)   /* EPWM4_A */
#define DAB_GPIO_DL2_RL         (7U)    /* EPWM4_B */

/* ---------- 板级通信引脚（资源配置表；数字 DO 见 App_Gpio.h） ---------- */
#define BOARD_GPIO_SCIB_TX      (14U)   /* ARM 通讯 SCIB_TX */
#define BOARD_GPIO_SCIB_RX      (15U)   /* ARM 通讯 SCIB_RX */
#define BOARD_GPIO_MCAN_TX      (31U)
#define BOARD_GPIO_MCAN_RX      (30U)

/**
 * 桥2 影子寄存器镜像桥1（CMP + AQ；不跨桥 XLINK，由各自 GLD 在本桥 ZERO 装载）。
 * 仅初始化时调用一次，用于把桥1 的静态 AQ 底座（关断沿 CLEAR）复制给桥2；
 * 运行期两桥各自按自己的占空比写 CMP/AQ，不再镜像。
 */
void DabPwm_MirrorShadowBridge2(void);

#endif /* BSP_DAB_PWM_MAP_H_ */
