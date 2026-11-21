/**
 * ============================================================================
 * 文件名称: App_Fault.h
 * 模块功能: DAB 故障统一（FaultDetect 角色：只置/清故障字）
 *
 *   §1 阈值与拍数  §2 位号与掩码  §3 故障字  §4 计数/快照类型与变量  §5 接口
 * ============================================================================
 */
#ifndef APP_FAULT_H
#define APP_FAULT_H

/* ==========================================================================
 * §1 阈值与拍数
 * ========================================================================== */

/* ---- 慢保护阈值（5ms 判据，单位 V / A / W；*_REC 为迟滞恢复阈值）---- */
#define FAULT_VBUS_OV              (530.0f)    /* 母线过压触发：VM > 阈值 */
#define FAULT_VBUS_OV_REC          (490.0f)    /* 母线过压恢复（迟滞 40V） */
#define FAULT_VBUS_UV              (350.0f)    /* 母线欠压触发：VM < 阈值 */
#define FAULT_VBUS_SHORT           (50.0f)     /* 短路判据：母线电压跌落门限 */
#define FAULT_IBUS_OV              (30.0f)     /* 母线过流触发：|Ibus1+Ibus2| > 阈值 */
#define FAULT_IBUS_OV_REC          (25.0f)     /* 母线过流恢复（迟滞 5A） */
#define FAULT_POWER_OV             (4740.0f)   /* 母线功率过限触发（滤波功率，W） */
#define FAULT_POWER_REC            (4000.0f)   /* 母线功率过限恢复 */
#define FAULT_I_SHORT_LO           (1.0f)      /* 短路动态阈值钳位下限：lim = |Iref|-2，见 Fault_Short */
#define FAULT_I_SHORT_HI           (4.0f)      /* 短路动态阈值钳位上限 */
#define FAULT_I_STUCK              (1.0f)      /* 关断电流异常门限：停机后 100ms 滤波电流 */

/* ---- 快保护阈值（Fault_FastProtect 在 ISR 中判据，通道 0/1 共用）---- */
#define FAULT_CH_OC                (28.0f)               /* 单通道过流触发：|I| > 阈值 */
#define FAULT_CH_OC_REC            (FAULT_CH_OC - 5.0f)  /* 单通道过流恢复（迟滞 5A） */
#define FAULT_CH_IMB_A             (2.0f)                /* 不平衡判据：电流绝对下限 */
#define FAULT_CH_IMB_RATIO         (0.30f)               /* 不平衡判据：|I0-I1| 相对 |I0+I1| 的比例 */
#define FAULT_FAST_OV_REC          (DAB_DISCHARGE_BUS_OVP_VOLT - 20.0f) /* 快过压恢复：软件触发阈值 -20V */

/* ---- CMPSS 分路恢复条件（各路独立连续 FAULT_N_CMPSS_REC 拍，见 Fault_CmpssRecTick）---- */
#define FAULT_CMPSS_V_REC          (400.0f)    /* 母线过压跳闸恢复：VM < 400V */
#define FAULT_CMPSS_I_REC          (20.0f)     /* 通道过流跳闸恢复：|I| < 20A */

/* ---- 慢保护去抖拍数（1 拍 = 5ms；带 _REC 的为恢复拍数）---- */
#define FAULT_N_VBUS_OV            (200U)     /* 过压触发 200 拍 = 1s */
#define FAULT_N_VBUS_OV_REC        (2000U)    /* 过压恢复 2000 拍 = 10s */
#define FAULT_N_VBUS_UV            (3000U)    /* 欠压触发 3000 拍 = 15s（须实际发波中） */
#define FAULT_N_VBUS_UV_REC        (3000U)    /* 欠压恢复 3000 拍 = 15s（仅计时，无电压回差条件） */
#define FAULT_N_IBUS_OV            (10U)      /* 过流触发 10 拍 = 50ms */
#define FAULT_N_IBUS_REC           (2000U)    /* 过流恢复 2000 拍 = 10s */
#define FAULT_N_POWER              (20U)      /* 功率过限触发 20 拍 = 100ms */
#define FAULT_N_POWER_REC          (2000U)    /* 功率过限恢复 2000 拍 = 10s */
#define FAULT_N_SHORT              (40U)      /* 短路触发 40 拍 = 200ms */
#define FAULT_N_SHORT_REC          (40U)      /* 短路恢复 40 拍 = 200ms */
#define FAULT_N_STUCK              (2000U)    /* 关断电流异常确认 2000 拍 = 10s（置位后不自动恢复） */
#define FAULT_N_CMPSS_REC          (2000U)    /* CMPSS 单路恢复 2000 拍 = 10s */
#define FAULT_SCI_TIMEOUT_N        (2000U)    /* SCI 空闲超时门限：空闲计数 >= 10s 判通讯超时 */
#define FAULT_N_SCI                (1U)       /* SCI 故障触发 1 拍（当拍即触发） */
#define FAULT_N_SCI_REC            (2U)       /* SCI 故障恢复 2 拍 = 10ms */

#define FAULT_N_LOCK_HIT           (3U)       /* 短路锁定门限：30s 窗口内第 3 次上升沿 → alm.lock */
/* 短路重试锁定窗口：6000 拍 × 5ms = 30 秒。必须明显大于"恢复延时
 * (FAULT_FAST_REC_N=1000) + 重启时间"，否则同一窗口内凑不满 3 次上升沿，
 * 锁定永不生效；30 秒允许约 3 次"5s 恢复 + 重启"循环后锁定。 */
#define FAULT_N_SHORT_LOCK_WIN     (6000U)
#define FAULT_N_CMPSS_LOCK_HIT     (5U)       /* CMPSS 锁定门限：5min 窗口内第 5 次上升沿 → alm.hardalm */
#define FAULT_N_CMPSS_LOCK_WIN     (60000UL)  /* CMPSS 锁定窗口：60000 拍 × 5ms = 5 分钟 */

/* ---- 软起超时（Fault_Detect5ms 内；计时 RunState.softStartTick）---- */
#define FAULT_N_SOFT_TO            (10000U)   /* 本态连续 10000×5ms=50s（EnterState 清 tick）；armed 至首次 RUN */

#define FAULT_BAT_UV_V             (35.0f)
#define FAULT_BAT_UV_REC_V         (40.0f)
#define FAULT_N_BAT_UV             (4U)       /* 3 拍，约 15ms */
#define FAULT_N_BAT_UV_REC         (1000U)    /* 1000 拍，约 5s */

/* ---- 快保护拍数（Fault_FastProtect 在 ISR 中调用，1 拍 = ISR 周期；
 *      FAST_REC_N 例外：用于 5ms 恢复任务）---- */
#define FAULT_FAST_OV_N            (3U)       /* 快过压确认 3 拍 */
#define FAULT_FAST_OC_N            (3U)       /* 通道过流确认 3 拍 */
#define FAULT_FAST_IMB_N           (300U)     /* 不平衡确认 300 拍（远宽于 OV/OC，防瞬态误锁） */
#define FAULT_FAST_REC_N           (1000U)    /* 快保护恢复延时 1000 拍 × 5ms = 5s */

/* SysCmd.bit.shutdown：1=允许运行，0=关机请求 */
#define FAULT_RUN_REQ()            (SysCmd.bit.shutdown != 0U)

/* ==========================================================================
 * §2 位号（协议位序）与掩码
 * ========================================================================== */

typedef enum
{
    FAULT_BIT_POWER_OV = 0,      /* bit0  母线功率过限（慢保护） */
    FAULT_BIT_VBUS_OV,           /* bit1  母线过压（慢保护） */
    FAULT_BIT_IBUS_OV,           /* bit2  母线过流（慢保护） */
    FAULT_BIT_VBUS_UN,           /* bit3  母线欠压 */
    FAULT_BIT_RELAY,             /* bit4  继电器异常（外部模块上报） */
    FAULT_BIT_SCI,               /* bit5  SCI 通讯超时 */
    FAULT_BIT_SHORT,             /* bit6  母线短路（动态阈值 + 电压跌落） */
    FAULT_BIT_CMPSS_VBUS_OVP,    /* bit7  CMPSS 硬件跳闸：母线过压 */
    FAULT_BIT_CMPSS_IDD1_OCP,    /* bit8  CMPSS 硬件跳闸：通道 0 过流（正/负向共用） */
    FAULT_BIT_CMPSS_IDD2_OCP,    /* bit9  CMPSS 硬件跳闸：通道 1 过流（正/负向共用） */
    FAULT_BIT_SOFT_TO,           /* bit10 软起超时（本态连续 50s 未收敛；对齐 Anker uPre_Tim_ov） */
    FAULT_BIT_BAT_UV             /* bit11 电池欠压 */
} FaultBit_e;

typedef enum
{
    FAST_BIT_VOUT_OV = 0,        /* bit0  输出母线过压（ISR 软件快保护，与 CMPSS 互补） */
    FAST_BIT_I1_OV,              /* bit1  通道 0 过流 */
    FAST_BIT_I2_OV,              /* bit2  通道 1 过流 */
    FAST_BIT_IBALANCE,           /* bit3  两通道电流不平衡 */
    FAST_BIT_COUNT               /* 快保护位总数（恢复计数数组长度） */
} FastFaultBit_e;

typedef enum
{
    ALM_BIT_LOCK = 0,            /* bit0  重复短路锁定：30s 窗口 3 次短路保护（§6 锁定） */
    ALM_BIT_HARDALM,             /* bit1  CMPSS 重复跳闸锁定：5min 窗口 5 次 */
    ALM_BIT_OFF_CURR             /* bit2  关断电流异常：停机请求后仍有电流（不自动恢复） */
} FaultAlmBit_e;

typedef char FastBitMapCheck_t[((FAST_BIT_VOUT_OV == 0) &&
                                (FAST_BIT_I1_OV == 1) &&
                                (FAST_BIT_I2_OV == 2) &&
                                (FAST_BIT_IBALANCE == 3) &&
                                (FAST_BIT_COUNT == 4)) ? 1 : -1];

#define FAULT_MASK(bit)            ((Uint32)1UL << (bit))
#define FAULT_MASK16(bit)          ((Uint16)(1U << (bit)))

#define FAST_MASK_VOUT_OV          FAULT_MASK16(FAST_BIT_VOUT_OV)
#define FAST_MASK_I1_OV            FAULT_MASK16(FAST_BIT_I1_OV)
#define FAST_MASK_I2_OV            FAULT_MASK16(FAST_BIT_I2_OV)
#define FAST_MASK_IBALANCE         FAULT_MASK16(FAST_BIT_IBALANCE)

#define ALM_MASK_LOCK              FAULT_MASK16(ALM_BIT_LOCK)
#define ALM_MASK_HARDALM           FAULT_MASK16(ALM_BIT_HARDALM)
#define ALM_MASK_OFF_CURR          FAULT_MASK16(ALM_BIT_OFF_CURR)

#define CMPSS_MASK                 (FAULT_MASK(FAULT_BIT_CMPSS_VBUS_OVP) | \
                                    FAULT_MASK(FAULT_BIT_CMPSS_IDD1_OCP) | \
                                    FAULT_MASK(FAULT_BIT_CMPSS_IDD2_OCP))
                                    /* slow 字中 CMPSS 硬件跳闸三位合并掩码（锁定事件源用） */

/* ==========================================================================
 * §3 故障数据（仅整数字）
 * ========================================================================== */

typedef struct
{
    Uint32 slow;   /* 慢保护 32 位 */
    Uint16 fast;   /* 快保护 16 位 */
    Uint16 alm;    /* lock / hardalm / offCurr */
} FaultData_t;

extern volatile FaultData_t stFault;

#define FAULT_ANY()   ((stFault.slow != 0UL) || (stFault.fast != 0U) || \
                       (stFault.alm != 0U))
                      /* 任一故障字非零（含 lock / hardalm / offCurr） */

/* ==========================================================================
 * §4 计数 / 快照类型与变量（供 CCS 观察；定义在 App_Fault.c）
 * ========================================================================== */

typedef struct
{
    Uint16 trip;   /* 触发方向计数：条件连续成立拍数（断即清零） */
    Uint16 rec;    /* 恢复方向计数：安全条件连续成立拍数（断即清零） */
} FaultTick_t;

typedef struct
{
    Uint16 hits;   /* 窗口内累计的上升沿次数（达门限即锁定清零） */
    Uint32 win;    /* 窗口已流逝拍数（自首次上升沿起算，固定窗口） */
} RetryLock_t;

typedef struct
{
    float  f32Vbus;      /* 继电器 VM 母线电压：过压/欠压/短路/CMPSS 恢复统一判据
                          * （VBUS1/2 只用于功率计算，不参与保护） */
    float  f32Iabs;      /* |Ibus1 + Ibus2|：母线总电流绝对值 */
    float  f32Islow;     /* 100ms 滤波总电流绝对值：关断电流异常判据 */
    float  f32I0;        /* 通道 0 电流：CMPSS 过流恢复判据 */
    float  f32I1;        /* 通道 1 电流：CMPSS 过流恢复判据 */
    Uint16 u16PwmOn;     /* 本拍 PWM 是否有效输出（电流类判据门控） */
    Uint16 u16OpenTest;  /* 本拍是否开路测试模式（欠压/SCI 判据门控） */
} FaultSnap_t;

typedef struct
{
    FaultTick_t batUv;      /* 电池欠压迟滞计数 */
    FaultTick_t vbusOv;     /* 慢过压迟滞计数 */
    FaultTick_t ibusOv;     /* 慢过流迟滞计数 */
    FaultTick_t powerOv;    /* 功率过限迟滞计数 */
    FaultTick_t vbusUn;     /* 欠压迟滞计数 */
    FaultTick_t busShort;   /* 短路迟滞计数 */
    FaultTick_t sci;        /* SCI 超时迟滞计数 */
    Uint16      offCurrHit; /* 关断电流异常连续命中计数 */

    Uint16      fastOvHit;  /* ISR：快过压连续命中计数 */
    Uint16      fastI1Hit;  /* ISR：通道 0 过流连续命中计数 */
    Uint16      fastI2Hit;  /* ISR：通道 1 过流连续命中计数 */
    Uint16      fastImbHit; /* ISR：不平衡连续命中计数 */
    Uint16      fastRec[FAST_BIT_COUNT];   /* 5ms：快保护各位恢复计数（5s 延时） */

    Uint16      cmpssVRec;  /* CMPSS 母线过压位恢复计数（连续 10s） */
    Uint16      cmpssI1Rec; /* CMPSS 通道 0 过流位恢复计数 */
    Uint16      cmpssI2Rec; /* CMPSS 通道 1 过流位恢复计数 */

    Uint16      u16ShortPrev;   /* 上一拍短路位（§6 上升沿计次用） */
    Uint32      u32CmpssPrev;   /* 上一拍 CMPSS 事件字（§6 上升沿计次用） */
} FaultRunCnt_t;

typedef struct
{
    RetryLock_t shortLock;      /* 短路重试：30s 窗口 3 次上升沿 → alm.lock */
    RetryLock_t cmpssLock;      /* CMPSS 重试：5min 窗口 5 次上升沿 → alm.hardalm */
} FaultRetryCnt_t;

typedef struct
{
    FaultRunCnt_t   run;
    FaultRetryCnt_t retry;
} FaultCnt_t;

extern FaultCnt_t stFaultCnt;

/* ==========================================================================
 * §5 对外接口
 * ========================================================================== */

void Fault_Init(void);
void Fault_Detect5ms(void);
void Fault_SetRelay(void);
void Fault_ClearRelay(void);
void Fault_ClearSoftTo(void);
Uint16 Fault_PreStartHwOk(void);
void Fault_FastProtect(float f32BusCurr0, float f32BusCurr1);

#endif /* APP_FAULT_H */
