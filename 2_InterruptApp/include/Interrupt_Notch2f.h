/*************************************************************************************************
 * Interrupt_Notch2f.h — 二倍工频限波器（陷波器）
 *
 * 干什么：母线电压上叠着 100Hz 纹波（工频 50Hz 的两倍）。本模块先估出这坨
 *         100Hz，再从采样里减掉，得到相对干净的直流，给能量环用。
 *
 * 怎么用：平时只改两个数即可（5ms 任务会自动重算系数）
 *         g_Notch2f.f32LineHz = 50;   // 电网频率，中国 50、美国 60；内部会陷 2 倍频
 *         g_Notch2f.f32K      = 0.1;  // 挖坑宽窄；越大越宽越稳，越小越尖越怕偏频
 *
 * 谁何时调：
 *   Notch2f_Init()           — 上电、开中断前调一次
 *   Notch2f_ParamUpdate5ms() — 后台 5ms 调；参数没变就立刻返回
 *   Notch2f_Isr() — 由PWM ISR统一按10kHz错峰调用，仅执行一次计算
 *                              u16OuterTick≠0 表示本拍刚算完电压外环，陷波顺延一拍
 *
 * 时基：固定 100us（10kHz），和能量外环同频但**不同拍**。充电 5 个 PWM、放电
 *       6 个 PWM 凑满同一门槛，所以充放共用一套系数，换向不用换表。
 *       外环那拍只累加不运算，把两坨浮点错开到相邻中断，避免 CPU 峰值叠加。
 *************************************************************************************************/
#ifndef INCLUDE_ISR_NOTCH2F_H
#define INCLUDE_ISR_NOTCH2F_H

#define NOTCH2F_TS_S        (100.0e-6f)          /* 每拍间隔 100us */
#define NOTCH2F_K_DEFAULT   (0.1f)               /* 默认挖宽约 ±10Hz@100Hz */
#define NOTCH2F_LINE_HZ_DEF (50.0f)              /* 默认工频；填 50 不是填 100 */

#if ((5U * DAB_CHG_TBPRD) != (6U * DAB_DIS_TBPRD))
#error "Notch2f: 充放折算不成同一 100us，不能共用系数"
#endif

/* 算好的一组系数；后台算，中断只读着用 */
typedef struct
{
    float32 f32K; /* 当前银行的 k；改 k 时用来平滑内部状态 */
    float32 f32H; /* 振荡一步走多远（定中心频率） */
    float32 f32A; /* 旧 Band 衰减 */
    float32 f32B; /* Band↔Quad 互相推动 */
    float32 f32C; /* 新采样灌进 Band 的量 */
} ST_NOTCH2F_COEFF;

/* 对外参数块：两套系数轮换，避免中断读到写了一半的数 */
typedef struct
{
    float32 f32LineHz; /* 【可调】工频 Hz，合法 40~70 */
    float32 f32K;      /* 【可调】带宽，合法 (0,2) */

    volatile ST_NOTCH2F_COEFF stBank0;
    volatile ST_NOTCH2F_COEFF stBank1;
    volatile Uint16 u16Pub; /* 0→中断读银行0；1→读银行1 */

    float32 f32LineHzApplied; /* 上次已生效的工频；没变就不重算 */
    float32 f32KApplied;      /* 上次已生效的 k */
} ST_NOTCH2F_PARAM;

extern ST_NOTCH2F_PARAM g_Notch2f;

extern void Notch2f_Init(void);
extern void Notch2f_ParamUpdate5ms(void);
extern void Notch2f_Isr(void); /* 结果写入各通道 stADC.*.f32Notch */

#endif /* INCLUDE_INTERRUPT_NOTCH2F_H */
