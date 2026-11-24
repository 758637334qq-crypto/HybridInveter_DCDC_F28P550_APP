#ifndef APP_DAB_MODEL_H
#define APP_DAB_MODEL_H

#include "ConfigChip_EpwmInit.h"   /* 仅作频率背景参考；下面常量已写死，不依赖它参与运算 */

/* 充/放电环路共用常量（ISR 内联侧引用）。
 * 充电：电流域增量 PI 出 i_cmd，再经开方逆模型 d = sqrt(KFL·i_cmd/ΔV) 得 d_low。
 * 放电：固定 PI 出归一化 u，再 Dab_Ctrl_DisDutyMap。 */

/* 快断阈值 */
#define DAB_BAT_OVP_VOLT         (60.0f)
#define DAB_DISCHARGE_BUS_OVP_VOLT           (530.0f)

#define DAB_BAT_VOLT_MIN     (30.0f)

/* v105_charge_close_v103_pff充电内环；母线外环及采样接口保持工程原实现。
 * 功率和电池电流限值为整机值，两路各分一半；峰值限值为单路原边值。 */
#define DAB_CHARGE_KFL          (1.75f)       /* 50kHz*35uH，不乘2 */
#define DAB_CHARGE_INV_KFL      (1.0f / DAB_CHARGE_KFL)
/* 漏感取 0.8 倍和 1.2 倍标称值。
 * 0.8 倍漏感同占空比下电流最大：只用于峰值占空比约束（上沿封限，防漏感偏小超峰）。
 * 外环物理能力按标称漏感估算——能力是限流依据，取每台机器可达的典型值；
 * 1.2 倍只参与电流修正量上沿。 */
#define DAB_CHARGE_KFL_MAX      (DAB_CHARGE_KFL * 1.20f)
#define DAB_CHARGE_KFL_MIN      (DAB_CHARGE_KFL * 0.80f)
#define DAB_CHARGE_INV_KFL_MIN  (1.0f / DAB_CHARGE_KFL_MIN)
/* 电流域 PI：开方逆模型线性化后对象增益为 1，系数固定。 */
#define DAB_CHARGE_ALPHA_I      (0.31415927f) /* 2*pi*2500Hz*20us */
#define DAB_CHARGE_WZ_I_TS      (0.08f)       /* 4000rad/s*20us */
#define DAB_CHARGE_K1           (DAB_CHARGE_ALPHA_I * (1.0f + DAB_CHARGE_WZ_I_TS))
#define DAB_CHARGE_K2           (DAB_CHARGE_ALPHA_I)
/* 电流修正量范围按 0.8~1.2 倍漏感折算：偏大需多给 0.2·Iref，偏小需少给 0.2·Iref；
 * 绝对裕量覆盖小电流段损耗与零点偏差。 */
#define DAB_CHARGE_CORR_POS_K   (DAB_CHARGE_KFL_MAX / DAB_CHARGE_KFL - 1.0f)
#define DAB_CHARGE_CORR_NEG_K   (1.0f - DAB_CHARGE_KFL_MIN / DAB_CHARGE_KFL)
#define DAB_CHARGE_CORR_ABS     (0.5f)        /* A */
/* Buck 发波判据：能力域回差（ΔV 与占空比上限共同折算的两路合计可达电流）。
 * 不用固定电压回差：ΔV 是两个 ADC 通道相减、再经匝比放大，量化误差最坏 8~10V，
 * 而 6V/2V 的回差只有 4V，落在误差带内形同虚设。能力域把误差按同样比例带进来，
 * 门限随 压差/ZCS 限幅/峰值限幅/漏感 自动收放，且与下发限流同源。 */
#define DAB_CHARGE_CAP_OFF_MILLI (700U)       /* 整数源(×1000)，供 C 层静态断言用 */
#define DAB_CHARGE_CAP_ON_MILLI  (1500U)
#define DAB_CHARGE_CAP_ON_A     ((float)DAB_CHARGE_CAP_ON_MILLI / 1000.0f)  /* A：整机限流 ≥ 此值允许开通 */
#define DAB_CHARGE_CAP_OFF_A    ((float)DAB_CHARGE_CAP_OFF_MILLI / 1000.0f) /* A：整机限流 < 此值退出发波 */
#define DAB_CHARGE_CAP_RISE_A       (0.05f)  /* 能力缓放：上升每拍最多放开 0.05A（50kHz 下 2.5A/ms） */
#define DAB_CHARGE_REF_HOLD_A       (1.0f)   /* 能力下降时，PI 旧请求可超出有效限流的最大幅值差 (A) */
/* 旧占空比修正带，充电电流环已不再读取；CorrMax≈0.5·D_LOW_MAX（现 ±0.20）。
 * 过窄时满功率点 PI 顶满，稳态电流可偏很大。 */
#define DAB_CHARGE_CORR_MAX     (0.50f * DAB_CHG_D_LOW_MAX)
#define DAB_CHARGE_ZCS_MAX      (0.44f)
#define DAB_CHARGE_IPK_MAX      (28.0f)
#define DAB_CHARGE_POWER_MAX    (3200.0f)
#define DAB_CHARGE_BAT_CUR_MAX  (60.0f)
#define DAB_CHARGE_I_KEEP_MIN   (0.02f)

/* DAB_Bus_Expert_v111_xs根层C-Script及本次粘贴代码的电压环参数。
 * Ki为每个10kHz外环拍的系数，不再重复乘采样周期。 */
#define DAB_BUS_GAIN_E_LO (3.0f)
#define DAB_BUS_GAIN_E_HI (9.0f)
/* 增益调度区间倒数, 编译期常量, 避免每拍做除法 */
#define DAB_BUS_GAIN_INV_SPAN   (1.0f / (DAB_BUS_GAIN_E_HI - DAB_BUS_GAIN_E_LO))
#define DAB_BUS_KP_SPAN         (DAB_BUS_KP_HI - DAB_BUS_KP_LO)                  /* Kp 跨度 */
#define DAB_BUS_KI_SPAN         (DAB_BUS_KI_HI - DAB_BUS_KI_LO)                  /* Ki 跨度(为负) */

#define BUS_CHG_OUT_MAX         ( 1.0f)     /* 充电: PI 校正量上限(A) */
#define BUS_DISCH_OUT_MIN       (-1.0f)     /* 放电: PI 校正量下限(A) */

/* 电压大动态：Ki 助推 + 预测停车 + Out 斜率限制（无电压环电流前馈）
 * 不用 −Kd·dV/dt：陷波后微分仍放大开关纹波，易搅打嗝/给定。制动靠
 * e_pred 提前退回 Ki_NORM，再用每拍 Out 限步压尖峰。
 * Ki_FAST 由物理量定标：Kp_HI·Ts/T_i_fast，Ts 或 Kp 改动自动跟随；
 * |e| 在 6~10V 线性过渡，避免 10V 处增益台阶。 */
#define DAB_BUS_LS_ENABLE       (1U)        /* 大信号 Ki 助推 */
#define DAB_BUS_TS_OUTER        (100.0e-6f) /* 外环周期 10kHz：充 50k/5、放 60k/6 */
#define DAB_BUS_FS              (1.0f / DAB_BUS_TS_OUTER) /* 外环频率 Hz */
#define DAB_BUS_LS_ON_V         (10.0f)     /* |e| 大信号满助推（低于 ±20V 换向带） */
#define DAB_BUS_LS_OFF_V        (6.0f)      /* |e| 开始助推 */
#define DAB_BUS_LS_INV_SPAN     (1.0f / (DAB_BUS_LS_ON_V - DAB_BUS_LS_OFF_V))
#define DAB_BUS_TI_FAST         (4.0e-3f)   /* 大信号积分时间 s */
#define DAB_BUS_KI_FAST         (DAB_BUS_KP_HI * DAB_BUS_TS_OUTER / DAB_BUS_TI_FAST)
#define DAB_BUS_T_LEAD          (0.001f)    /* 预测提前量 s（≈误差滤波+内环迟滞） */
#define DAB_BUS_OUT_SLEW_A      (0.5f)      /* 外环单拍 |ΔOut| 上限 @10kHz ≈5A/ms */

#define DAB_BUS_KP_LO     (0.02f)
#define DAB_BUS_KP_HI     (0.2f)

#define DAB_BUS_KI_LO     (0.0006f)
#define DAB_BUS_KI_HI     (0.0002f)
/* 轻载打嗝迟滞幅值（充放电统一，判决方向随工况镜像，见 Interrupt_CtrlDab.c）。
 * A = 最大外环 Kp × 封波期允许电压漂移，定值方法：ΔV允许 ≈ A / Kp；
 * 取 Kp=KP_HI(0.20A/V)、ΔV允许=1.0V → A=0.20A。公式只给工程初值，
 * 增量 PI、积分项也影响实际漂移，改 KP_HI/ΔV 后须回归
 * 打嗝频率与封波期电压波动实测。 */
#define DAB_BUS_VDRIFT_ALLOW_V      (1.0f)
#define DAB_BURST_HYST_A            (DAB_BUS_KP_HI * DAB_BUS_VDRIFT_ALLOW_V)

#define DAB_DISCHARGE_HV_SLOPE   (0.45f)            /* 放电高压桥占空比后段斜率 */

/* 放电电流环参数沿用当前工程值，实机动态性能尚待验证。 */
#define DAB_DISCHARGE_KP           (0.002f)          /* 放电内环 Kp */
#define DAB_DISCHARGE_KI_TS        (0.00333333f)     /* Ki*Ts = 200/60000 = 0.0033333，放电内环 60kHz */

#define DAB_DUTY_MAX_STEPS         (5.0f)            /* PI增量限步系数：剩余调制行程/5 */
#define DAB_DUTY_MAX_STEPS_INV     (1.0f / DAB_DUTY_MAX_STEPS)  /* =0.2，热路径用乘逆代替每拍除法 */
#define DAB_DUTY_STEP_MIN          (0.1f)            /* PI允许单拍步长的最小上限 */
#define DAB_DUTY_STEP_DOWN         (0.2f)            /* PI允许单拍下降步长的固定上限, 限制电流冲击 */

/* 误差低通每载波拍执行，Alpha 固定不随工况切换。
 * 外环由ISR共享相位充5拍/放6拍执行一次(10kHz)。
 * Alpha=Ts/tau：50kHz 载波约 200Hz，60kHz 载波约 240Hz。
 * 电压斜坡由1ms任务推进，ISR不再维护VoltStep。 */
#define DAB_BUS_CLK_HZ       (150.0e6f)      /* ePWM 时钟 150MHz = F28P550 主频，见 ConfigChip_EpwmInit.h */
#define DAB_BUS_TS_CHG       (20.0e-6f)      /* up-down: Ts=2*TBPRD/CLK=2*1500/150e6=20us (50kHz) */
#define DAB_BUS_TS_DIS       (16.6667e-6f)   /* up-down: Ts=2*TBPRD/CLK=2*1250/150e6=16.6667us (60kHz) */
    
#define DAB_BUS_TAU          (0.7958e-3f)    /* 误差低通 tau=1/(2*pi*200Hz)=0.7958ms */
#define DAB_BUS_ALPHA        (0.025133f)     /* 充放电共用: alpha=Ts/tau=20us/0.7958ms */

/* SR 使能判据（单路）：只看实测瞬时功率；关断单拍即撤，开通需连续 N 拍确认。
 * 失效代价不对称，所以两侧时序不同：
 *   关断必须快——滤波功率滞后几拍就是倒流，且倒流会让外环给定为负、实测为正，
 *   控制器直接失去控制权；所以任一拍跌破下限立即撤，不加任何确认。
 *   开通可以慢——晚开只是多走几拍体二极管，不影响安全；而单点采样带纹波相位偏差
 *   和开关噪声，直接拿它开通会让 SR 反复开关，所以要求连续 N 拍都过开通门限。 */
#define DAB_SR_ON_INST_W        (500.0f)      /* W：瞬时功率 ≥ 此值才计入开通确认 */
#define DAB_SR_OFF_INST_W       (425.0f)      /* W：瞬时功率 ≤ 此值立即退出 */
#define DAB_SR_ON_CNT           (8U)          /* 拍：连续N拍过开通门限才使能(50kHz≈160us) */
/* 极短软起：使能后的前 N 拍，SR 窗口按 1/N 线性放宽到目标值。
 * 目的不是平滑功率（功率由 d 决定，与 SR 无关），而是万一判据漏判，
 * 第一拍的窗口只有目标的 1/20，把误开的冲击摊到几拍里。
 * 不做长软起：DAB 半开区间电流走体二极管，反向恢复会削弱 ZCS，且轻载打嗝
 * 会反复归零，长软起反而长期停在半开、效率更差。 */
#define DAB_SR_SOFT_CNT         (20U)          /* 拍：窗口从 1/N 爬到 1 (50kHz≈400us) */
#define DAB_SR_SOFT_INV         (1.0f / (float)DAB_SR_SOFT_CNT)  /* 软起系数倒数，编译期常量 */
#define DAB_SR_DUTY_MARGIN      (0.01f)
#define DAB_SR_DUTY_MAX         (0.44f)

#endif
