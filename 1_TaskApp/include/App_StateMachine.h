/**
 * ============================================================================
 * 文件名称: App_StateMachine.h
 * 模块功能: DAB (双有源全桥变换器) 主状态机头文件
 * 控制核心: 充放电全工况统一闭环控制高压直流母线电压 (Vbus) 与母线电流 (Ibus)
 * 任务周期：主状态处理 2 ms，待机与换向 100 µs，软起超时计时 5 ms
 * 版权说明: 最终量产定版 (Production Ready)
 * ============================================================================
 *
 * 状态流转：
 *   INIT -> RELAY -> GPIO -> PRECHECK -> RAMP_CLOSE -> RUN（起机跳过 STANDBY）
 *   协议 standby 不参与状态；只认 RunState.state
 *   换向：100µs 仲裁+提交；强制/CAN 驻留 5×100µs，自动确认≈400µs + 1×100µs 后出 RUN
 *   Iset≈0：不进 STANDBY，由 curLowSealing 方向封波
 *   故障或关机：任意阶段 -> INIT。STANDBY 占用原 RAMP_OPEN=4。
 * ============================================================================
 */

#ifndef APP_STATE_MACHINE_H
#define APP_STATE_MACHINE_H

#include "Includes.h"

/* ==========================================================================
 * 1. 运行模式与控制指令枚举
 * ========================================================================== */

/**
 * @brief 工作模式枚举
 */
typedef enum
{
    WORK_MODE_STANDBY   = 0U,                       /**< 待机模式：封波停机，输出占空比为零 */
    WORK_MODE_CHARGE    = 1U,                       /**< 充电模式：高压母线输入 -> 电池储能 */
    WORK_MODE_DISCHARGE = 2U                        /**< 放电模式：电池储能输出 -> 维持高压母线 */
} WorkMode_e;

/**
 * @brief 主状态机状态枚举（7 个正常状态，1 个预留值）
 */
typedef enum
{
    SM_STATE_INIT         = 0,                      /**< [状态0] 初始化 / 硬件故障保护停机锁死 */
    SM_STATE_PRECHECK     = 1,                      /**< [状态1] 预检查：建立初态采样，决断充放电 */
    SM_STATE_RELAY        = 2,                      /**< [状态2] 继电器吸合与高低压侧压差平稳判定 */
    SM_STATE_GPIO         = 3,                      /**< [状态3] 继电器动作后硬件电平建立缓冲 */
    SM_STATE_STANDBY      = 4,                      /**< [状态4] 待机/换向中转：100us 封波并清控制量（起机不经此态） */
    SM_STATE_RAMP_CLOSE   = 5,                      /**< [状态5] 闭环给定斜坡软起 (母线双闭环共用) */
    SM_STATE_RUN          = 6,                      /**< [状态6] 稳态发波闭环运行 (支持在线调参平滑) */
    SM_STATE_MODE_SWITCH  = 7                       /**< [状态7] 预留（原 MODE_SWITCH）；误入回 INIT */
} SmState_e;

/**
 * @brief CAN 通信外部调度模式指令枚举
 */
typedef enum
{
    CAN_CMD_NONE      = 0U,                         /**< 无外部动作请求 */
    CAN_CMD_CHARGE    = 1U,                         /**< 远程指令：请求切入充电模式 */
    CAN_CMD_DISCHARGE = 2U                          /**< 远程指令：请求切入放电模式 */
} CanCmd_e;

/* ==========================================================================
 * 2. 2ms 独立时基拍数参数宏定义 (公式: 拍数 = 目标时间(ms) / 2ms)
 * ========================================================================== */
#define SM_T_RELAY_STABLE             (250U)        /**< 继电器压差平稳持续确认时延：500ms (250拍) */
#define SM_T_RELAY_TIMEOUT            (750U)        /**< 继电器闭合最大超时故障门限：1500ms (750拍) */
#define SM_T_GPIO_DELAY               (25U)         /**< GPIO 动作吸收缓冲时延：50ms (25拍) */
#define SM_T_RAMP_CLOSE_MIN           (5U)          /**< RAMP_CLOSE 最短停留：10ms (5拍)，防零拍直穿 RUN */
#define SM_T_STANDBY_FORCE            (5U)          /**< 强制/CAN：STANDBY 驻留 5×100µs */
#define SM_T_STANDBY_AUTO             (1U)          /**< 环路自动：STANDBY 驻留 1×100µs */

/* ==========================================================================
 * 3. 电压/电流工程判定阈值与 100µs 换向防抖
 * ========================================================================== */
#define SM_V_BUS_DIFF_MAX             (30.0f)       /**< 继电器允许最大母线压差 (30V) */
#define SM_V_MODE_BAND_LO             (10.0f)       /**< 放→充 电压反时限起算带：高于设定 +10V（对应 5ms） */
#define SM_V_MODE_BAND                (20.0f)       /**< 充→放 电压速切带（低于设定 -20V 直接 300µs 切放电）/ 放→充 1ms 参考带 */

#define SM_N_PRECHECK_WIN             (20U)         /**< 预检滑动窗 40ms */
#define SM_N_PRECHECK_TH              (15U)         /**< 窗内「PCS < 设定值」命中门限 → 放电（无死区带，靠多数票抑制带内抖动） */
#define SM_N_IREF_SWITCH              (4U)          /**< RUN：电流给定换向确认 ≈400µs（两方向同一时长，需分开再拆两个宏） */
#define SM_N_VOLT_TO_DISCH            (3U)          /**< RUN：充→放 电压低于设定 20V 直接 3 拍（300µs）切放电 */

/* 放→充 在线电压反时限积分计票（100µs 节拍，满计 SM_VOLT_ACC_FULL = 500）：
 *   ΔV <= 10V  → 清零不计票
 *   ΔV  = 10V  → 每拍 +10  → 50 拍 = 5.0ms
 *   ΔV  = 20V  → 每拍 +50  → 10 拍 = 1.0ms
 *   ΔV >= 32.5V→ 每拍 +100 →  5 拍 = 0.5ms（最快下限，防单拍毛刺） */
#define SM_VOLT_ACC_FULL              (500U)        /**< 放→充 电压反时限满计门限 */
#define SM_VOLT_ACC_STEP_MIN          (10.0f)       /**< ΔV=10V 时每 100µs 增量（500/10 = 50拍 = 5ms） */
#define SM_VOLT_ACC_STEP_MAX          (100.0f)      /**< 最大每 100µs 增量（500/100 = 5拍 = 0.5ms） */
#define SM_VOLT_ACC_SLOPE             (4.0f)        /**< 增量斜率：(50-10)/(20-10) = 4/V，20V 时恰好 1ms */


/* ==========================================================================
 * 4. 浮点收敛死区与禁止方向电流设定门限
 * ========================================================================== */
#define SM_EPS_CURR                   (0.5f)        /**< 母线电流环收敛判决死区容差 (0.5A) */

/* 禁止方向门限：设定 <= 0.5A 视为该方向禁用（只封该方向，另一方向照常发波） */
#define SM_IBUS_DISCHAR_STANDBY_MIN   (0.5f)        /**< 放电母线电流设定 <= 0.5A 禁止放电 */
#define SM_IBUS_CHAR_STANDBY_MIN      (0.5f)        /**< 充电母线电流设定 <= 0.5A 禁止充电 */

/* 禁止方向位图（RunState.curLowSealing）：1ms 由 Ctrl_RampCfg 重算，
 * ISR 只在"当前工况正好等于被禁方向"时封波，另一方向照常工作。 */
#define SM_SEAL_NO_CHARGE             (1U)          /**< bit0: 禁止充电（充电设定 <= 0.5A） */
#define SM_SEAL_NO_DISCHARGE          (2U)          /**< bit1: 禁止放电（放电设定 <= 0.5A） */

/* ==========================================================================
 * 5. 系统控制命令字结构体 (严格匹配 16-bit 协议寄存器)
 * ========================================================================== */
typedef struct
{
    Uint16 shutdown  : 1;                           /**< Bit 0: 关机/停机控制 (0:关机请求, 1:正常运行) */
    Uint16 standby   : 1;                           /**< Bit 1: 保留（待机以 SM_STATE_STANDBY 为准，不镜像协议） */
    Uint16 rsvdMode  : 2;                           /**< Bit 2..3: 保留（原 workMode 已移至 RunState.workMode，避免 ISR/任务对位域读改写竞争） */
    Uint16 fault     : 1;                           /**< Bit 4: 硬件综合故障状态 */
    Uint16 update    : 1;                           /**< Bit 5: IAP 在线固件升级激活 */
    Uint16 openTest  : 1;                           /**< Bit 6: 开环发波工程测试模式使能 */
    Uint16 forceMode : 2;                           /**< Bit 7..8: 强制模式控制 (0:自适应, 1:强放, 2:强充) */
    Uint16 rsvd      : 7;                           /**< Bit 9..15: 协议保留位填充 */
} SysCmdBits_t;

typedef union
{
    Uint16        all;                              /**< 16位全字访问，用于通讯报文收发解析 */
    SysCmdBits_t  bit;                              /**< 单比特位段访问 */
} SysCmd_t;

/* ==========================================================================
 * 6. 运行状态位图结构体 (与 SmState_e 位序 1:1 绝对映射，上位机监视专用)
 * ========================================================================== */
typedef struct
{
    Uint16 init       : 1;                          /**< Bit 0: INIT */
    Uint16 precheck   : 1;                          /**< Bit 1: PRECHECK */
    Uint16 relay      : 1;                          /**< Bit 2: RELAY */
    Uint16 gpio       : 1;                          /**< Bit 3: GPIO */
    Uint16 standby    : 1;                          /**< Bit 4: STANDBY */
    Uint16 rampClose  : 1;                          /**< Bit 5: RAMP_CLOSE */
    Uint16 run        : 1;                          /**< Bit 6: RUN */
    Uint16 modeSwitch : 1;                          /**< Bit 7: 预留 MODE_SWITCH */
    Uint16 rsvd       : 8;                          /**< Bit 8..15 */
} RunStaBits_t;

typedef union
{
    Uint16        all;                              /**< 16 位全字，与位域同宽 */
    RunStaBits_t  bit;                              /**< 位域访问 */
} RunSta_t;

/* ==========================================================================
 * 7. 状态机全局运行上下文对象
 * ========================================================================== */
typedef struct
{
    SmState_e  state;                               /**< 主状态机当前阶段 */
    RunSta_t   staFlag;                             /**< 上位机状态观测位图映射 */
    Uint16     workMode;                            /**< 当前实际工作模式 (WorkMode_e)：ISR/任务共享，单 16 位原子 */

    Uint16     stateTick;                           /**< 通用阶段计时：RELAY/GPIO/PRECHECK/RAMP_CLOSE 最短驻留；进态清零 */
    Uint16     softStartTick;                       /**< 软起本态计时(5ms，SM_SoftStartTick5ms)；EnterState 清零；满→Fault_SetSoftTo */
    Uint16     softStartArmed;                      /**< 1=冷启监视中（首次 RUN 当场清；在线换向不再套 50s） */
    Uint16     relayStable;                         /**< RELAY 压差连续平稳计数（与超时并行，不能并进 stateTick） */
    Uint16     precheckLowCnt;                      /**< 预检窗内低压命中次数（非计时） */

    Uint16     irefDebounce;                        /**< RUN：反向给定且本向电流已退出（充 iAct>-0.5A / 放 iAct<0.5A）；充→放 / 放→充 均 400µs */
    Uint16     voltDebounce;                        /**< 仅 RUN 电压计票（充→放：<-20V 300µs；放→充：>+10V 反时限 5ms~0.5ms） */
    Uint16     standbyDwellN;                       /**< 本趟 STANDBY 目标驻留拍数（100µs） */

    Uint32     openTestTime;                        /**< 调试与发波累计运行时长计数器 */

    CanCmd_e   canCmd;                              /**< CAN 远程总线调度模式指令 */
    Uint16     isMaster;                            /**< 主/从机工作角色标志 (1:主机, 0:从机) */
    Uint16     curLowSealing;                       /**< 禁止方向位图 (bit0:禁充电, bit1:禁放电, 0:双向可用) */

    float      f32ColdDis;                          /**< PRECHECK 判向当拍有效放电限值(A)：冷启装载直接复用，不重算 */
    float      f32ColdChg;                          /**< PRECHECK 判向当拍有效充电限值(A)：冷启装载直接复用，不重算 */
} RunState_t;

/* ==========================================================================
 * 8. 全局变量声明与对外调用 API 原型
 * ========================================================================== */
extern volatile SysCmd_t    SysCmd;                  /**< 全局系统命令控制字实体（任务写、ISR 读，必须 volatile；workMode 已移至 RunState.workMode） */
extern volatile RunState_t  RunState;                        /**< 全局状态机运行上下文实体（含 workMode：ISR/任务共享，单 16 位原子） */

/**
 * @brief  主状态机软硬件变量冷态初始化函数
 * @param  无
 * @return 无
 */
void SM_Init(void);

/**
 * @brief  2ms 周期状态机执行核心步进函数
 * @param  无
 * @return 无
 */
void SM_Step_2ms(void);

/**
 * @brief  从 CfgParam.SysCmd 同步运行命令字（串口 0x00 / CAN 0x20 / 2ms 共用）
 */
void SM_SyncSysCmd(void);

/**
 * @brief  5ms：软起超时计时；armed 且本态满 FAULT_N_SOFT_TO → Fault_SetSoftTo
 */
void SM_SoftStartTick5ms(void);

/**
 * @brief  100us：工况仲裁 + 换向进 STANDBY（强制 5 拍 / 自动 1 拍）后装载回 RAMP_CLOSE
 */
void SM_Standby_100us(void);

#endif /* APP_STATE_MACHINE_H */
