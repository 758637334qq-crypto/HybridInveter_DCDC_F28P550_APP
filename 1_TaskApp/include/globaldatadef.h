#ifndef GLOBALDATADEF_H_
#define GLOBALDATADEF_H_

#include <stdint.h>

/* ============================================================================
 * 全局数据结构定义（DAB 双向电源）
 *
 * 本文件只放“跨模块共享”的数据：结构体、位域、全局对象声明与初始化入口。
 * 约定：
 *   1. 结构体类型名全大写（DATASCOPE / MDADDRMES / SCANMES / CFGParam…），
 *      全局对象名用驼峰（DataScope / uMdlAddMes / sCanMes…）。
 *   2. 通信量按协议单位命名：_dV=0.1V、_cA=0.01A、_dC=0.1℃、_W=W、_dv/_dC=0.1 单位。
 *   3. CfgParam / sCanMes 由通信任务写、ISR 或 2ms 任务读，必须 volatile，
 *      防止 -O2 把读缓存成旧值（见各自声明处的说明）。
 * ============================================================================ */

/* ============================================================================
 * 平台基础数据类型重定义 (兼容 TI C2000 DSP)
 * 注意: #ifndef Uint16 对 typedef 无效；若本头先于 f28x 被包含，勿再让
 * f28003x_device.h 重复 typedef —— 编译单元应保证 f28x 优先，或仅用本兜底。
 * ============================================================================ */
#if !defined(F28_DATA_TYPES) && !defined(DSP28_DATA_TYPES)
typedef uint16_t Uint16;
typedef uint32_t Uint32;
typedef int16_t  int16;
#endif

/* TI device 头只有 int16；用宏别名避免 typedef 与后续头冲突 */
#ifndef Int16
#if defined(F28_DATA_TYPES) || defined(DSP28_DATA_TYPES)
#define Int16 int16
#else
typedef int16_t Int16;
#endif
#endif

/* 并联模块最大数量：数组统一开 recnum+1（下标 0 不用，1~recnum 对应模块号） */
#define recnum                  (8)

/* ============================================================================
 * 0. 软件示波器 (DataScope)
 *
 * 经典 scopeGet 结构：未触发时在 [0, PRE_TRIGGER_NUM-1] 维持 64 点预触发环；
 * 触发后从 PRE_TRIGGER_NUM 起线性存满 900 点并锁定，由 CAN 邮箱 2 分包上传，
 * 传完清 uTran_CMD 重新布防。4 路通道放大 10 倍存 int16。
 * ============================================================================ */
#define SCOPENUM            900     /* 录波总点数 */
#define PRE_TRIGGER_NUM     64      /* 预触发点数（须为 2 的幂，环形掩码用） */
#define PRE_TRIGGER_MASK    (PRE_TRIGGER_NUM - 1U)

typedef struct
{
    int16  uDataScope0[SCOPENUM];   /* 通道 0：Ramp.iPos  ×10 */
    int16  uDataScope1[SCOPENUM];   /* 通道 1：Ramp.iNeg  ×10 */
    int16  uDataScope2[SCOPENUM];   /* 通道 2：Bus.f32Ref ×10 */
    int16  uDataScope3[SCOPENUM];   /* 通道 3：Ramp.v     ×10 */
    Uint16 uCurrent_Save_subscript; /* 录波写入下标：触发后线性递增，到 SCOPENUM 锁定 */
    Uint16 uCurrent_Tran_subscript; /* 上传点序 0~SCOPENUM-1：CAN 任务按 TXBTO 推进；点序→槽位映射见 CanComm_SendScope */
    Uint16 uTran_CMD;               /* 0=预触发环形记录；1=已触发/锁定，存满后待上传 */
    Uint16 uPreTriggerIdx;          /* 预触发环写指针 0~63；触发后冻结（槽 r=最旧样本），上传按 (r+点序)&63 映射取数，上传完成清 0 */
} DATASCOPE;

extern volatile DATASCOPE DataScope;   /* ISR 写、1ms CAN 任务读写：必须 volatile，防 -O2 缓存 */

/* ============================================================================
 * 1. 模块地址 / CAN 运行态 (MDADDRMES)
 *
 * 多机并联时，主机通过 CAN 给各从机分配逻辑地址（0~255），本结构保存
 * 本机地址、已登记模块的地址表，以及均流所需的各模块电流/电压样本。
 * ============================================================================ */
typedef struct
{
    Uint16      bCalIDCmd:1;    /* 1=收到地址仲裁命令，需要应答 */
    Uint16      bSendIDCMD:1;   /* 1=需要主动发送本机 ID 请求 */
    Uint16      bStopAddr:1;    /* 1=停止地址分配流程 */
    Uint16      bRecAddr:1;     /* 1=已收到地址分配命令，待重构邮箱 */
    Uint16      res:12;         /* 保留位 */
} CANADDSTATE_BITS;

typedef union
{
    Uint16            all;      /* 整字访问（通信/调试） */
    CANADDSTATE_BITS  bits;     /* 按位访问 */
} CANADDSTATE;

typedef struct
{
    CANADDSTATE uCanState;                  /* 地址分配状态位（见 CANADDSTATE_BITS） */
    Uint16 uReqIDnum;                       /* 已请求分配地址的节点数 */
    Uint16 uTranEND_num;                     /* 地址分配流程结束标志计数 */
    Uint16 uIndex;                          /* 本机逻辑地址（0~255，分配后有效） */
    Uint16 MDADDRID_num;                    /* 已登记模块数量 */
    Uint32 uSelfID;                         /* 本机唯一 ID（硬件序列号/芯片特征 ID） */
    Uint16 MDADDRID[recnum+1];              /* 各模块逻辑地址表（下标即模块序号） */
    float uRecMdl_Cur[recnum+1];            /* 各模块实测电流 [A]（均流用） */
    Uint16 uRecMdl_onoff[recnum+1];         /* 各模块在线/开关机状态 */
    float uRecMdl_Curset[recnum+1];         /* 各模块电流设定 [A] */
    float uRecMdl_CursetCoff[recnum+1];     /* 放电均流系数（按容量分配） */
    float uRecMdl_ReCursetCoff[recnum+1];   /* 充电均流系数（按容量分配） */
    float uRecMdl_vbat[recnum+1];           /* 各模块电池电压 [V] */
    float uRecMdl_Ibatset[recnum+1];        /* 各模块电池电流设定 [A] */
    Uint16 ADDRID_Config[recnum+1];         /* 地址配置表（掉电保存/上位机下发） */
    Uint16 uRecMdl_RunState[recnum+1];      /* 各模块运行状态（与 SmState_e 对应） */
    float fAverCur;                         /* 并联平均电流 [A] */
    float fAverVbat;                        /* 并联平均电池电压 [V] */
    Uint16 uAverStyle;                      /* 均流方式：0=主从，1=平均 */
} MDADDRMES;

extern MDADDRMES uMdlAddMes;

/* ============================================================================
 * 2. CAN 调试/统计 (SCANMES)
 *
 * CAN 中断与通信任务写、2ms 任务与安全任务读；故障锁存位一旦置位只能由
 * “停机 + 故障源消失”清除（见 UpdateShutdownFlag）。
 * ============================================================================ */
typedef struct
{
    Uint16 uCalADDR_Cyle;           /* 地址重分配超时计数（15s 封顶） */
    Uint16 uCanRec_count;           /* 收到有效报文的累计数 */
    Uint16 uCanWatch_Alarm_count;   /* 上位机巡检/看门狗告警计数 */
    Uint16 uCan_Test;               /* 通信自检计数 */
    float  utest_temp;              /* 调试临时量 */
} SCANMES;

extern volatile SCANMES sCanMes;   /* CAN 中断/通信任务写、2ms 任务与 ISR 读：必须 volatile，防 -O2 缓存 */

/* ============================================================================
 * 3. 系统控制指令位域
 *
 * 与 SysCmd（运行态工作模式字）不同，这里是“可下发的控制命令”，由上位机
 * 通过 0x20 参数写入或通信任务更新。
 * ============================================================================ */
typedef struct
{
    Uint16 onoffcmd:1;      /* 开关机命令：1=开机，0=关机 */
    Uint16 standby:1;       /* 待机命令 */
    Uint16 CharDcdcOn:1;    /* 充电 DCDC 使能 */
    Uint16 ReCharDcdcOn:1;  /* 放电 DCDC 使能 */
    Uint16 faultcmd:1;      /* 远程故障注入：1=置故障 */
    Uint16 clearfaultcmd:1; /* 清除故障命令 */
    Uint16 openTest:1;      /* 开环发波工程测试模式 */
    Uint16 ReCharForce:2;   /* 强制充放：0=自适应，1=强放，2=强充 */
    Uint16 rsvd:7;          /* 保留位 */
} SYS_CMD_BITS;

typedef union
{
    Uint16        all;      /* 整字访问 */
    SYS_CMD_BITS  bit;      /* 按位访问 */
} UnionSysCmd;

/* ============================================================================
 * 4. 参数配置表
 *
 * 上位机 0x20 命令按“字偏移”逐字改写本结构，因此字段顺序即协议地址，
 * 增删字段会破坏上位机兼容性。数值单位见各行注释。
 * ============================================================================ */
typedef struct
{
    UnionSysCmd SysCmd;             /* 系统控制命令位（见 SYS_CMD_BITS） */
    Uint16 BatChgCurSet;            /* 电池充电电流    [0.1A] */
    Uint16 BatDischgCurSet;         /* 电池放电电流    [0.1A] */
    Uint16 BusChgCurSet;            /* 母线充电电流    [0.1A] */
    Uint16 BusDischgCurSet;         /* 母线放电电流    [0.1A] */
    Uint16 BusVolSet;               /* 母线电压给定    [0.1V] */
    Uint16 BatVolSet;               /* 电池电压给定    [0.1V] */
    Uint16 OpenLoopDuty;            /* 开环测试占空比  [0.1%] */
    Uint16 ScopeTrigCmd;            /* 软件示波器触发：0=停止/复位录波，1=立即触发（索引 8） */
    Uint16 res;                     /* 保留（对齐/协议占位，索引 9） */
} CFGParam;

extern volatile CFGParam CfgParam; /* 通信任务写、ZERO ISR 读给定/增益：必须 volatile，防 -O2 缓存 */

/* ============================================================================
 * 运行标志位定义 (CAN 0x19 低半字)
 *
 * bit0~bit7  : 状态机状态码 (SmState_e：0=INIT ... )
 * bit8~bit9  : 工作模式码 (WorkMode_e：0=待机, 1=充电, 2=放电)
 * bit10      : 禁充 (SM_SEAL_NO_CHARGE)
 * bit11      : 禁放 (SM_SEAL_NO_DISCHARGE)
 * bit12      : 待机 (RunState.state == STANDBY)
 * bit13~15   : 保留
 * ============================================================================ */
typedef struct
{
    Uint16 workState  : 8;      /* bit0~bit7  : 当前状态码 */
    Uint16 workMode   : 2;      /* bit8~bit9  : 当前工作模式码 */
    Uint16 noCharge   : 1;      /* bit10      : 禁充 */
    Uint16 noDischg   : 1;      /* bit11      : 禁放 */
    Uint16 standby    : 1;      /* bit12      : 待机 */
    Uint16 rsvd       : 3;      /* bit13~15   : 保留 */
} RunFlagBits_t;

typedef union
{
    Uint16        all;          /* 整字访问（CAN 上报） */
    RunFlagBits_t bit;          /* 按位访问 */
} RunFlag_t;

/* ============================================================================
 * 双向电源遥测上报数据结构体 (严格 1:1 对齐 CAN 0x11 轮询协议)
 *
 * 0x11 命令按通道号把下面的量两两打包成 4 字节，大端发送。
 * 字段顺序 = 通道表顺序，改动字段会直接改变上位机解析结果，勿随意调整。
 * ============================================================================ */
typedef struct
{
    struct {
        Uint16  vbat_dV;          /* 电池电压                [0.1V]  */
        Uint16  vbusPcs_dV;       /* PCS 端电压(VPBUS_ADC)   [0.1V]  */
        Uint16  vbus1_dV;         /* 高压母线 1              [0.1V]  */
        Uint16  vbus2_dV;         /* 高压母线 2              [0.1V]  */
        Uint16  vmBus_dV;         /* 继电器 VM(VMBUS_ADC)    [0.1V]  */
        Int16   sampleIr_cA;      /* 采样电流            [0.01A] */
        Int16   iout1_cA;         /* 输出电流 1          [0.01A] */
        Int16   iout2_cA;         /* 输出电流 2          [0.01A] */
        Int16   pout1_W;          /* 输出功率 1          [W]     */
        Int16   pout2_W;          /* 输出功率 2          [W]     */
        Int16   tempInductor_dC;  /* 电感温度            [0.1℃] */
        Int16   tempAmbient_dC;   /* 环境温度            [0.1℃] */
        Int16   tempHeatsink_dC;  /* 散热器温度          [0.1℃] */
        Int16   tempMos_dC;       /* MOS 温度            [0.1℃] */
    } ADC;

    struct {
        Int16   vLoop1_Err_cV;    /* 电压环 1 误差      [0.01V] */
        Int16   vLoop1_Out;       /* 电压环 1 输出      [0.01V] */
        Int16   vLoop2_Err_cV;    /* 电压环 2 误差      [0.01V] */
        Int16   vLoop2_Out;       /* 电压环 2 输出      [0.01V] */
        Uint16  vLoop1_DutyFf;    /* 电压环 1 前馈占空比 */
        Uint16  vLoop2_DutyFf;    /* 电压环 2 前馈占空比 */
        Int16   iLoop1_Err_cA;    /* 电流环 1 误差      [0.01A] */
        Int16   iLoop1_Out;       /* 电流环 1 输出      [0.01A] */
        Int16   iLoop2_Err_cA;    /* 电流环 2 误差      [0.01A] */
        Int16   iLoop2_Out;       /* 电流环 2 输出      [0.01A] */
        Uint16  iLoop1_DutyFf;    /* 电流环 1 前馈占空比，0.01%（1.0 = 10000） */
        Uint16  iLoop2_DutyFf;    /* 电流环 2 前馈占空比，0.01%（1.0 = 10000） */
        Int16   vLoop_Ref_dV;     /* 电压环参考，两路共用 [0.1V]，CAN 0x2B 低半字 */
        Int16   vLoop_Ff_cA;      /* 电压环前馈 [0.01A]，充电为 0，CAN 0x2B 高半字 */
        Int16   iLoop1_Ff_cA;     /* 电流前馈 1 [0.01A]，CAN 0x2C 低半字 */
        Int16   iLoop2_Ff_cA;     /* 电流前馈 2 [0.01A]，CAN 0x2C 高半字 */
    } LoopCtrl;

    struct {
        Uint16  voutSet_dV;       /* 输出电压给定        [0.1V]  */
        Int16   vbusDroop_dV;     /* 母线总调节电压 voltAdj [0.1V] */
        Uint16  vbus1Ref_dV;      /* 母线 1 控制 ref（Bus.f32Vref）[0.1V] */
        Uint16  vbus1Set_dV;      /* 母线 1 下发设定（BusVolSet） [0.1V] */
        Uint16  vbus1Cmd_dV;      /* 母线 1 步进设定（Ramp.v）    [0.1V] */
        Uint16  resv_0x0F_hi;     /* 通道 0x0F 高半字保留          */
        Uint16  vbus2Ref_dV;      /* 母线 2 控制 ref（同外环）    [0.1V] */
        Uint16  vbus2Set_dV;      /* 母线 2 下发设定（同外环）    [0.1V] */
        Uint16  vbus2Cmd_dV;      /* 母线 2 步进设定（同外环）    [0.1V] */
        Uint16  resv_0x11_hi;     /* 通道 0x11 高半字保留          */
        Int16   iCharSet_cA;      /* 实际的充电电流设定     [0.01A] */
        Int16   ibus1ref_cA;      /* 母线 1 电流参考     [0.01A] */
        Int16   iReCharSet_cA;    /* 实际的放电电流设定     [0.01A] */
        Int16   ibus2ref_cA;      /* 母线 2 电流参考     [0.01A] */
        Int16   droopCur_dv;      /* CAN 0x14 低：下垂调节电压 droop   [0.1V] */
        Int16   canAverCur_dv;    /* CAN 0x14 高：均流调节电压 voltAdj [0.1V] */
    } Setpoint;

    struct {
        Uint16  bmsCharPowSet_W;   /* BMS 充电功率上限    [W] */
        Uint16  bmsrecharPowSet_W; /* BMS 放电功率上限    [W] */
        Uint16  sysMaxChgPow_W;    /* 系统最大充电功率    [W] */
        Uint16  sysMaxRechPow_W;   /* 系统最大放电功率    [W] */
        Uint16  tempLimChar_W;     /* 温度降额充电功率    [W] */
        Uint16  tempLimRech_W;     /* 温度降额放电功率    [W] */
        Uint16  ibatLimChar_cA;    /* 电池限流充电        [0.01A] */
        Uint16  vbatLimRech_W;     /* 电池限压放电功率    [W] */
    } PowerLimit;

    struct {
        RunFlag_t runFlag;        /* 运行标志：低字节状态码，高字节工作模式码 */
        Uint16  dabRunSta;        /* DAB 状态机当前状态（SmState_e） */
        Uint16  canRecCnt;        /* CAN 接收计数（低 16 位） */
        Uint16  resv;             /* 保留 */
    } Status;

    struct {
        Uint32  slowfault1;       /* 实时慢保护字 stFault.slow */
        Uint16  fastfault;        /* 实时快保护字 stFault.fast */
        Uint16  alarm;            /* 实时 alm（CAN/SCI 打包处再映射到原 slowfault2 槽） */
    } RealAlarm;

    struct {
        Uint32  saveslowfault1;   /* 锁存慢保护字 */
        Uint16  savefastfault;    /* 锁存快保护字 */
        Uint16  savealarm;        /* 锁存 alm（打包映射到原 saveslowfault2 槽） */
    } SaveAlarm;

    float   utest1;               /* 调试观测通道 1（0x23） */
    float   utest2;               /* 调试观测通道 2（0x24） */
    float   utest3;               /* 调试观测通道 3（0x25） */
    float   utest4;               /* 调试观测通道 4（0x26） */
    float   utest5;               /* 调试观测通道 5（0x27） */
    float   utest6;               /* 调试观测通道 6（0x28） */
    float   utest7;               /* 调试观测通道 7（0x29） */
    float   utest8;               /* 调试观测通道 8（0x2A） */

} CanMonitorData_t;

extern CanMonitorData_t g_CanMon;
void Update_CanMon(void);         /* 汇总上述遥测量（DataCalc 任务里周期调用） */

void init_global_data(void);      /* 上电初始化：清空所有全局结构并填默认配置 */

#endif /* GLOBALDATADEF_H_ */
