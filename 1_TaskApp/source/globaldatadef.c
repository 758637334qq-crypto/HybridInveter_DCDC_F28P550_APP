/* ============================================================================
 * 文件：globaldatadef.c
 * 功能：全局共享对象的实体定义 + 上电初始化
 *
 * 这些对象被 2ms 状态机、1ms 数据任务、CAN 通信（中断/任务）和 ZERO ISR
 * 同时访问，所以有两条硬规则：
 *   1. sCanMes / CfgParam / DataScope 声明为 volatile（读侧每次必须落内存，见头文件）；
 *   2. 这里的 memset 只在上电、开中断之前执行一次，因此对 volatile 对象
 *      用 (void *) 显式去掉限定符是安全的。
 * ============================================================================ */
#include "globaldatadef.h"
#include <string.h>

/* 软件示波器缓冲：4 通道 × 900 点，约 4*SCOPENUM 个字。
 * 默认放在 .bss；若 RAMLS5 放不下，可打开下面的 DATA_SECTION 把它挪到
 * 自定义段 "datascope_buf"（需要链接命令文件里同步分配该段）。 */
//#pragma DATA_SECTION(DataScope, "datascope_buf")
volatile DATASCOPE DataScope;  /* EPWM10 ISR 写、1ms CAN 任务读写 */

MDADDRMES uMdlAddMes;          /* 多机并联地址表与均流样本，通信任务维护 */
volatile SCANMES   sCanMes;    /* CanComm 写，DataCalc 读 */
volatile CFGParam  CfgParam;   /* 通信任务写，ZERO ISR 读给定/增益 */
CanMonitorData_t g_CanMon;     /* 0x11 遥测快照，DataCalc 任务周期刷新 */

void init_global_data(void)
{
    /* 上电清空：必须在开中断之前调用（main 的 InitDSP 后、EINT 前） */
    memset((void *)&DataScope, 0, sizeof(DataScope)); /* 显式去掉 volatile 限定，仅上电初始化用 */
    memset(&uMdlAddMes, 0, sizeof(uMdlAddMes));
    memset((void *)&sCanMes, 0, sizeof(sCanMes)); /* 显式去掉 volatile 限定，仅上电初始化用 */
    memset((void *)&CfgParam, 0, sizeof(CfgParam));
    memset(&g_CanMon, 0, sizeof(g_CanMon));

    /* 默认运行参数（单位见 globaldatadef.h 里 CFGParam 各字段注释）：
     * 上位机可通过 0x20 命令按字偏移改写，这里只是掉电后的默认值。 */
    CfgParam.BatChgCurSet = 1600;    /* 电池充电电流 160.0A */
    CfgParam.BatDischgCurSet = 1600; /* 电池放电电流 160.0A */
    CfgParam.BusChgCurSet = 200;     /* 母线充电电流  20.0A */
    CfgParam.BusDischgCurSet = 200;  /* 母线放电电流  20.0A */
    CfgParam.BusVolSet = 4000;       /* 母线电压给定 400.0V */
    CfgParam.BatVolSet = 400;        /* 电池电压给定  40.0V */
    CfgParam.OpenLoopDuty = 100;     /* 开环测试占空比 10.0% */
}
