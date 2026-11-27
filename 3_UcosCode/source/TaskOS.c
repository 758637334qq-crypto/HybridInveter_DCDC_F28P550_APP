/*************************************************************************************************
Copyright 2022, celink Corporation.
File name:		TaskOS.c
Description:	Task function
Update Date:	2022/5/19
Writer:			zy
*************************************************************************************************/

#include "Includes.h"

//-----------------------------------------------------------------------------
//Adding Private Enumerated and Structure Definitions
//-----------------------------------------------------------------------------
typedef void (*TaskFunction)(void);
typedef struct
{
	TaskFunction func; 
	Uint16  u16timeCnt;
	Uint16  u16setTimeCnt;
	Uint16 	u16taskToInvoke;
}TimerTaskBoxStruct;

//-----------------------------------------------------------------------------
//Adding Private Variables (i.e. static)
//-----------------------------------------------------------------------------
/* u16setTimeCnt 单位：0.1ms（Timer2 tick = 100us），10 = 1ms。
 * P2 错相：1/2/5/50ms 任务初值取 1/2/3/4 —— 两任务同 tick 到期的充要条件是
 * vA≡vB(mod gcd(PA,PB))，六对组合(1,2)(1,5)(2,5)(1,50)(2,50)(5,50)全部错开，
 * 任意 tick 至多 100us+1 个任务到期，消解 50ms 边界四任务叠峰对 100us 节拍的推迟；
 * 各任务周期不变，仅首拍相位平移 ≤4 tick。 */
const TimerTaskBoxStruct constTaskBoxTable[MAX_TASK_NUM] =
{
		{Task100us,  		0, 1,   0},
		{Task1ms,           1, 10,  0},
		{Task2ms,           2, 20,  0},
		{Task5ms,           3, 50,  0},
		{Task50ms,          4, 500, 0},
};

TimerTaskBoxStruct taskBoxTable[MAX_TASK_NUM] =
{
		{Task100us,  		0, 1,   0},
		{Task1ms,           1, 10,  0},
		{Task2ms,           2, 20,  0},
		{Task5ms,           3, 50,  0},
		{Task50ms,          4, 500, 0},
};

/* P1：任务耗时统计（us）。Task_InvokingProcess 维护，调试器 Watch / 遥测只读；
 * 用途：验证同一 tick 多任务叠峰总耗时是否超出 100us 预算（超出则 100us 节拍被推迟）。 */
volatile Uint32 u32TaskRunTimeUs[MAX_TASK_NUM] = {0};
volatile Uint32 u32TaskRunTimeMaxUs[MAX_TASK_NUM] = {0};

/* 与 ConfigChip_DspInit.c 的 ConfigCpuTimer(&CpuTimer2,150,100) 同源：150MHz，150 计数 = 1us */
#define TASKOS_SYSCLK_MHZ   (150U)
static Uint16 s_u16MeasureTimerInit = 0;	/* CpuTimer1 测量时基已初始化标志 */

/* P1：CpuTimer1 全工程空闲（ConfigChip_DspInit.c 仅留注释配置，外设时钟已在
 * ConfigChip_SysCtrlInit.c 使能、InitCpuTimers 后保持停止态）。配置为 150MHz 32bit 自由
 * 下行计数，专用于任务耗时测量：32bit@150MHz 回绕周期约 28.6s 远大于任务耗时，无需回绕
 * 处理；不使能中断，不影响任何现有功能。 */
static void sTaskOS_MeasureTimerInit(void)
{
	CPUTimer_setEmulationMode(CPUTIMER1_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
	CPUTimer_setPreScaler(CPUTIMER1_BASE, 0U);
	CPUTimer_setPeriod(CPUTIMER1_BASE, 0xFFFFFFFFU);
	CPUTimer_stopTimer(CPUTIMER1_BASE);
	CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
	CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);
	CPUTimer_startTimer(CPUTIMER1_BASE);
}

void OSISREventSend(Uint16 prio,Uint16 EventId)
{
    taskBoxTable[prio].u16taskToInvoke|=(1<<EventId);
}

void OSEventSend(Uint16 prio,Uint16 EventId)
{
    taskBoxTable[prio].u16taskToInvoke|=(1<<EventId);
}

void OSISREventClr(Uint16 prio,Uint16 EventId)
{
    taskBoxTable[prio].u16taskToInvoke &=(~(1<<EventId));
}

Uint16 OSEventConfirm(Uint16 prio, Uint16 EventId)
{
    if((taskBoxTable[prio].u16taskToInvoke) & (1 << EventId))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void Task_System_AppInit(void)
{
	Uint16 index;

	for (index = 0; index < MAX_TASK_NUM; index++)
	{
		taskBoxTable[index].func = constTaskBoxTable[index].func;
		taskBoxTable[index].u16timeCnt = constTaskBoxTable[index].u16timeCnt;
		taskBoxTable[index].u16setTimeCnt = constTaskBoxTable[index].u16setTimeCnt;
		taskBoxTable[index].u16taskToInvoke = constTaskBoxTable[index].u16taskToInvoke;	
	}
}

void Task_TimerCheckIn(void)
{
	Uint16 index;

	for (index = 0; index < MAX_TASK_NUM; index++)
	{
		taskBoxTable[index].u16timeCnt++;
		if(taskBoxTable[index].u16timeCnt >=  taskBoxTable[index].u16setTimeCnt)
		{
			taskBoxTable[index].u16timeCnt = 0;
			OSISREventSend(index,TASK_TIMER);
		}
	}
}

/* 到期才执行：Timer2 ISR(100us) 按各任务 u16setTimeCnt 计数并置 TASK_TIMER 事件，
 * 主循环每圈只跑已到期的任务 —— 五档节拍(100us/1ms/2ms/5ms/50ms)严格按 tick 分频；
 * 同一任务多次到期时，若待执行位一直未被主循环取走，只执行一次。
 * p550 移植任务内部无事件门，必须在此统一门控（v102 旧任务内部自带 OSEventConfirm，已删除）。
 * P1：执行前后读 CpuTimer1 TIM 测任务耗时（含被 ISR 抢占的墙钟时间），写入
 * u32TaskRunTimeUs/u32TaskRunTimeMaxUs，用于验证各 tick 叠峰总耗时是否超出 100us 预算。 */
void Task_InvokingProcess(void)
{
	Uint16 index;
	Uint32 u32TimStart;
	Uint32 u32TimEnd;

	if (s_u16MeasureTimerInit == 0U)
	{
		sTaskOS_MeasureTimerInit();		/* 首次进入启动 CpuTimer1 测量时基 */
		s_u16MeasureTimerInit = 1U;
	}

	for (index = 0; index < MAX_TASK_NUM; index++)
	{
		if ((taskBoxTable[index].u16taskToInvoke & (1U << TASK_TIMER)) != 0U)
		{
			OSISREventClr(index, TASK_TIMER);

			u32TimStart = CpuTimer1Regs.TIM.all;
			(*taskBoxTable[index].func)();
			u32TimEnd = CpuTimer1Regs.TIM.all;

			/* TIM 下行计数：耗时计数 = start - end（无回绕）；150MHz 下 150 计数 = 1us */
			u32TaskRunTimeUs[index] = (u32TimStart - u32TimEnd) / TASKOS_SYSCLK_MHZ;
			if (u32TaskRunTimeUs[index] > u32TaskRunTimeMaxUs[index])
			{
				u32TaskRunTimeMaxUs[index] = u32TaskRunTimeUs[index];
			}
		}
	}
}

Uint32 u32Tick_Count = 0;
__interrupt void OSTimerTick(void)
{
	Uint16 u16SavedIER;

	CPUTimer_clearOverflowFlag(CPUTIMER2_BASE);

	/* ★ 嵌套窗口（TI spra876 标准做法）：放行 EPWM10 控制中断（PIE 组3，50/60kHz
	 * 硬实时）抢占本节拍 ISR，避免控制环被推迟。窗口期 IER 只留 INT3：
	 * PIEIER3 仅使能 INTx10，其余中断一律不放行；不放行 INT14 自身，杜绝节拍重入。
	 * INT14 直连 CPU 不走 PIE，本侧无需应答；EPWM10 抢占后自行写 PIEACK_GROUP3。
	 * 本体只动 TaskOS/TaskSched 数据，与 EPWM10 控制环数据无交集，抢占数据安全。 */
	u16SavedIER = IER;
	IER = (Uint16)M_INT3;
	EINT;

	Task_TimerCheckIn();

	TaskSched_TimeBase100us();	/* P0：维护 TaskSched 对外时基，Sys_GetTick100us/1ms/10ms 恢复有效 */

	u32Tick_Count++;

	DINT;					/* 先关全局中断再恢复 IER，退出前无空窗 */
	IER = u16SavedIER;
}
