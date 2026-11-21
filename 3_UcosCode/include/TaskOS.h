/*************************************************************************************************
Copyright 2022, celink Corporation.
File name:		 TaskOS.h
Description:	 Task Application Include Head file
Update Date:	 2026/11/20
Writer:			 zy
*************************************************************************************************/
#ifndef INCLUDE_TASKOS_H
#define INCLUDE_TASKOS_H

#define 	OS_ENTER_CRITICAL()		DINT
#define 	OS_EXIT_CRITICAL()		EINT

/*******************************************************************/
/*				                 Task Macro			               */
/*******************************************************************/
/* 五档节拍片：Timer2 100us tick，周期 = setTimeCnt × 0.1ms */
#define PRIO_TASK_100US				0
#define PRIO_TASK_1MS				1
#define PRIO_TASK_2MS				2
#define PRIO_TASK_5MS				3
#define PRIO_TASK_50MS				4
#define	MAX_TASK_NUM					5

/*******************************************************************/
/*          			event definition						   */
/*******************************************************************/
#define TASK_TIMER                  0

extern Uint32 u32Tick_Count;
extern volatile Uint32 u32TaskRunTimeUs[MAX_TASK_NUM];		/* P1：各任务最近一次耗时 us */
extern volatile Uint32 u32TaskRunTimeMaxUs[MAX_TASK_NUM];	/* P1：各任务历史最大耗时 us */
extern __interrupt void OSTimerTick(void);

extern void OSISREventSend(Uint16 prio,Uint16 EventId);
extern void OSEventSend(Uint16 prio,Uint16 EventId);
extern void OSISREventClr(Uint16 prio,Uint16 EventId);
extern Uint16 OSEventConfirm(Uint16 prio, Uint16 EventId);

extern void Task_System_AppInit(void);
extern void Task_TimerCheckIn(void);
extern void Task_InvokingProcess(void);

#endif /* INCLUDE_TASKOS_H */
