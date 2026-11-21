/*
 * Sys.h
 *
 *  Created on: 2025年4月25日
 *      Author: wangxj
 */

#ifndef _SYS_TASKSCHED_H_
#define _SYS_TASKSCHED_H_

#include "stdint.h"
#include "stddef.h"

#define MAX_TASK_LEN 10		//最大任务数
#define INVALID_FLAG -1		//非法任务标志

#define MASK_DAY_31 0x15AA
#define MASK_DAY_30 0xA50
#define BASE_YEAR_30 2026U

#define IS_LEAP_YEAR(y) (((y) % 4 == 0 && (y) % 100 != 0) || ((y) % 400 == 0))
#define GET_MONTH_DAYS(days, year, month) do {\
    if ((1U << month) & MASK_DAY_31) days = 31; \
    else if ((1U << month) & MASK_DAY_30) days = 30;\
    else {days = IS_LEAP_YEAR(year) ? 29 : 28;} }while(0)// February

/* 调度时基只读接口：勿直接读写 TaskSched.u32TickTime */
extern uint32_t Sys_GetTick100us(void);   /* 100us 计数 */
extern uint32_t Sys_GetTick1ms(void);     /* 1ms 计数 */
extern uint32_t Sys_GetTick10ms(void);    /* 10ms 计数 */
extern void TaskSched_TimeBase100us(void); /* 100us 时基维护：由 TaskOS.OSTimerTick 每拍调用 */

// 系统轮询延时机制（基于 1ms 时基）
//使用方法 STATE_MACHINE_BEGIN  STATE_MACHINE_DELAY(ms) STATE_MACHINE_DELAY(ms)... STATE_MACHINE_END()
#define STATE_MACHINE_BEGIN() static uint32_t state = 0,delay_start = 0; switch (state) { case 0:
#define STATE_MACHINE_END() state = 0;default:state = 0;delay_start = 0;return;}
#define STATE_MACHINE_DELAY(ms) \
    do { \
        delay_start = Sys_GetTick1ms(); \
        state = __LINE__; return; case __LINE__: \
        if ((Sys_GetTick1ms() - delay_start) < (uint32_t)(ms)) return; \
    } while (0)

/*************************类型定义*************************/
/*************Task**********/
typedef void (*fTask)(void *arg);	//回调型函数定义
typedef struct
{
    fTask task;
    uint32_t u32TaskTick;		//上次运行Tick用于检测是否到时间
    uint32_t u32TaskPrd;		//任务运行周期
    uint32_t u32TaskRunTime;	//任务运行耗时，单位 0.1us
    uint32_t u32TaskRunTimeMax;	//任务最大耗时，单位 0.1us
}tTask;

typedef union//时间联合体
{
    struct
    {
        uint32_t year : 6;
        uint32_t month : 4;
        uint32_t day : 5;
        uint32_t hour : 5;
        uint32_t min : 6;
        uint32_t sec : 6;
    }t;
    uint32_t all;
}UTime;

typedef struct
{
	uint32_t	u32TickTime;				/* 调度内核100us时基，仅调度器读写；应用请用 Sys_GetTickXxx() */
	tTask 		TaskArray[MAX_TASK_LEN];	//任务列表
}STU_TASKSCHED_DEF;

//
#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

/*************************函数声明*************************/
/*************Task**********/
extern int Sys_TaskAdd(uint16_t TaskPrd, fTask tTaskFunc);
extern int  Sys_TaskAddDelay(int cycTimeMs, fTask tTaskFunc);
extern void Sys_TaskDelete(fTask tTaskFunc);
extern void Sys_TaskInit(void);
extern void Sys_TaskRun(void);
extern void Sys_TaskTimeTickAdd(void);
extern void Sys_TaskTickCntReg(uint32_t *p,uint32_t Prd);
extern int  Sys_TaskFind(fTask tTaskFunc);
extern void Sys_Init(void);
extern void Sys_TimeProc(UTime *pRtc);

#endif /* SYS_SYS_H_ */
