/*************************************************************************************************
Copyright 2022, celink Corporation.									                                    	*									                                            			*
File name:		 TaskAppInclude.h								                                       	*
Description:		Task Application Include Head file 							                                				*
Update Date:		 2022/5/22								                                    			*
Writer:			zy									                                            	*
*************************************************************************************************/
#ifndef INCLUDE_TASKAPPINCLUDE_H
#define INCLUDE_TASKAPPINCLUDE_H


/*******************************************************************/
/*				                 Task Macro			               */
/*******************************************************************/
#define T2MS				    2
#define T10MS					10
#define T20MS				    20
#define T30MS				    30
#define T40MS				    40
#define T50MS				    50
#define T100MS		            100
#define T200MS		        	200
#define T500MS                  500
#define T1000MS				   	1000


/*******************************************************************/
/*          			event definition						   */
/*******************************************************************/

#define StdbyStep_BusSoft 		0
#define StdbyStep_RelayCheck 	1
#define StdbyStep_GridConnect 	2
#define StdbyStep_Fail			3



/*******************************************************************
	system time cnt timer tick=1ms  
*******************************************************************/
#define		cT2ms				    2		
#define 	cT4ms				    4
#define		cT10ms				    10
#define 	cT20ms				    20
#define 	cT50ms				    50
#define 	cT70ms				    70
#define 	cT100ms				    100
#define 	cT200ms				    200
#define 	cT500ms				    500

#define cDelay1s_10ms		    100
#define cDelay3s_10ms		    300
#define cDelay30s_10ms		    3000

// 20ms timer delay counter define

#define cDelay20ms_20ms			1
#define cDelay100ms_20ms		5
#define cDelay200ms_20ms		10
#define cDelay500ms_20ms		25
#define cDelay1000ms_20ms		50
#define cDelay1s_20ms		    50
#define cDelay1s5_20ms		    75
#define cDelay2s_20ms		    100
#define cDelay3s_20ms		    150
#define cDelay5s_20ms		    250
#define cDelay9s5_20ms		    475
#define cDelay10s_20ms		    500
#define cDelay11s5_20ms		    575
#define cDelay13s_20ms		    650
#define cDelay15s_20ms		    750
#define cDelay20s_20ms		    1000
#define cDelay30s_20ms		    1500
#define cDelay60s_20ms			3000
#define cDelay1Min_20ms		    3000
#define cDelay5min_20ms		    15000
#define cDelay10Min_20ms		30000
#define cDelay1hour_20ms		180000
#define cDelay2hour_20ms		360000

//--------------------------------------------------------------------
//     timer set count
//--------------------------------------------------------------------
#define TSK200MS_TIMER20MS  	10
#define TSK500MS_TIMER20MS      25
#define TSK1S_TIMER20MS      	50
#define TSK2S_TIMER20MS      	100
#define TSK3S_TIMER20MS     	150
#define TSK5S_TIMER20MS     	250
#define TSK10S_TIMER20MS     	500
#define TSK4Hour_TIMER20MS     	720000

#define TSK100MS_TIMER10MS     	10
#define TSK1S_TIMER10MS     	100
#define TSK5S_TIMER10MS     	500
#define TSK10S_TIMER10MS     	1000
#define TSK20S_TIMER10MS     	2000
#define TSK30S_TIMER10MS     	3000

#define TSK5S_TIMER50MS     	100

#define cOn			1
#define cOff		0

#define cOk  		1
#define cError	   	0

#define cYes  		1
#define cNo	   		0

#define cOver 		1
#define cStart 	   	0

#define	cActivePowerDerateOrder		0x01
#define	cTemprDerate				0x02
#define	cFrequencyDerate			0x04

#define	cTurnOnVoltNOK		0x02
#define	cTurnOnFreqNOK		0x04

#endif /* INCLUDE_TASKAPPINCLUDE_H */
