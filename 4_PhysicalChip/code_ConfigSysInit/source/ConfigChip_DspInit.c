/*************************************************************************************************
Copyright 2022, CELink Corporation.									                                    	*
VERSION:										                                            			*
File name:		ConfigChip_DspInit.c								                                       	*
Description:		DSP Initial 							                                				*
Update Date:		 								                                    			*
Writer:			zy									                                            	*
*************************************************************************************************/

#include "Includes.h"

void InitChip_CpuTimerRegs(void);

void InitChip_CpuTimerRegs(void)
{
    // Initialize the Device Peripheral. For this example, only initialize the
    // Cpu Timers.
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 150MHz CPU Freq（PLL 实际输出：20MHz XTAL ×30 /2 /2，见 device.h / InitSysPll），Period in uSeconds
//    ConfigCpuTimer(&CpuTimer0, 150, 100);// 100us
//    ConfigCpuTimer(&CpuTimer1, 150, 10000);// 10ms
    ConfigCpuTimer(&CpuTimer2, 150, 100);   // 100us：PRD = 15000-1 @150MHz，硬件自动重装，周期恒定无累计偏差

    // To ensure precise timing, use write-only instructions to write to the
    // entire register. Therefore, if any of the configuration bits are changed
    // in ConfigCpuTimer and InitCpuTimers, the below settings must also be
    // be updated.
//    CpuTimer0Regs.TCR.all = 0x4000;
//    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

}

void InitDSP(void)
{
	// Step 1. Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the F2806x_SysCtrl.c file.

	InitChip_SysCtrlRegs();    

	// Step 2. Initalize GPIO:
	// This example function is found in the F2806x_Gpio.c file and
	// illustrates how to set the GPIO to it's default state.
    InitGpio();
/* ★ 外设引脚配置完全由 Board_Init()（p550 Bsp_BoardCfg 移植本）完成，与 p550 对齐：
 *   引脚复位由上面的 InitGpio() 完成，等效 p550 的 Device_initGPIO()；
 *   旧板（280039 遗留）的 InitChip_GpioRegs 与板级 LED/SCIA 调试脚配置均已移除：
 *   - InitChip_GpioRegs 与资源表冲突（GPIO2/29 PWM 双驱、GPIO50/51 MCAN 双驱、
 *     GPIO23 SCIB_RX 双输入、GPIO20/21/13→TZ1/2/3 占用 EPWM 脚）；
 *   - LED1/LED2（GPIO31/34）与 SCIA（GPIO28/29）配置 SCIA 从未初始化，
 *     且 GPIO31 与 Board_Init 的 MCANA_TX 冲突，一并删除。 */
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    InitPieVectTable();

	InitChip_CpuTimerRegs();
	InitChip_AdcRegs();
    InitChip_EpwmRegs();
	InitChip_CmpssRegs();

	InitChip_SpiRegs();

	//InitChip_McanRegs();	/* MCAN 初始化由 init_Drv_can() 在 main 中完成（ConfigChip_McanInit.c） */

	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	EALLOW;  // This is needed to write to EALLOW protected registers

	PieVectTable.TIMER2_INT = &OSTimerTick;


//    PieVectTable.ADCA1_INT = &INTERRUPT_ISR_ADC;


	//PieVectTable.EPWM1_TZ_INT = &Boost_OverCurrProtect;  	//trip2 select event;  //BB_OCP
	PieVectTable.EPWM10_INT = &EPWM10_ISR;

    //PieVectTable.SCIA_RX_INT = &DebugComSciRxISR;
    //PieVectTable.SCIA_TX_INT = &DebugComSciTxISR;

	//PieVectTable.SCIB_RX_INT = &InterCommSciRxISR;
	//PieVectTable.SCIB_TX_INT = &InterCommSciTxISR;

	//PieVectTable.SPIB_RX_INT = &SpiRxISR;
	//PieVectTable.SPIB_TX_INT = &SpiTxISR;
	
	EDIS;

	DINT;
	IER = 0x0000;	// CPU Interrupt Enable PIE Group 3
	IFR = 0x0000;
	
//	IER = (M_INT2 | M_INT3 | M_INT6 | M_INT9 | M_INT14);
//	IER = (M_INT1 |M_INT2 |M_INT3 | M_INT9 | M_INT14);
	IER = (M_INT3| M_INT14);

//	PieCtrlRegs.PIEIER1.bit.INTx1 = 1; 	// ADCA1_INT

	//PieCtrlRegs.PIEIER2.bit.INTx1 = 1;	//epwm1_TZint
	
	PieCtrlRegs.PIEIER3.bit.INTx10 = 1; 	// EPWM10_INT（DAB 控制，EPWM10 零点中断）

	//PieCtrlRegs.PIEIER4.bit.INTx2 = 0; 	// ECAP2_INT

	//PieCtrlRegs.PIEIER6.bit.INTx3 = 1; 	// SPIRXINTB
	//PieCtrlRegs.PIEIER6.bit.INTx4 = 1; 	// SPITXINTB	

	//PieCtrlRegs.PIEIER9.bit.INTx1 = 1; 	// SCIRXINTA
	//PieCtrlRegs.PIEIER9.bit.INTx2 = 1; 	// SCITXINTA

	//PieCtrlRegs.PIEIER9.bit.INTx3 = 1; 	// SCIRXINTB
	//PieCtrlRegs.PIEIER9.bit.INTx4 = 1; 	// SCITXINTB


}

