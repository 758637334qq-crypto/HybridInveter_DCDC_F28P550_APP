/*************************************************************************************************
Copyright 2022, celink Corporation.									                                    	*								                                            			*
File name:		ConfigChip_SysCtrlInit.c								                                       	*
Description:		DSP System Control Initialization & Support Functions.							                                				*
Update Date:		 2022/5/22								                                    			*
Writer:			zy									                                            	*
*************************************************************************************************/
#include "f28P55x_device.h"         // f28003x Headerfile Include File
#include "f28P55x_examples.h"       // f28003x Examples Include File
#include "device.h"                 // Board oscillator selection used by the PLL setup

// Functions that will be run from RAM need to be assigned to
// a different section.  This section will then be mapped to a load and
// run address using the linker cmd file.
//
//  *IMPORTANT*
//  IF RUNNING FROM FLASH, PLEASE COPY OVER THE SECTION ".TI.ramfunc"  FROM FLASH
//  TO RAM PRIOR TO CALLING InitSysCtrl(). THIS PREVENTS THE MCU FROM THROWING 
//  AN EXCEPTION WHEN A CALL TO DELAY_US() IS MADE. 
//
#pragma CODE_SECTION(InitFlash, ".TI.ramfunc");

void InitChip_SysCtrlRegs(void);
void InitPeripheralClocks_User(void);

//---------------------------------------------------------------------------
// InitSysCtrl:
//---------------------------------------------------------------------------
// This function initializes the System Control registers to a known state.
// - Disables the watchdog
// - Set the PLLCR for proper SYSCLKOUT frequency
// - Set the pre-scaler for the high and low frequency peripheral clocks
// - Enable the clocks to the peripherals

void InitChip_SysCtrlRegs(void)
{
    //
    // Disable the watchdog
    //
    DisableDog();

#ifdef _FLASH
    //
    // Copy time critical code and Flash setup code to RAM. This includes the
    // following functions: InitFlash()
    //
    // The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the device .cmd file.
    //
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

    //
    // Call Flash Initialization to setup flash waitstates. This function must
    // reside in RAM.
    //
    InitFlash();
#endif

    //
    // Initialize the SYSPLL for the 150 MHz system clock.
    //
    // Defined options to be passed as arguments to this function are defined
    // in f28003x_examples.h.
    //
    // Note: The internal oscillator CANNOT be used as the PLL source if the
    // PLLSYSCLK is configured to frequencies above 194 MHz.
    //
    //  PLLSYSCLK = (XTAL_OSC) * (IMULT) /(REFDIV) * (ODIV) * (PLLSYSCLKDIV)
    //
#if defined(USE_PLL_SRC_XTAL)
    InitSysPll(XTAL_OSC, IMULT_30, REFDIV_2, ODIV_1, PLLCLK_BY_2, SYSCTL_DCC_BASE0);
#elif defined(USE_PLL_SRC_INTOSC)
    InitSysPll(INT_OSC2, IMULT_30, REFDIV_1, ODIV_1, PLLCLK_BY_2, SYSCTL_DCC_BASE0);
#else
#error "Select a PLL source in device.h"
#endif

    /* LSPCLK = SYSCLK/4 = 37.5MHz，与 device.h 的 DEVICE_LSPCLK_FREQ 一致（SCI/SPI 波特依赖） */
    EALLOW;
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 2U;   /* 2 → ÷4（复位默认值，此处显式锁定） */
    EDIS;

#ifndef _FLASH
    //
    // Call Device_cal function when run using debugger
    // This function is called as part of the Boot code. The function is called
    // in the InitSysCtrl function since during debug time resets, the boot code
    // will not be executed and the gel script will reinitialize all the
    // registers and the calibrated values will be lost.
    //
    Device_cal();
#endif


	// Initialize the peripheral clocks
	//InitPeripheralClocks();    //from SDK
	InitPeripheralClocks_User();

    /* REVID==1 硅片 ADC/PGA 勘误（对齐 TI device.c Device_init）：须在外设时钟使能之后 */
    if (HWREG(DEVCFG_BASE + SYSCTL_O_REVID) == 1U)
    {
        PGA_setOffsetTrimNMOS(PGA2_BASE);
        PGA_setOffsetTrimPMOS(PGA2_BASE);

        EALLOW;
        HWREG(ADCA_BASE + 0x66U) |= 0x00000C00U;
        HWREG(ADCB_BASE + 0x66U) |= 0x00000C00U;
        HWREG(ADCC_BASE + 0x66U) |= 0x00000C00U;
        HWREG(ADCD_BASE + 0x66U) |= 0x00000C00U;
        HWREG(ADCE_BASE + 0x66U) |= 0x00000C00U;

        if ((HWREGH(ADCA_BASE + ADC_O_OFFTRIM) % 2U) != 0U)
        {
            HWREGH(ADCA_BASE + ADC_O_OFFTRIM) += 1U;
        }
        if ((HWREGH(ADCB_BASE + ADC_O_OFFTRIM) % 2U) != 0U)
        {
            HWREGH(ADCB_BASE + ADC_O_OFFTRIM) += 1U;
        }
        if ((HWREGH(ADCC_BASE + ADC_O_OFFTRIM) % 2U) != 0U)
        {
            HWREGH(ADCC_BASE + ADC_O_OFFTRIM) += 1U;
        }
        if ((HWREGH(ADCD_BASE + ADC_O_OFFTRIM) % 2U) != 0U)
        {
            HWREGH(ADCD_BASE + ADC_O_OFFTRIM) += 1U;
        }
        if ((HWREGH(ADCE_BASE + ADC_O_OFFTRIM) % 2U) != 0U)
        {
            HWREGH(ADCE_BASE + ADC_O_OFFTRIM) += 1U;
        }
        EDIS;
    }

    /* 本芯片不支持 VREGCTL，锁寄存器防误写（对齐 TI device.c） */
    ASysCtl_lockVREG();
}

//--------------------------------------------------------------------------
// Example: InitPeripheralClocks:
//---------------------------------------------------------------------------
// This function initializes the clocks to the peripheral modules.
// First the high and low clock prescalers are set
// Second the clocks are enabled to each peripheral.
// To reduce power, leave clocks to unused peripherals disabled
//
// Note: If a peripherals clock is not enabled then you cannot
// read or write to the registers for that peripheral

void InitPeripheralClocks_User(void)
{
    EALLOW;

    CpuSysRegs.PCLKCR0.bit.CLA1 = 1;
     CpuSysRegs.PCLKCR0.bit.DMA = 1;
     CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;
     CpuSysRegs.PCLKCR0.bit.CPUTIMER1 = 1;
     CpuSysRegs.PCLKCR0.bit.CPUTIMER2 = 1;
     CpuSysRegs.PCLKCR0.bit.HRCAL = 1;
     CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
     CpuSysRegs.PCLKCR0.bit.ERAD = 1;

     CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
     CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
     CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;
     CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;
     CpuSysRegs.PCLKCR2.bit.EPWM5 = 1;
     CpuSysRegs.PCLKCR2.bit.EPWM6 = 1;
     CpuSysRegs.PCLKCR2.bit.EPWM7 = 1;
     CpuSysRegs.PCLKCR2.bit.EPWM8 = 1;
     CpuSysRegs.PCLKCR2.bit.EPWM9 = 1;
     CpuSysRegs.PCLKCR2.bit.EPWM10 = 1;
     CpuSysRegs.PCLKCR2.bit.EPWM11 = 1;
     CpuSysRegs.PCLKCR2.bit.EPWM12 = 1;


     CpuSysRegs.PCLKCR3.bit.ECAP1 = 1;
     CpuSysRegs.PCLKCR3.bit.ECAP2 = 1;

     CpuSysRegs.PCLKCR4.bit.EQEP1 = 1;
     CpuSysRegs.PCLKCR4.bit.EQEP2 = 1;
     CpuSysRegs.PCLKCR4.bit.EQEP3 = 1;

     CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
     CpuSysRegs.PCLKCR7.bit.SCI_B = 1;
     CpuSysRegs.PCLKCR7.bit.SCI_C = 1;

     CpuSysRegs.PCLKCR8.bit.SPI_A = 1;
     CpuSysRegs.PCLKCR8.bit.SPI_B = 1;

     CpuSysRegs.PCLKCR9.bit.I2C_A = 1;
     CpuSysRegs.PCLKCR9.bit.I2C_B = 1;

     CpuSysRegs.PCLKCR10.bit.MCAN_A = 1;
     CpuSysRegs.PCLKCR10.bit.MCAN_B = 1;

     CpuSysRegs.PCLKCR11.bit.USB_A = 1;

     CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
     CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
     CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
     CpuSysRegs.PCLKCR13.bit.ADC_D = 1;
     CpuSysRegs.PCLKCR13.bit.ADC_E = 1;

     CpuSysRegs.PCLKCR14.bit.CMPSS1 = 1;
     CpuSysRegs.PCLKCR14.bit.CMPSS2 = 1;
     CpuSysRegs.PCLKCR14.bit.CMPSS3 = 1;
     CpuSysRegs.PCLKCR14.bit.CMPSS4 = 1;

     CpuSysRegs.PCLKCR15.bit.PGA1 = 1;
     CpuSysRegs.PCLKCR15.bit.PGA2 = 1;
     CpuSysRegs.PCLKCR15.bit.PGA3 = 1;

     CpuSysRegs.PCLKCR16.bit.DAC_A = 1;

     CpuSysRegs.PCLKCR17.bit.CLB1 = 1;
     CpuSysRegs.PCLKCR17.bit.CLB2 = 1;

     CpuSysRegs.PCLKCR18.bit.FSITX_A = 1;
     CpuSysRegs.PCLKCR18.bit.FSIRX_A = 1;

     CpuSysRegs.PCLKCR19.bit.LIN_A = 1;

     CpuSysRegs.PCLKCR20.bit.PMBUS_A = 1;

     CpuSysRegs.PCLKCR21.bit.DCC0 = 1;
     CpuSysRegs.PCLKCR21.bit.DCC1 = 1;

     CpuSysRegs.PCLKCR26.bit.AESA = 1;
     CpuSysRegs.PCLKCR27.bit.EPG1 = 1;

    EDIS;
	
}


//===========================================================================
// End of file.
//===========================================================================
