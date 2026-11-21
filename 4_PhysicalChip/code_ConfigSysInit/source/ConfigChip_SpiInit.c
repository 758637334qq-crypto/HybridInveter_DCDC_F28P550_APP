/*************************************************************************************************
Copyright 2022, CELink Corporation.									                                    	*
VERSION:											                                            			*
File name:		ConfigChip_SpiInit.c								                                       	*
Description:	DSP SPI Initial 							                                				*
Update Date:								                                    			*
Writer:			zy									                                            	*
*************************************************************************************************/

#include "Includes.h"

volatile struct SPI_REGS *pCommSpiRegs = &SpibRegs;

void InitChip_SpiRegs(void)
{	
    // Initialize SPI FIFO registers
	pCommSpiRegs->SPICCR.bit.SPISWRESET=0; // Reset SPI

	pCommSpiRegs->SPICCR.all=0x001F;       //16-bit character, Loopback mode
	
	pCommSpiRegs->SPICTL.all=0x0017;       //Interrupt enabled, Master/Slave XMIT enabled
	
	pCommSpiRegs->SPISTS.all=0x0000;
	
	pCommSpiRegs->SPIBRR.bit.SPI_BIT_RATE = 127;           // Baud rate Low speed clock= SYSCLKOUT/4, SPI Baud rate =LSPCLK/(SPIBRR+1)=200K
	
	pCommSpiRegs->SPIFFTX.all=0xC021;      // Enable FIFO's, set TX FIFO level to 1,TXFIFO Interrupt Enable,TXFIFO Interrupt clear
	
	pCommSpiRegs->SPIFFRX.all=0x0021;      // Set RX FIFO level to 1, RXFIFO Interrupt Enable
	
	pCommSpiRegs->SPIFFCT.all=0x00;
	
	pCommSpiRegs->SPIPRI.all=0x0010;   //Free emulation mode control

	pCommSpiRegs->SPICCR.bit.SPILBK = 0;  // no loopback mode

	pCommSpiRegs->SPICTL.bit.CONTROLLER_PERIPHERAL = 0; // slave enable

	pCommSpiRegs->SPICCR.bit.SPISWRESET=1;  // SPI Software Reset
	pCommSpiRegs->SPIFFTX.bit.TXFIFO=1;
	pCommSpiRegs->SPIFFRX.bit.RXFIFORESET=1;
   
}

