/*************************************************************************************************
Copyright 2011, CELink Corporation.									                                    *
VERSION:		V00									                                            			*
File name:		ConfigChip_SpiInit.h								                                       	*
Description:	DSP SPI Initial 							                                				*
Update Date:	2018/11/22								                                    				*
Writer:			zy									                                            	*
*************************************************************************************************/

#ifndef INCLUDE_CONFIGCHIP_SPIINIT_H
#define INCLUDE_CONFIGCHIP_SPIINIT_H


void InitChip_SpiRegs(void);

extern volatile struct SPI_REGS *pCommSpiRegs;

#endif /* INCLUDE_CFG_DSPINIT_S_SPIINIT_H */
