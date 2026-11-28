/*************************************************************************************************
Copyright 2011, celink Corporation.									                                    	*								                                            			*
File name:		ConfigChip_DspInit.h							                                       	*
Description:		DSP Initial 							                                				*
Update Date:		 2022/5/22								                                    			*
Writer:			zy									                                            	*
*************************************************************************************************/

#ifndef INCLUDE_CONFIGCHIP_DSPINIT_H
#define INCLUDE_CONFIGCHIP_DSPINIT_H


#define RED_LED_ON		            asm("    NOP")
#define RED_LED_OFF		            asm("    NOP")
#define RED_LED_TOGGLE		        asm("    NOP")

#define GREEN_LED_ON			    asm("    NOP")
#define GREEN_LED_OFF			    asm("    NOP")
#define GREEN_LED_TOGGLE			asm("    NOP")

//defined by link file
//App请求升级标志位
extern volatile uint16_t FromAppRequestFlag;

extern void InitDSP(void);

#endif /* INCLUDE_CONFIGCHIP_DSPINIT_H */
