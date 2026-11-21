/*************************************************************************************************
Copyright 2022, celink Corporation.									                                    *							                                            *
File name:		main.c								                                        *
Description:	main function（p550 DAB 移植：App/Task/驱动三层，boot 与 cmd 保留本工程）	        *
Update Date:	2026/11/20								                                    *
Writer:			zy								                                            *
*************************************************************************************************/

#include "Includes.h"

#define BOOT_VER_ADDRESS  0x080012

#define PACK2(a, b) ((uint16_t)(a) | ((uint16_t)(b) << 8U))
#define PACK4(s)    { PACK2((s)[0], (s)[1]), PACK2((s)[2], (s)[3]) }
#define PACK10(s)   { PACK2((s)[0], (s)[1]), PACK2((s)[2], (s)[3]), \
                      PACK2((s)[4], (s)[5]), PACK2((s)[6], (s)[7]), \
                      PACK2((s)[8], (s)[9]) }

typedef struct
{
    uint16_t codeType[2];
    uint16_t keyword[2];
    uint16_t appLength[2];
    uint16_t version[5];
} AppInfo_t;

#pragma DATA_SECTION(stAppInfo, "Flash_AppInfo")
#pragma RETAIN(stAppInfo)
const AppInfo_t stAppInfo =
{
    .codeType  = PACK4("SDSP"),
    .keyword   = PACK4("DOHN"),
    .appLength = PACK4("LENG"),
    .version   = PACK10("VER:001.01")
};

//--------------------------------------------------------------------
//      MCU software version
//--------------------------------------------------------------------
Uint16 u6FirmwareCode[2];
Uint16 u16BootSoftwareVerison;
Uint16 u16BootSoftwareSubverison;
Uint16 u16SoftwareVerison;
Uint16 u16SoftwareSubverison;
Uint16 u16HardwareVerison =99;

void Software_Version_Handle(void)
{
	u16SoftwareVerison = (Uint16)(((stAppInfo.version[2] & 0x00FFU) - '0') * 100U +
								(((stAppInfo.version[2] >> 8U) & 0x00FFU) - '0') * 10U +
								((stAppInfo.version[3] & 0x00FFU) - '0'));
	u16SoftwareSubverison = (Uint16)(((stAppInfo.version[4] & 0x00FFU) - '0') * 10U +
								   (((stAppInfo.version[4] >> 8U) & 0x00FFU) - '0'));

	u6FirmwareCode[0] = stAppInfo.keyword[0];
	u6FirmwareCode[1] = stAppInfo.keyword[1];

	u16BootSoftwareVerison = (Uint16)(((*(const uint16_t *)(BOOT_VER_ADDRESS - 1U)) & 0x00FFU) - '0');
	u16BootSoftwareSubverison = (Uint16)((((*(const uint16_t *)BOOT_VER_ADDRESS) & 0x00FFU) - '0') * 10U +
	                                     ((((*(const uint16_t *)BOOT_VER_ADDRESS) >> 8U) & 0x00FFU) - '0'));
}

/*------------------------------------------------------------------------------------------------*/
/* p550 初始化序列（会 memset 全局量的调用必须放在开中断之前）：
 *   InitDSP（时钟/PIE/向量表/Timer2 100us/EPWM10 向量）→ 板级 → ADC/EPWM/CMPSS 底座
 *   → 串口 → 采样池/陷波 → 全局清零 → CAN → 故障/状态机 → 启动 PWM → 零漂校准 → 开中断 → 看门狗
 * 中断分工：EPWM10 ZERO ISR = DAB 控制；CPU-Timer2 ISR(100us) = TaskOS 五档节拍
 */
void main(void)
{
	InitDSP();					/* 时钟/PIE/InitPieVectTable/Timer2 100us/EPWM10_INT 向量 + PIEIER3.INTx10 */

	Software_Version_Handle();

	Board_Init();				/* 板级：外设时钟 + ePWM/MCAN/SCIB 引脚 + 模拟引脚 */

	CustomUart_Init();			/* SCIB 115200 底层字节通道 */
	UartProto_Init();			/* 自定义串口协议状态机 */
	BootProto_Init();			/* ARM 升级协议解析（字节由 UartProto_RxTask 直递，与 UartProto 共存） */
	AdcCtrlInit();				/* 采样池清零 */
	Notch2f_Init();				/* 100Hz 陷波系数 */

	init_global_data();			/* DataScope / CfgParam / g_CanMon 清零 */
	init_Drv_can();				/* MCAN 经典 CAN 500kbps */
	CanComm_Init();				/* CAN 协议层初始化 */
	Fault_Init();				/* 故障字清零，早于 ISR 首次置位 */
	SM_Init();					/* memset RunState/SysCmd */

	EPWMStartTimer();			/* 启动 PWM 时基（SOCA 开始刷新） */
	Adc_CalibCurrZero();		/* 封波零流下 IDD1/IDD2 各 100 次平均 → 电流过零校准 */

	EINT;						/* Enable Global interrupt INTM */
	ERTM;						/* Enable Global realtime interrupt DBGM */

	/* 使能看门狗：INTOSC1≈10MHz，PREDIV=512，PRESCALE=32 → WDCLK≈610Hz，溢出≈420ms */
	SysCtl_setWatchdogMode(SYSCTL_WD_MODE_RESET);
	SysCtl_setWatchdogPredivider(SYSCTL_WD_PREDIV_512);
	SysCtl_setWatchdogPrescaler(SYSCTL_WD_PRESCALE_32);
	SysCtl_serviceWatchdog();
	SysCtl_enableWatchdog();

	for(;;)
	{
		Task_InvokingProcess();		/* TaskOS 五档节拍：100us/1ms/2ms/5ms/50ms */
		SysCtl_serviceWatchdog();	/* 主循环每圈喂狗，后台卡死则复位 */
	}
}


//===========================================================================
// No more.
//===========================================================================
