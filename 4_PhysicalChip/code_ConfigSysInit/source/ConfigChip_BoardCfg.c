/*************************************************************************************************
 * ConfigChip_BoardCfg.c  —  板级初始化（F28P550SG，双桥 DAB）
 * 资源表：6.2~10KWH_DSP资源配置表_F28P550SG.xlsx
 *************************************************************************************************/
#include "Includes.h"
#include "ConfigChip_DabPwmMap.h"

static void Board_InitPeripheralClocks(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER2 = 1U;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1U;

    /* 桥1 */
    CpuSysRegs.PCLKCR2.bit.EPWM3  = 1U;
    CpuSysRegs.PCLKCR2.bit.EPWM10 = 1U;
    CpuSysRegs.PCLKCR2.bit.EPWM11 = 1U;
    CpuSysRegs.PCLKCR2.bit.EPWM12 = 1U;
    /* 桥2 */
    CpuSysRegs.PCLKCR2.bit.EPWM2  = 1U;
    CpuSysRegs.PCLKCR2.bit.EPWM4  = 1U;
    CpuSysRegs.PCLKCR2.bit.EPWM7  = 1U;
    CpuSysRegs.PCLKCR2.bit.EPWM9  = 1U;

    CpuSysRegs.PCLKCR10.bit.MCAN_A = 1U;
    CpuSysRegs.PCLKCR7.bit.SCI_B = 1U;   /* ARM 通讯 SCIB */

    CpuSysRegs.PCLKCR13.bit.ADC_A = 1U;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1U;
    CpuSysRegs.PCLKCR13.bit.ADC_C = 1U;

    /* CMPSS：VMBUS_OVP / IDD1_OCP / IDD2_OCP */
    CpuSysRegs.PCLKCR14.bit.CMPSS1 = 1U;
    CpuSysRegs.PCLKCR14.bit.CMPSS2 = 1U;
    CpuSysRegs.PCLKCR14.bit.CMPSS4 = 1U;
    EDIS;
}

static void Board_InitAnalogReference(void)
{
    ASysCtl_setAnalogReferenceExternal(ASYSCTL_ANAREF_VREFHI_ADCA |
                                       ASYSCTL_ANAREF_VREFHI_ADCB |
                                       ASYSCTL_ANAREF_VREFHI_ADCC);
}

static void Board_CfgEpwmPin(uint32_t pinConfig, uint16_t gpio)
{
    GPIO_setPinConfig(pinConfig);
    GPIO_setPadConfig(gpio, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(gpio, GPIO_DIR_MODE_OUT);
}

static void Board_InitEpwmGpio(void)
{
    EALLOW;
    /* 桥1 BUS */
    Board_CfgEpwmPin(GPIO_62_EPWM10_A, DAB_GPIO_DH1_LH);
    Board_CfgEpwmPin(GPIO_63_EPWM10_B, DAB_GPIO_DH1_LL);
    Board_CfgEpwmPin(GPIO_20_EPWM12_A, DAB_GPIO_DH1_RH);
    Board_CfgEpwmPin(GPIO_21_EPWM12_B, DAB_GPIO_DH1_RL);
    /* 桥1 BAT */
    Board_CfgEpwmPin(GPIO_8_EPWM11_A,  DAB_GPIO_DL1_LH);
    Board_CfgEpwmPin(GPIO_4_EPWM11_B,  DAB_GPIO_DL1_LL);
    Board_CfgEpwmPin(GPIO_0_EPWM3_A,   DAB_GPIO_DL1_RH);
    Board_CfgEpwmPin(GPIO_1_EPWM3_B,   DAB_GPIO_DL1_RL);
    /* 桥2 BUS */
    Board_CfgEpwmPin(GPIO_12_EPWM7_A,  DAB_GPIO_DH2_LH);
    Board_CfgEpwmPin(GPIO_13_EPWM7_B,  DAB_GPIO_DH2_LL);
    Board_CfgEpwmPin(GPIO_16_EPWM9_A,  DAB_GPIO_DH2_RH);
    Board_CfgEpwmPin(GPIO_17_EPWM9_B,  DAB_GPIO_DH2_RL);
    /* 桥2 BAT */
    Board_CfgEpwmPin(GPIO_41_EPWM2_A,  DAB_GPIO_DL2_LH);
    Board_CfgEpwmPin(GPIO_40_EPWM2_B,  DAB_GPIO_DL2_LL);
    Board_CfgEpwmPin(GPIO_22_EPWM4_A,  DAB_GPIO_DL2_RH);
    Board_CfgEpwmPin(GPIO_7_EPWM4_B,   DAB_GPIO_DL2_RL);
    EDIS;
}

static void Board_InitMcanGpio(void)
{
    EALLOW;
    GPIO_setPinConfig(GPIO_31_MCANA_TX);
    GPIO_setPadConfig(BOARD_GPIO_MCAN_TX, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(BOARD_GPIO_MCAN_TX, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(BOARD_GPIO_MCAN_TX, GPIO_DIR_MODE_OUT);

    GPIO_setPinConfig(GPIO_30_MCANA_RX);
    GPIO_setPadConfig(BOARD_GPIO_MCAN_RX, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(BOARD_GPIO_MCAN_RX, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(BOARD_GPIO_MCAN_RX, GPIO_DIR_MODE_IN);
    EDIS;
}

/* 资源表：GPIO14=SCIB_TX / GPIO15=SCIB_RX（ARM 通讯） */
static void Board_InitSciGpio(void)
{
    EALLOW;
    GPIO_setPinConfig(GPIO_14_SCIB_TX);
    GPIO_setPadConfig(BOARD_GPIO_SCIB_TX, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(BOARD_GPIO_SCIB_TX, GPIO_DIR_MODE_OUT);

    GPIO_setPinConfig(GPIO_15_SCIB_RX);
    GPIO_setPadConfig(BOARD_GPIO_SCIB_RX, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(BOARD_GPIO_SCIB_RX, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(BOARD_GPIO_SCIB_RX, GPIO_DIR_MODE_IN);
    EDIS;
}

static void Board_CfgOutLow(uint32_t pinCfg, uint16_t gpio)
{
    GPIO_setPinConfig(pinCfg);
    GPIO_setPadConfig(gpio, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(gpio, GPIO_DIR_MODE_OUT);
    GPIO_writePin(gpio, 0U);
}

/* 全部数字 DO：复用/方向/初值；App 层只负责 writePin */
static void Board_InitDigitalOut(void)
{
    EALLOW;
    Board_CfgOutLow(GPIO_55_GPIO55, BOARD_GPIO_RELAY_BUS1P);
    Board_CfgOutLow(GPIO_60_GPIO60, BOARD_GPIO_RELAY_BUS1N);
    Board_CfgOutLow(GPIO_33_GPIO33, BOARD_GPIO_RELAY_BUS2P);
    Board_CfgOutLow(GPIO_11_GPIO11, BOARD_GPIO_RELAY_BUS2N);
    Board_CfgOutLow(GPIO_25_GPIO25, BOARD_GPIO_DRV_EN);
    Board_CfgOutLow(GPIO_26_GPIO26, BOARD_GPIO_SOFT_RLY);
    Board_CfgOutLow(GPIO_27_GPIO27, BOARD_GPIO_HEAT_PWM);
    Board_CfgOutLow(GPIO_56_GPIO56, BOARD_GPIO_HEAT_EN);
    EDIS;
}

/**
 * AGPIO → ADC：先复用成 GPIO，再开模拟模式（Device_initGPIO 会默认关掉模拟）。
 * 无 AGPIO 的纯模拟脚（如 A4/A5/A8/B11）无需本函数。
 */
static void Board_CfgAdcPin(uint32_t pinCfg, uint16_t gpio)
{
    GPIO_setPinConfig(pinCfg);
    GPIO_setAnalogMode(gpio, GPIO_ANALOG_ENABLED);
}

/* 资源表里带 GPIO/AIO 号的 ADC 通道全部在此切到模拟 */
static void Board_InitAnalogGpio(void)
{
    EALLOW;
    /*
     * ADC 模拟脚：与 ConfigChip_AdcInit 三模块均分一致。
     * 电流首采 IDD1@A7、IDD2@B8 无 AGPIO，硬件直采，无需在此配置。
     * 同网并联脚（如 A10/C10）只配一次 GPIO 模拟模式即可。
     */
    Board_CfgAdcPin(GPIO_238_GPIO238, 238U); /* A12     → ADCA VMBUS_ADC（继电器 VM） */
    Board_CfgAdcPin(GPIO_248_GPIO248, 248U); /* C1      → ADCC VBUS1_ADC */
    Board_CfgAdcPin(GPIO_245_GPIO245, 245U); /* A7      → ADCA IDD1_ADC 首采 */
    Board_CfgAdcPin(GPIO_230_GPIO230, 230U); /* A10/C10 → ADCC VDC_ADC（采 C10） */
    Board_CfgAdcPin(GPIO_236_GPIO236, 236U); /* B4/C8   → ADCB VPBUS_ADC（PCS端电压） */
    Board_CfgAdcPin(GPIO_252_GPIO252, 252U); /* B5      → ADCB VMBUS_OVP */

    /* B8 IDD2 首采 / A5 IDD1_OCP / A8 IDD2_OCP / B11 VBUS2：无 AGPIO */

    Board_CfgAdcPin(GPIO_228_GPIO228, 228U); /* A6      → ADCA PACK_NTC */
    Board_CfgAdcPin(GPIO_226_GPIO226, 226U); /* B2/C6   → ADCC HV_NTC（采 C6） */
    Board_CfgAdcPin(GPIO_28_GPIO28,    28U); /* A16/C16 → ADCC BATT_NTC（采 C16） */
    Board_CfgAdcPin(GPIO_224_GPIO224, 224U); /* A2/C9   → ADCC MODEL2（采 C9） */
    EDIS;
}

void Board_Init(void)
{
    Board_InitPeripheralClocks();
    Board_InitAnalogReference();
    Board_InitEpwmGpio();
    Board_InitMcanGpio();
    Board_InitSciGpio();
    Board_InitDigitalOut();  /* 继电器 / DRV / 软启 / 加热膜：脚配置 */
    Board_InitAnalogGpio();
    AppGpio_Init();          /* App：功率侧安全态 */
    HeatFilm_Init();         /* App：加热膜软件状态 */
}

void Board_LedToggle(void)
{
}

static void Board_PwmPinToGpioIn(uint32_t pinCfg, uint16_t gpio)
{
    GPIO_setPinConfig(pinCfg);
    GPIO_setPadConfig(gpio, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(gpio, GPIO_DIR_MODE_IN);
}

void Board_PrepareForBoot(void)
{
    DAB_PWM_OstStop();
    DAB_PWM_Stop();

    AppGpio_SafeOff();

    EALLOW;
    /* PWM 脚改回普通输入，避免复位窗口误驱动；SCIB(GPIO14/15) 保留 ARM 通讯 */
    Board_PwmPinToGpioIn(GPIO_62_GPIO62, DAB_GPIO_DH1_LH);
    Board_PwmPinToGpioIn(GPIO_63_GPIO63, DAB_GPIO_DH1_LL);
    Board_PwmPinToGpioIn(GPIO_20_GPIO20, DAB_GPIO_DH1_RH);
    Board_PwmPinToGpioIn(GPIO_21_GPIO21, DAB_GPIO_DH1_RL);
    Board_PwmPinToGpioIn(GPIO_8_GPIO8,   DAB_GPIO_DL1_LH);
    Board_PwmPinToGpioIn(GPIO_4_GPIO4,   DAB_GPIO_DL1_LL);
    Board_PwmPinToGpioIn(GPIO_0_GPIO0,   DAB_GPIO_DL1_RH);
    Board_PwmPinToGpioIn(GPIO_1_GPIO1,   DAB_GPIO_DL1_RL);
    Board_PwmPinToGpioIn(GPIO_12_GPIO12, DAB_GPIO_DH2_LH);
    Board_PwmPinToGpioIn(GPIO_13_GPIO13, DAB_GPIO_DH2_LL);
    Board_PwmPinToGpioIn(GPIO_16_GPIO16, DAB_GPIO_DH2_RH);
    Board_PwmPinToGpioIn(GPIO_17_GPIO17, DAB_GPIO_DH2_RL);
    Board_PwmPinToGpioIn(GPIO_41_GPIO41, DAB_GPIO_DL2_LH);
    Board_PwmPinToGpioIn(GPIO_40_GPIO40, DAB_GPIO_DL2_LL);
    Board_PwmPinToGpioIn(GPIO_22_GPIO22, DAB_GPIO_DL2_RH);
    Board_PwmPinToGpioIn(GPIO_7_GPIO7,   DAB_GPIO_DL2_RL);
    EDIS;
}
