/*************************************************************************************************
 * App_AdcDataDeal.c — ADC 采样池 + DAB 直流三档慢滤波
 *
 * 硬件 SOC/双触发/均分在 ConfigChip_AdcInit；RESULT 别名在 App_AdcDataDeal.h（ADRESULT_*）
 * 本文件：池清零 + Task_1ms 三档低通（τ=10/100/400ms）；陷波见 Interrupt_Notch2f
 *************************************************************************************************/
#include "Includes.h"
#include "App_DabModel.h"   /* DAB_BAT_VOLT_MIN：电池电压合法区间 */

/* 零点平均的倒数(编译期常量) */
#define ADC_CURR_ZERO_INV   (1.0f / (float)ADC_CURR_ZERO_N)

/* 全局采样池：全工程只在这一处定义（含采样数据 + 校准参数） */
volatile ST_ADC_POOL stADC;

static Uint16 s_u16BatVolt100msSeeded;

/* 用瞬时值并行推进三档一阶低通 */
static inline void AdcLpf3Update(volatile ST_ADC_LPF3 *pFilt, float32 fIn)
{
    pFilt->f32Filt10ms =
        fIn * ADC_DAB_FILT_10MS_NEW + pFilt->f32Filt10ms * ADC_DAB_FILT_10MS_OLD;
    pFilt->f32Filt100ms =
        fIn * ADC_DAB_FILT_100MS_NEW + pFilt->f32Filt100ms * ADC_DAB_FILT_100MS_OLD;
    pFilt->f32Filt400ms =
        fIn * ADC_DAB_FILT_400MS_NEW + pFilt->f32Filt400ms * ADC_DAB_FILT_400MS_OLD;
}

void AdcCtrlInit(void)
{
    /* 整池清零，避免上电垃圾值进控制/滤波状态；电流零点回到中点 */
    memset((void *)&stADC, 0, sizeof(stADC));
    s_u16BatVolt100msSeeded = 0U;
    stADC.CurrOffset.f32Idd1 = BUS_CURR_MID_ADC;
    stADC.CurrOffset.f32Idd2 = BUS_CURR_MID_ADC;
}

/**
 * 电流过零点校准（中点偏置采样）
 *
 * 前提：EPWM 时基已跑（SOCA 在刷新），功率级封波 / 零电流。
 * 做法：IDD1、IDD2 各累加 ADC_CURR_ZERO_N 次 raw，平均写入 offset；
 *       ISR：I = (RESULT − offset) × BUS_CURR_RATIO。
 * 若均值偏离中点过大（硬件异常），保留 BUS_CURR_MID_ADC 不覆盖。
 */
void Adc_CalibCurrZero(void)
{
    Uint16 i;
    float sum1 = 0.0f;
    float sum2 = 0.0f;
    float avg1;
    float avg2;

    for (i = 0U; i < ADC_CURR_ZERO_N; i++)
    {
        /* >1 个开关周期，保证双 SOCA 至少各刷新一次（50 kHz→20 µs） */
        DEVICE_DELAY_US(30U);
        sum1 += (float)ADRESULT_IDD1_ADC;
        sum2 += (float)ADRESULT_IDD2_ADC;
    }

    avg1 = sum1 * ADC_CURR_ZERO_INV;
    avg2 = sum2 * ADC_CURR_ZERO_INV;

    /* 合理窗：中点 ±512 码（约 ±11 A @ 现比例），否则不采纳 */
    if ((avg1 > (BUS_CURR_MID_ADC - 512.0f)) && (avg1 < (BUS_CURR_MID_ADC + 512.0f)))
    {
        stADC.CurrOffset.f32Idd1 = avg1;
    }
    if ((avg2 > (BUS_CURR_MID_ADC - 512.0f)) && (avg2 < (BUS_CURR_MID_ADC + 512.0f)))
    {
        stADC.CurrOffset.f32Idd2 = avg2;
    }
}

/**
 * 1ms：对各 DAB 直流通道并行更新 10ms / 100ms / 400ms 三档低通
 */
void AdcDabFilt1ms(void)
{
    float iAbs;

    AdcLpf3Update(&stADC.BusVolt1.Filt, stADC.BusVolt1.f32IsrResult);
    AdcLpf3Update(&stADC.BusVolt2.Filt, stADC.BusVolt2.f32IsrResult);
    AdcLpf3Update(&stADC.VmBusVolt.Filt, stADC.VmBusVolt.f32IsrResult);

    AdcLpf3Update(&stADC.PcsBusVolt.Filt, stADC.PcsBusVolt.f32IsrResult);
    if (stADC.PcsBusVolt.Filt.f32Filt10ms < 0.0f)
    {
        stADC.PcsBusVolt.Filt.f32Filt10ms = 0.0f;
    }

    AdcLpf3Update(&stADC.BusCurr1.Filt, stADC.BusCurr1.f32IsrResult);
    AdcLpf3Update(&stADC.BusCurr2.Filt, stADC.BusCurr2.f32IsrResult);

    static ST_ADC_LPF3 st_lpf3 = {0.0f};
    const float32 f32BatVolt = stADC.BatVolt.f32IsrResult;

    /* 欠压保护使用 100ms 滤波：首拍用实测值初始化，消除从 0V 爬升导致的误报。 */
    if (s_u16BatVolt100msSeeded == 0U)
    {
        st_lpf3.f32Filt100ms = f32BatVolt;
        s_u16BatVolt100msSeeded = 1U;
    }
    AdcLpf3Update(&st_lpf3, f32BatVolt);

    /* 电池电压 10ms 滤波值先限幅到合法区间 [DAB_BAT_VOLT_MIN, 65V] 再发布：
     * 上电时滤波器从 0 爬升、以及瞬时越界都会夹回区间内，下游只取值不再判范围。 */
    if (st_lpf3.f32Filt10ms <= DAB_BAT_VOLT_MIN)
    {
        st_lpf3.f32Filt10ms = DAB_BAT_VOLT_MIN;
    }
    else if (st_lpf3.f32Filt10ms >= 65.0f)
    {
        st_lpf3.f32Filt10ms = 65.0f;
    }

    stADC.BatVolt.Filt.f32Filt10ms = st_lpf3.f32Filt10ms;

    stADC.BatVolt.Filt.f32Filt100ms     = st_lpf3.f32Filt100ms;
    stADC.BatVolt.Filt.f32Filt400ms     = st_lpf3.f32Filt400ms;

    AdcLpf3Update(&stADC.BatCurr.Filt, stADC.BatCurr.f32IsrResult);

    /* 功率粗估：PCS 端 400ms 压 × (Ibus1+Ibus2) 400ms */
    iAbs = stADC.BusCurr1.Filt.f32Filt400ms + stADC.BusCurr2.Filt.f32Filt400ms;
    if (iAbs < 0.0f)
    {
        iAbs = -iAbs;
    }
    stADC.f32BusPowerFilt = stADC.PcsBusVolt.Filt.f32Filt400ms * iAbs;
}
