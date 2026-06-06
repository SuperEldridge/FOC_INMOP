/*
***********************************************************************
* 文件名：adc_drv.c
* 作  者：？？？
* 日  期：2024/10/23
* 说  明：ADC驱动
***********************************************************************
*/

#include "task.h"

ADC_Sample AdcPara = ADC_Sample_DEFAULTS;
ADC_Sample_F AdcFilPara = ADC_Sample_F_DEFAULTS;

uint16_t ADC_Value[3];
#define ADC_CUR_U_INJ() ((uint16_t)ADC1->JDR1)
#define ADC_CUR_W_INJ() ((uint16_t)ADC1->JDR2)
// 电流零偏校准
void AdcSampleOffset(void)
{
      uint16_t i = 0;
      uint32_t sum_u = 0;
      uint32_t sum_w = 0;
      /* 
       * 先等待注入组触发稳定：
       * 此时 TIM1 已经在运行，ADC injected 已经由 TIM1_CH4 触发更新 JDR1/
       * 这里延时仅用于跨过启动瞬态，避免把未稳定数据当作零偏。
       */
      HAL_Delay(20);
      /*
       * 连续累计多次 injected 结果做真实平均。
       * 注意：这里必须“每次累加”，不能像旧代码那样只取循环最后一次值。
       */
      for (i = 0; i < 512; i++)
      {
        sum_u += ADC_CUR_U_INJ();
        sum_w += ADC_CUR_W_INJ();
        HAL_Delay(1);
      }
      AdcFilPara.OffsetCurU = (int16_t)(sum_u / 512U);
      AdcFilPara.OffsetCurW = (int16_t)(sum_w / 512U);
}

// 电流原始数据采样
static void AdcSample(void)
{
	uint16_t raw_u = ADC_CUR_U_INJ();
  uint16_t raw_w = ADC_CUR_W_INJ();
	AdcPara.CurU = (float)(AdcFilPara.OffsetCurU - raw_u) * I_GAIN;
	AdcPara.CurW = (float)(AdcFilPara.OffsetCurW - raw_w) * I_GAIN;
	AdcPara.CurV = -AdcPara.CurU - AdcPara.CurW;

	AdcPara.Vbus = (float)ADC_Value[1] * VBUS_GAIN;
	AdcPara.Temp = (float)ADC_Value[0];
	AdcPara.Vr = (float)ADC_Value[2];
}

// 采样数据过低通滤波
static void AdcSampleDeal(void)
{
	AdcFilPara.CurU = LPF_I_RUN_B * AdcFilPara.CurU + LPF_I_RUN_A * AdcPara.CurU;
	AdcFilPara.CurV = LPF_I_RUN_B * AdcFilPara.CurV + LPF_I_RUN_A * AdcPara.CurV;
	AdcFilPara.CurW = LPF_I_RUN_B * AdcFilPara.CurW + LPF_I_RUN_A * AdcPara.CurW;
	AdcFilPara.Vbus = AdcFilPara.Vbus * LPF_I_STOP_B + AdcPara.Vbus * LPF_I_STOP_A;
	AdcFilPara.Temp = AdcFilPara.Temp * LPF_I_STOP_B + AdcPara.Temp * LPF_I_STOP_A;
}

// 获得相电流和母线电压
void GetAdc(void)
{
	AdcSample();
	AdcSampleDeal();
}
