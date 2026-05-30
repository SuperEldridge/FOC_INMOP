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

uint16_t ADC_Value[5];

// 校准零漂
void AdcSampleOffset(void)
{
	uint8_t i = 0;
	uint8_t j = 0;
	uint16_t Sum1 = 0;
	uint16_t Sum2 = 0;

	for (i = 0; i < 10; i++)
	{
		for (j = 0; j < 100; j++)
		{
			AdcPara.OffsetCurU = ADC_Value[0];
			AdcPara.OffsetCurW = ADC_Value[3];

			HAL_Delay(1);
		}
		Sum1 += AdcPara.OffsetCurU;
		Sum2 += AdcPara.OffsetCurW;
	}
	AdcFilPara.OffsetCurU = Sum1 / 10;
	AdcFilPara.OffsetCurW = Sum2 / 10;
}

// 电流原始数据采样
static void AdcSample(void)
{
	AdcPara.CurU = (float)(AdcFilPara.OffsetCurU - ADC_Value[0]) * I_GAIN;
	AdcPara.CurW = (float)(AdcFilPara.OffsetCurW - ADC_Value[3]) * I_GAIN;
	AdcPara.CurV = -AdcPara.CurU - AdcPara.CurW;

	AdcPara.Vbus = (float)ADC_Value[2] * VBUS_GAIN;
	AdcPara.Temp = (float)ADC_Value[1];
	AdcPara.Vr = (float)ADC_Value[4];
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
