/*
***********************************************************************
* 文件名：adc_drv.h
* 作  者：???
* 日  期：2025/10/23
* 说  明：ADC驱动
***********************************************************************
*/

#ifndef _ADC_DRV_H
#define _ADC_DRV_H

#include "main.h"

typedef struct
{
	float Vbus;	   // 母线电压
	float Vr;	   // 电位器
	uint16_t Temp; // 温度

	float CurU; // U相电流
	float CurV; // V相电流
	float CurW; // W相电流

	int16_t OffsetCurU; // U相电流偏置值
	int16_t OffsetCurV; // V相电流偏置值
	int16_t OffsetCurW; // W相电流偏置值
} ADC_Sample;

#define ADC_Sample_DEFAULTS {0, 0, 0, 0, 0, 0} // 初始化参数

typedef struct
{
	float Vbus; // 母线电压
	float Temp;

	float CurU;
	float CurV;
	float CurW;

	int16_t OffsetCurU;
	int16_t OffsetCurV;
	int16_t OffsetCurW;
} ADC_Sample_F;

#define ADC_Sample_F_DEFAULTS {0, 0, 0, 0, 0, 0} // 初始化参数

typedef struct
{
	float Yn;
	float Yn_last;
	float Xn;
	float Xn_last;
	float Tau;
} V_Comp, *P_Vcomp;

#define Voltage_Compensation_DEFAULTS {0, 0, 0, 0, 0}

extern uint16_t ADC_Value[5]; // ADC接收数组
extern uint8_t NTC_ERROR;	  // NTC温度报警

extern ADC_Sample AdcPara;
extern ADC_Sample_F AdcFilPara;

void AdcSampleOffset(void);						 // 校准零漂
void GetAdc(void);								 // 获得滤波后采样数据
void TempTrans(uint16_t ADC_value, float *temp); // NTC温度采样

#endif
