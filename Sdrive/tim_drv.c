/*
***********************************************************************
* 文件名：tim_drv.c
* 作  者：？？？
* 日  期：2024/06/20
* 说  明：定时器任务
***********************************************************************
*/

#include "task.h"

ENCOSPEED EnCoSpeed;

// 计算电机速度
void MotorSpeedGet(void)
{
	static float SpeedMrpmFil = 0.0f;

	EnCoSpeed.EncoderCnt = TIM3->CNT; // 当前CNT的值+溢出次数*定时器的脉冲触发数

	int32_t Diff = (int32_t)((uint32_t)EnCoSpeed.EncoderCnt - (uint32_t)EnCoSpeed.EncoLastCnt); // 利用模算术

	// 通用回绕处理（适用于上溢或下溢）
	if (Diff > (int32_t)(TIM3_PULSE / 2))
		Diff -= (int32_t)TIM3_PULSE;
	else if (Diff < -(int32_t)(TIM3_PULSE / 2))
		Diff += (int32_t)TIM3_PULSE;

	float SpeedMrpmTemp = ((float)Diff * ((float)ENCODER_FRQ / (float)ENCODE_PULSE));

	SpeedMrpmFil = (LPF_U_A * SpeedMrpmTemp) + (LPF_U_B * SpeedMrpmFil);

	Motor.speed_M_rpm = SpeedMrpmFil;

	Motor.speed_E_rpm = Motor.speed_M_rpm * Motor.P; // 电角速度 = 机械角速度 * 极对数
	EnCoSpeed.EncoLastCnt = EnCoSpeed.EncoderCnt;
}

// 计算电机电角度
void MotorAngleGet(void)
{
	Motor.E_theta = (float)TIM3->CNT / ENCODE_PULSE * PIX2 * Motor.P; // CNT/16384(编码器转一圈计数值)*2PI*极对数

	while (Motor.E_theta >= PIX2)
		Motor.E_theta -= PIX2;
	while (Motor.E_theta <= 0.0f)
		Motor.E_theta += PIX2;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // 定时器更新中断回调函数(htim:定时器句柄)
{
	if (htim == (&htim1)) // FOC控制		20KHz
	{
		GetAdc(); // 获取电流
		AxisDq(); // 坐标轴变换

		EnCoSpeed.SpeedClock++;
		if (EnCoSpeed.SpeedClock == SPEED_CNT) // 计算一次电机转速
		{
			EnCoSpeed.SpeedClock %= SPEED_CNT;
			MotorSpeedGet(); // 速度获取
		}

		MotorModeSel(); // 控制模式选择
	}
}
