#ifndef __SMO_H
#define __SMO_H

#include "main.h"
#include "delay.h"

typedef struct
{
	float Ualpha;	  // 两相静止坐标系α轴电压
	float Ubeta;	  // 两相静止坐标系β轴电压
	float Ialpha;	  // 两相静止坐标系α轴电流
	float Ibeta;	  // 两相静止坐标系β轴电流
	float Est_Ialpha; // 两相静止坐标系观测的α电流
	float Est_Ibeta;  // 两相静止坐标系观测的β电流
	float Ialpha_Err; // 两相静止坐标系α轴电流误差
	float Ibeta_Err;  // 两相静止坐标系β轴电流误差
	float Zalpha;	  // 滑膜观测器的α轴Z平面
	float Zbeta;	  // 滑膜观测器的β轴Z平面
	float Ealpha;	  // 滑膜观测的α轴反电动势
	float Ebeta;	  // 滑膜观测的β轴反电动势
	float Fsmopos;	  // 滑膜系数1
	float Gsmopos;	  // 滑膜系数2
	float E0;		  // 滑膜的电流误差的限幅值 0.5
	float atan_theta; // 反正切计算的角度
} Angle_SMO, *p_Angle_SMO;

#define Angle_SMO_DEFAULTS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // 初始化参数

#define PLL_DEFAULTS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 初始化参数

extern Angle_SMO AngleSmoPara;

extern float SMO_Angle1;
extern float SMO_H;

void SmoAngleCale(p_Angle_SMO pv); // 滑膜观测器
void SMO_Init(void);			   // 滑膜参数初始化
void SmoObserver(float Id, float Iq);
void SmoThetaCale(float Ialpha, float Ibeta, float Ualpha, float Ubeta);

#endif
