#ifndef __HFI_H
#define __HFI_H

#include "delay.h"
#include "stdint.h"
#include "stdbool.h"

typedef struct
{
	float theta;		 // HFI观测的角度
	float theta_err;	 // HFI观测的角度误差
	float Ialpha_last;	 // 上一次采样的α轴电流
	float Ibeta_last;	 // 上一次采样的β轴电流
	float ialpha_h;		 // α轴高频电流分量
	float ibeta_h;		 // β轴高频电流分量
	float ialpha_h_last; // 上一次的α轴高频电流分量
	float ibeta_h_last;	 // 上一次的β轴高频电流分量
	float ialpha_f;		 // α轴低频电流分量
	float ibeta_f;		 // β轴低频电流分量
	float Ialpha_h;		 // 经过包络检测后的α轴高频电流分量
	float Ibeta_h;		 // 经过包络检测后的β轴高频电流分量
	float Idf;			 // d轴低频电流
	float Iqf;			 // q轴低频电流
	float Ids;
	float Iqs;
	float Iq_last;	  // q轴上次采样电流
	float Id_last;	  // d轴上次采样电流
	float Idh;		  // d轴高频响应电流
	int8_t Uin;		  // d轴注入的高频电压
	int8_t SIGN;	  // 符号函数后的结果
	float theta_Init; // 初始角度
} HFI_Cal, *p_HFI;

#define HFI_DEFAULTS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 状态参数初始化

typedef struct
{
	float Kp;			  // 锁相环PI控制器Kp
	double Ki;			  // 锁相环PI控制器Ki
	float Theta_Err;	  // 角度误差
	float Theta_Err_last; // 上一次角度误差
	float Omega;		  // 电角速度
	float Omega_F;		  // 滤波后的电角速度
	float Theta;		  // 锁相环计算得到的电角度
	float Angle;		  // 经过角度补偿的电角度
} HFI_PLL, *p_HFI_PLL;

#define HFI_PLL_DEFAULTS {0, 0, 0, 0, 0, 0, 0, 0}; // 初始化参数

typedef struct
{
	float Kp;			  // 锁相环PI控制器Kp
	double Ki;			  // 锁相环PI控制器Ki
	float Theta_Err;	  // 角度误差
	float Theta_Err_last; // 上一次角度误差
	float Omega;		  // 电角速度
	float Omega_F;		  // 滤波后的电角速度
	float Theta;		  // 锁相环计算得到的电角度
	float Angle;		  // 经过角度补偿的电角度

	float Theta_last; // 上一次的电角度
	float Omega2;
	float Omega2_F;
} PLL, *p_PLL;
extern PLL PllSmoPara;
extern PLL PllHfiPara;

#define PLL_DEFAULTS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 初始化参数

extern bool HfiUinFlag;

extern HFI_Cal HFI;

void HfiAngleCale(p_HFI pv);		  // HFI角度识别
void HFI_Injection(void);			  // HFI测试，注入高频方波脉冲
void HfiInitTheta(void);			  // HFI初始角度识别
void HfiSpdCloseLoop(void);			  // HFI速度闭环
void HfiObserver(float Id, float Iq); // 高频方波注入观测器
void ExtractIdqF(void);				  // 提取低频电流分量
void PllCale(p_PLL pv, float alpha, float beta);

#endif
