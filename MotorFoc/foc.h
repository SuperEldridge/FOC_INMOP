#ifndef __FOC_H
#define __FOC_H

#include "delay.h"

typedef struct
{
	float Iu;	 // 三相电流A
	float Iv;	 // 三相电流B
	float Iw;	 // 三相电路C
	float Alpha; // 二相静止坐标系 Alpha 轴
	float Beta;	 // 二相静止坐标系 Beta 轴
} CLARKE, *M_CLARKE;

typedef struct
{
	float Alpha; // 二相静止坐标系 Alpha 轴
	float Beta;	 // 二相静止坐标系 Beta 轴
	float Theta; // 电机电角度
	float Ds;	 // 电机二相旋转坐标系下的d轴电流
	float Qs;	 // 电机二相旋转坐标系下的q轴电流
} PARK, *M_PARK;

typedef struct
{
	float Alpha; // 二相静止坐标系 Alpha 轴
	float Beta;	 // 二相静止坐标系 Beta 轴
	float Theta; // 电机电角度
	float Ds;	 // 电机二相旋转坐标系下的d轴电流
	float Qs;	 // 电机二相旋转坐标系下的q轴电流
} IPARK, *M_IPARK;

#define CLARKE_DEFAULTS {0, 0, 0, 0, 0} // CLARK初始化参数
#define PARK_DEFAULTS {0, 0, 0, 0, 0}	// PARK初始化参数
#define IPARK_DEFAULTS {0, 0, 0, 0, 0}	// IPARK初始化参数

/* 按照公式计算Tx+Ty可能会大于Ts,所以需要对实际的Tx和Ty进行修正 */
#define MODIFY_TX_TY(x, y, z, s) \
	{                            \
		z = x + y;               \
		if (x + y > s)           \
		{                        \
			x = x / z * s;       \
			y = y / z * s;       \
		}                        \
	}

typedef struct
{
	float Ualpha;	   // 二相静止坐标系alpha-轴
	float Ubeta;	   // 二相静止坐标系beta-轴
	float Ta;		   // 三相矢量占空比Ta
	float Tb;		   // 三相矢量占空比Tb
	float Tc;		   // 三相矢量占空比Tc
	float u1;		   // 三相静止坐标系的电压temp1
	float u2;		   // 三相静止坐标系的电压temp2
	float u3;		   // 三相静止坐标系的电压temp3
	float T0;		   // 零矢量作用时间
	uint8_t VecSector; // 矢量空间扇区号
} SVPWM, *M_SVPWM;

#define SVPWM_DEFAULTS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // 初始SVPWM化参数

extern SVPWM Svpwm;

void SvpwmCal(M_SVPWM pv);

void ClarkeCale(M_CLARKE pv); // Clark变换
void ParkCale(M_PARK pv);	  // Park变换
void IPARK_Cale(M_IPARK pv);  // Park反变换
#endif
