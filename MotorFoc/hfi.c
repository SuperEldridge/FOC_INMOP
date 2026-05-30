#include "motor_ctrl.h"

HFI_Cal HFI = HFI_DEFAULTS;
PLL PllSmoPara = PLL_DEFAULTS;
PLL PllHfiPara = PLL_DEFAULTS;

// 正交锁相环
void PllCale(p_PLL pv, float alpha, float beta)
{
	float SmoThaComsat = 0; // 角度补偿

	pv->Theta_Err = -(alpha * arm_cos_f32(pv->Theta)) + (-beta) * arm_sin_f32(pv->Theta);	  // 通过矢量叉乘或者角度误差
	pv->Omega += pv->Kp * (pv->Theta_Err - pv->Theta_Err_last) + TS * pv->Ki * pv->Theta_Err; // 经过PI调节器获得转速，转速为弧度制电角速度

	if (SensorLess.Observer == 3)
	{
		pv->Omega_F = pv->Omega_F * LPF_PLL_B + pv->Omega * LPF_PLL_A;
	}
	else
	{
		IirButterworth(PllHfiPara.Omega, &PllHfiPara.Omega_F, &WE_IIR_LPF_Par);
	}

	pv->Theta_Err_last = pv->Theta_Err;
	pv->Theta += TS * pv->Omega; // 对电角速度进行积分获得电角度
	// 对电角度进行限幅
	if (pv->Theta >= PIX2)
		pv->Theta -= PIX2;
	else if (pv->Theta < 0)
		pv->Theta += PIX2;
	/*
		if(Theta_CNT == 4)				//执行频率2KHz
		{
			Theta_CNT = 0;

			if(pi_spd.Ref > 0)
			{
				if(pv->Theta < pv->Theta_last)
					pv->Omega2 = (pv->Theta + PIX2 - pv->Theta_last)*5000;
				else
					pv->Omega2 = (pv->Theta  - pv->Theta_last)*5000;
			}
			else
			{
				if(pv->Theta > pv->Theta_last)
					pv->Omega2 = (pv->Theta - (pv->Theta_last + PIX2))*5000;
				else
					pv->Omega2 = (pv->Theta  - pv->Theta_last)*5000;
			}
			pv->Omega2_F = pv->Omega2_F*LPF_PLL_B + pv->Omega2*LPF_PLL_A;

			pv->Theta_last = pv->Theta;
		}
	*/

	// 在SMO中，反电动势需要进行LPF，因此存在滞后，通过观测电角速度对电角度进行补偿
	SmoThaComsat = my_atan(my_abs(pv->Omega) / (-wc));
	pv->Angle = pv->Theta - SmoThaComsat;
	// 对补偿后的电角度进行限幅
	if (pv->Angle >= PIX2)
		pv->Angle -= PIX2;
	else if (pv->Angle < 0)
		pv->Angle += PIX2;
}

// HFI角度计算
void HfiAngleCale(p_HFI pv)
{
	// 提取高频电流分量
	pv->ialpha_h_last = pv->ialpha_h;
	pv->ibeta_h_last = pv->ibeta_h;

	pv->ialpha_h = (Clarke.Alpha - pv->Ialpha_last) * 0.5f;
	pv->ibeta_h = (Clarke.Beta - pv->Ibeta_last) * 0.5f;
	pv->ialpha_f = (Clarke.Alpha + pv->Ialpha_last) * 0.5f;
	pv->ibeta_f = (Clarke.Beta + pv->Ibeta_last) * 0.5f;

	pv->Ialpha_last = Clarke.Alpha;
	pv->Ibeta_last = Clarke.Beta;

	pv->SIGN = -Sign(pv->Uin);

	// 包络检测
	pv->Ialpha_h = (pv->ialpha_h - pv->ialpha_h_last) * pv->SIGN;
	pv->Ibeta_h = (pv->ibeta_h - pv->ibeta_h_last) * pv->SIGN;

	//	pv->theta = my_atan2(pv->Ialpha_h, pv->Ibeta_h);
	PllCale((p_PLL)&PllHfiPara, -pv->Ibeta_h, pv->Ialpha_h);
}
// 初始角度识别
bool HfiUinFlag;

void HfiInitTheta(void)
{
	static uint16_t HFI_Clock = 0;
	static float Sum_Id1 = 0;
	static float Sum_Id2 = 0;

	if (HFI_Init_Angle_flag == 0)
		HFI_Clock++;
	HfiUinFlag = !HfiUinFlag;
	//	HFI_Angle_Cale((p_HFI)&HFI);
	// 提取低频电流分量
	HFI.Idf = (Park.Ds + HFI.Id_last) * 0.5f;
	HFI.Iqf = (Park.Qs + HFI.Iq_last) * 0.5f;
	//	HFI.Idf = HFI.Idf*LPF_HFI_I_B + HFI.Ids*LPF_HFI_I_A;
	//	HFI.Iqf = HFI.Iqf*LPF_HFI_I_B + HFI.Iqs*LPF_HFI_I_A;
	// 提取高频电流分量
	HFI.Idh = (Park.Ds - HFI.Id_last) * 0.5f;

	HFI.Id_last = Park.Ds;
	HFI.Iq_last = Park.Qs;

	if (HFI_Clock < 400) // 20ms，识别的初始电角度收敛
	{
		pi_id.Ref = 0;
		pi_id.Fbk = HFI.Idf;
		PiController((M_PI_Control)&pi_id);
		//		pi_id.OutF = pi_id.OutF*LPF_HFI_B + pi_id.Out*LPF_HFI_A;
	}
	else if (HFI_Clock == 400) // 初始电角度赋值
	{
		HFI.theta_Init = PllHfiPara.Theta; // 初始角度赋值
		pi_id.Ref = 0;
		pi_id.Fbk = HFI.Idf;
		PiController((M_PI_Control)&pi_id);
		//		pi_id.OutF = pi_id.OutF*LPF_HFI_B + pi_id.Out*LPF_HFI_A;
	}
	else if ((HFI_Clock > 400) && (HFI_Clock <= 600)) // 10ms,Id偏置电流注入
	{
		pi_id.Ref = HFI_ID_OFFSET;
		pi_id.Fbk = HFI.Idf;
		PiController((M_PI_Control)&pi_id);
		//		pi_id.OutF = pi_id.OutF*LPF_HFI_B + pi_id.Out*LPF_HFI_A;
	}
	else if ((HFI_Clock > 600) && (HFI_Clock <= 610)) // 累计 高频电流分量*Id_offset
	{
		pi_id.Ref = HFI_ID_OFFSET;
		pi_id.Fbk = HFI.Idf;
		PiController((M_PI_Control)&pi_id);
		//		pi_id.OutF = pi_id.OutF*LPF_HFI_B + pi_id.Out*LPF_HFI_A;
		Sum_Id1 += my_abs(HFI.Idh * HFI_ID_OFFSET);
	}
	else if ((HFI_Clock > 610) && (HFI_Clock <= 800)) // 10ms,Id偏执电流归零
	{
		pi_id.Ref = 0;
		pi_id.Fbk = HFI.Idf;
		PiController((M_PI_Control)&pi_id);
		//		pi_id.OutF = pi_id.OutF*LPF_HFI_B + pi_id.Out*LPF_HFI_A;
	}
	else if ((HFI_Clock > 800) && (HFI_Clock <= 1000)) // 10ms,Id负偏置电流注入
	{
		pi_id.Ref = -HFI_ID_OFFSET;
		pi_id.Fbk = HFI.Idf;
		PiController((M_PI_Control)&pi_id);
		//		pi_id.OutF = pi_id.OutF*LPF_HFI_B + pi_id.Out*LPF_HFI_A;
	}
	else if ((HFI_Clock > 1000) && (HFI_Clock <= 1010)) // 累计 高频电流分量*Id_offset
	{
		pi_id.Ref = -HFI_ID_OFFSET;
		pi_id.Fbk = HFI.Idf;
		PiController((M_PI_Control)&pi_id);
		//		pi_id.OutF = pi_id.OutF*LPF_HFI_B + pi_id.Out*LPF_HFI_A;
		Sum_Id2 += my_abs(-HFI.Idh * HFI_ID_OFFSET);
	}
	else if (HFI_Clock > 1010) // 初始角度识别结束，根据Sum_Id1&2大小判断收敛在N极还是S极
	{
		HFI_Init_Angle_flag = 1;
		if (Sum_Id1 < Sum_Id2) // 收敛在S极，对角度进行补偿
		{
			HFI.theta_Init = PllHfiPara.Theta + PI;
			if (HFI.theta_Init > PIX2)
				HFI.theta_Init -= PIX2;

			//			HFI.theta = HFI.theta_Init;
			PllHfiPara.Theta = HFI.theta_Init;
		}
		Sum_Id1 = 0;
		Sum_Id2 = 0;

		pi_id.Ref = 0;
		pi_id.Fbk = HFI.Idf;
		PiController((M_PI_Control)&pi_id);
		//		pi_id.OutF = pi_id.OutF*LPF_HFI_B + pi_id.Out*LPF_HFI_A;
		HFI_Clock = 0;
	}

	// 注入的高频方波电压
	if (HfiUinFlag)
		HFI.Uin = HFI_UIN_OFFSET;
	else
		HFI.Uin = -HFI_UIN_OFFSET;

	// IPARK计算
	Ipark.Theta = PllHfiPara.Theta;
	Ipark.Qs = 0;
	Ipark.Ds = pi_id.Out + HFI.Uin; // d轴高频方波注入
	IPARK_Cale((M_IPARK)&Ipark);
}

// HFI速度闭环
void HfiSpdCloseLoop(void)
{
	float ISpeed_Ref_EX = 0.0f;

	SpdRefGXieLv.XieLv_Grad = SPD_REF_OD;	 // 定义速度的梯度值
	SpdRefGXieLv.Grad_Timer = SPD_REF_TIMER; // 定义速度的梯度时间
	SpdRefGXieLv.XieLv_X = Para.SpeedRef;	 // 定义梯度上限，即速度环目标速度

	if (HFI_Init_Angle_flag)
	{
		SpdRefGXieLv.Timer_Count++;
		if (SpdRefGXieLv.Timer_Count > SpdRefGXieLv.Grad_Timer)
		{
			SpdRefGXieLv.Timer_Count = 0;
			GradXieLv((p_GXieLv)&SpdRefGXieLv); // 梯度计算
		}
		ISpeed_Ref_EX = Limit_Sat(SpdRefGXieLv.XieLv_Y, HFI_SPD_REF_MAX, -HFI_SPD_REF_MAX);
	}

	HfiUinFlag = !HfiUinFlag;
	// 提取低频电流分量
	HFI.Idf = (Park.Ds + HFI.Id_last) * 0.5f;
	HFI.Iqf = (Park.Qs + HFI.Iq_last) * 0.5f;
	//	HFI.Idf = HFI.Idf*LPF_HFI_I_B + HFI.Ids*LPF_HFI_I_A;
	//	HFI.Iqf = HFI.Iqf*LPF_HFI_I_B + HFI.Iqs*LPF_HFI_I_A;

	HFI.Id_last = Park.Ds;
	HFI.Iq_last = Park.Qs;

	// 速度环
	if (Para.SpeedCalTime == SPD_CAL_PERIOD)
	{
		Para.SpeedCalTime %= SPD_CAL_PERIOD;

		pi_spd.Ref = ISpeed_Ref_EX * Motor.P;
		pi_spd.Fbk = PllHfiPara.Omega_F / PIX2 * 60;
		PiController((M_PI_Control)&pi_spd); // 速度环
											 //		pi_spd.OutF = pi_spd.OutF*LPF_I_RUN_B + pi_spd.Out*LPF_I_RUN_A; //环路滤波后输出
	}

	// 电流环
	pi_id.Ref = 0;
	pi_id.Fbk = HFI.Idf;
	PiController((M_PI_Control)&pi_id);
	//	pi_id.OutF = pi_id.OutF*LPF_HFI_B + pi_id.Out*LPF_HFI_A;

	// Iq轴闭环
	pi_iq.Ref = pi_spd.Out;
	pi_iq.Fbk = HFI.Iqf;
	PiController((M_PI_Control)&pi_iq);
	//	pi_iq.OutF = pi_iq.OutF*LPF_I_RUN_B + pi_iq.Out*LPF_I_RUN_A;

	// d轴注入的高频方波电压
	if (HfiUinFlag)
		HFI.Uin = HFI_UIN_OFFSET;
	else
		HFI.Uin = -HFI_UIN_OFFSET;

	// IPARK计算
	Ipark.Theta = PllHfiPara.Theta;
	Ipark.Qs = pi_iq.Out;
	Ipark.Ds = pi_id.Out + HFI.Uin;
	IPARK_Cale((M_IPARK)&Ipark);
}
// 提取低频电流分量
void ExtractIdqF(void)
{
	HFI.Idf = (Park.Ds + HFI.Id_last) * 0.5f;
	HFI.Iqf = (Park.Qs + HFI.Iq_last) * 0.5f;
	//	HFI.Idf = HFI.Idf*LPF_HFI_I_B + HFI.Ids*LPF_HFI_I_A;
	//	HFI.Iqf = HFI.Iqf*LPF_HFI_I_B + HFI.Iqs*LPF_HFI_I_A;

	HFI.Id_last = Park.Ds;
	HFI.Iq_last = Park.Qs;
}

// HFI速度环，全速度无感控制用
void HfiObserver(float Id, float Iq)
{
	HfiUinFlag = !HfiUinFlag;

	// 速度环
	if (Para.SpeedCalTime == SPD_CAL_PERIOD)
	{
		pi_spd.Ref = SensorLess.Speed_Ref;
		pi_spd.Fbk = SensorLess.Speed_Fbk;
		PiController((M_PI_Control)&pi_spd); // 速度环
											 //		pi_spd.OutF = pi_spd.OutF*LPF_I_RUN_B + pi_spd.Out*LPF_I_RUN_A; //环路滤波后输出
	}

	// 电流环
	pi_id.Ref = 0;
	pi_id.Fbk = Id;
	PiController((M_PI_Control)&pi_id);
	//	pi_id.OutF = pi_id.OutF*LPF_HFI_B + pi_id.Out*LPF_HFI_A;

	// Iq轴闭环
	pi_iq.Ref = pi_spd.Out;
	pi_iq.Fbk = Iq;
	PiController((M_PI_Control)&pi_iq);
	//	pi_iq.OutF = pi_iq.OutF*LPF_I_RUN_B + pi_iq.Out*LPF_I_RUN_A;

	// d轴注入的高频方波电压
	if (HfiUinFlag)
		HFI.Uin = HFI_UIN_OFFSET;
	else
		HFI.Uin = -HFI_UIN_OFFSET;

	//	IPARK_SMO_PVdq.Theta = SensorLess.theta;
	//	IPARK_SMO_PVdq.Ds = pi_id.Out;
	//	IPARK_SMO_PVdq.Qs = pi_iq.Out;
	//	IPARK_Cale((M_IPARK)&IPARK_SMO_PVdq);

	// IPARK计算
	Ipark.Theta = SensorLess.theta;
	Ipark.Qs = pi_iq.Out;
	Ipark.Ds = pi_id.Out + HFI.Uin;
	IPARK_Cale((M_IPARK)&Ipark);
}
