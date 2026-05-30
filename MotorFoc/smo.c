#include "motor_ctrl.h"

Angle_SMO AngleSmoPara = Angle_SMO_DEFAULTS;

float SMO_Angle1; // 角度补偿后的角度
float SMO_H;

static float Smo_Dir = 1.0f;
// 滑膜观测器角度计算
void SmoAngleCale(p_Angle_SMO pv)
{
	//	pv->Ialpha = CLARKE_ICurr.Alpha;
	//	pv->Ibeta = CLARKE_ICurr.Beta;
	//
	//	pv->Ualpha = SVPWM_dq.Ualpha;
	//	pv->Ubeta = SVPWM_dq.Ubeta;
	// 观测电流
	pv->Est_Ialpha += TS * (-Motor.Rs / Motor.Ls * pv->Est_Ialpha + 1 / Motor.Ls * (pv->Ualpha - pv->Zalpha));
	pv->Est_Ibeta += TS * (-Motor.Rs / Motor.Ls * pv->Est_Ibeta + 1 / Motor.Ls * (pv->Ubeta - pv->Zbeta));
	// 反电动势，Zalpha与Zbeta是开关信号，需要经过滤波才能得到反电动势的等效值
	//	pv->Zalpha = SMO_KSLIDE*Sign(pv->Est_Ialpha - pv->Ialpha);
	//	pv->Zbeta = SMO_KSLIDE*Sign(pv->Est_Ibeta - pv->Ibeta);

	pv->Ialpha_Err = pv->Est_Ialpha - pv->Ialpha;
	pv->Ibeta_Err = pv->Est_Ibeta - pv->Ibeta;

	pv->Zalpha = SMO_KSLIDE * Sat(pv->Ialpha_Err, 0.5f);
	pv->Zbeta = SMO_KSLIDE * Sat(pv->Ibeta_Err, 0.5f);
	// 低通滤波
	pv->Ealpha = pv->Ealpha * LPF_SMO_B + pv->Zalpha * LPF_SMO_A;
	pv->Ebeta = pv->Ebeta * LPF_SMO_B + pv->Zbeta * LPF_SMO_A;

	// 测试滑膜增益
	//	SMO_H = my_max(-Motor.Rs*my_abs(pv->Ialpha_Err) +pv->Ealpha*Sign(pv->Ialpha_Err), -Motor.Rs*my_abs(pv->Ibeta_Err) +pv->Ebeta*Sign(pv->Ibeta_Err));

	//	pv->atan_theta =my_atan2(pv->Ebeta, -pv->Ealpha);						//反正切法获得角度
	//	SMO_theta_compersation = my_atan(PllHfiPara.Omega/(-wc));
	//	SMO_Angle1 = pv->atan_theta - SMO_theta_compersation;
	//	if(SMO_Angle1 >= PIX2)
	//		SMO_Angle1 -=PIX2;
	//	else if(SMO_Angle1 < 0)
	//		SMO_Angle1 += PIX2;
	if (SensorLess.Speed_Ref >= 0)
	{
		// 指令要求正转：只有当电机实际速度大于 -50 RPM（即基本停止反转）时，才切换为正向算法
		if (SensorLess.Speed_Fbk >= -50.0f)
			Smo_Dir = 1.0f;
	}
	else
	{
		// 指令要求反转：只有当电机实际速度小于 50 RPM（即基本停止正转）时，才切换为反向算法
		if (SensorLess.Speed_Fbk <= 50.0f)
			Smo_Dir = -1.0f;
	}

	// 根据方向标志计算 PLL
	if (Smo_Dir == 1.0f)
		PllCale((p_PLL)&PllSmoPara, pv->Ealpha, pv->Ebeta);
	else
		PllCale((p_PLL)&PllSmoPara, -pv->Ealpha, -pv->Ebeta);
}

void SMO_Init(void)
{
	AngleSmoPara.Fsmopos = exp((-Motor.Rs / Motor.Ls) * TS);
	AngleSmoPara.Gsmopos = (Motor.Vb / Motor.Ib) * (1 / Motor.Rs) * (1 - AngleSmoPara.Fsmopos);
	AngleSmoPara.E0 = 0.5; // 标幺值1的一半
}

// void SmoAngleCale(p_Angle_SMO pv)
//{
//	float IalphaError_Limit=0,IbetaError_Limit=0;
//	/*	Sliding mode current observer	*/
//	pv->Est_Ialpha = pv->Est_Ialpha*pv->Fsmopos + (pv->Ualpha - pv->Ealpha - pv->Zalpha)*pv->Gsmopos;
//	pv->Est_Ibeta = pv->Est_Ibeta*pv->Fsmopos + (pv->Ubeta - pv->Ebeta - pv->Zbeta)*pv->Gsmopos;
//
//	/*	Current errors	*/
//   pv->Ialpha_Err = pv->Est_Ialpha - pv->Ialpha;
//	pv->Ibeta_Err = pv->Est_Ibeta - pv->Ibeta;
//
//	/*  Sliding control calculator	*/
//	/* pV->Zalpha=pV->IalphaError*pV->Kslide/pV->E0) where E0=0.5 here*/
//	//限制赋值函数
//	IalphaError_Limit = Limit_Sat(pv->Ialpha_Err, pv->E0, -pv->E0);
//	IbetaError_Limit = Limit_Sat(pv->Ibeta_Err, pv->E0, -pv->E0);
//
//	pv->Zalpha = IalphaError_Limit*SMO_KSLIDE;
//	pv->Zbeta = IbetaError_Limit*SMO_KSLIDE;
//
//	/*	Sliding control filter -> back EMF calculator	*/
//	pv->Ealpha = pv->Ealpha + (pv->Zalpha - pv->Ealpha)*SMO_KSLF;
//	pv->Ebeta = pv->Ebeta + (pv->Zbeta - pv->Ebeta)*SMO_KSLF;
//
////	pv->atan_theta =my_atan2(pv->Ebeta, -pv->Ealpha);
//	PllCale((p_PLL)&PllSmoPara, pv->Ealpha, pv->Ebeta);
//}

// SMO速度闭环控制
void SmoObserver(float Id, float Iq)
{
	if (Para.SpeedCalTime == SPD_CAL_PERIOD)
	{
		pi_spd.Ref = SensorLess.Speed_Ref;
		pi_spd.Fbk = SensorLess.Speed_Fbk;
		PiController((M_PI_Control)&pi_spd);								// 速度环
		pi_spd.OutF = pi_spd.OutF * LPF_I_RUN_B + pi_spd.Out * LPF_I_RUN_A; // 环路滤波后输出
	}

	pi_id.Ref = 0; // id=0，d轴电流闭环，力矩大
	pi_id.Fbk = Id;
	PiController((M_PI_Control)&pi_id); // Id轴闭环
	pi_id.OutF = pi_id.OutF * LPF_I_RUN_B + pi_id.Out * LPF_I_RUN_A;

	pi_iq.Ref = pi_spd.OutF; // Iq的设定和电机的额定转矩有关，IF启动一般Iq选择比额定转矩下的Iq略大
	pi_iq.Fbk = Iq;
	PiController((M_PI_Control)&pi_iq); // Iq轴闭环
	pi_iq.OutF = pi_iq.OutF * LPF_I_RUN_B + pi_iq.Out * LPF_I_RUN_A;

	Ipark.Theta = SensorLess.theta;
	Ipark.Ds = pi_id.OutF;
	Ipark.Qs = pi_iq.OutF;
	IPARK_Cale((M_IPARK)&Ipark); // IPARK函数计算
}

// 滑膜观测器角度计算
void SmoThetaCale(float Ialpha, float Ibeta, float Ualpha, float Ubeta)
{
	AngleSmoPara.Ialpha = Ialpha;
	AngleSmoPara.Ibeta = Ibeta;
	AngleSmoPara.Ualpha = Ualpha;
	AngleSmoPara.Ubeta = Ubeta;
	SmoAngleCale((p_Angle_SMO)&AngleSmoPara);
}
