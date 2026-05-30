#include "motor_ctrl.h"

PI_Control pi_spd = PI_Control_DEFAULTS;
PI_Control pi_id = PI_Control_DEFAULTS;
PI_Control pi_iq = PI_Control_DEFAULTS;
PI_Control pi_pos = PI_Control_DEFAULTS;

float Limit_Sat(float Uint, float U_max, float U_min) // 限制赋值函数
{
	float Uout;

	if (Uint <= U_min)
		Uout = U_min;
	else if (Uint >= U_max)
		Uout = U_max;
	else
		Uout = Uint;
	return Uout;
}

// 增量式pid
void PiController(M_PI_Control pv)
{
	pv->err = pv->Ref - pv->Fbk;
	// 积分累加
	pv->integrator += pv->Ki * pv->err * TS;
	// 限制积分值
	if (pv->integrator > pv->Umax)
		pv->integrator = pv->Umax;
	if (pv->integrator < pv->Umin)
		pv->integrator = pv->Umin;
	// 输出 = P + I
	float u = pv->Kp * pv->err + pv->integrator;
	// 输出限幅
	pv->Out = Limit_Sat(u, pv->Umax, pv->Umin);
}

void PiParaInit(void)
{
	pi_spd.Kp = Para.SpdLessKp;
	pi_spd.Ki =Para.SpdLessKi;		   // 0.0001*10 / 0.2   T*SpeedLoopPrescaler/0.2
	pi_spd.Umax = PI_MAX_SPD;  //  电流 10
	pi_spd.Umin = -PI_MAX_SPD; //-10

	pi_id.Kp = 0.3f;  //
	pi_id.Ki = 1000.0f; //
	pi_id.Umax = PI_MAX_Ud;
	pi_id.Umin = -PI_MAX_Ud;

	pi_iq.Kp = 0.3f;
	pi_iq.Ki = 1000.0f;
	pi_iq.Umax = PI_MAX_Uq;
	pi_iq.Umin = -PI_MAX_Uq; //-PI_MAX_Uq

	pi_pos.Kp = 0.02f;
	pi_pos.Ki = 0.0f;
	pi_pos.Umax = SPD_REF_MAX;
	pi_pos.Umin = -SPD_REF_MAX; //-PI_MAX_Uq
}
