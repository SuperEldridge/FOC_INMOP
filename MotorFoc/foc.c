#include "motor_ctrl.h"

SVPWM Svpwm = SVPWM_DEFAULTS;
/*
**Clarkb变换
**Ia+Ib+Ic=0
**等幅值变换
**Alpha=Iu
**Beta=sqrt(3)/3*Iu+x*sqrt(3)/3*Iv
*/

void ClarkeCale(M_CLARKE pv)
{
    pv->Alpha = pv->Iu;
    pv->Beta = (pv->Iu * 0.57735027f) + (pv->Iv * 1.1547004f);
}

/*
**Park变换
*/
void ParkCale(M_PARK pv)
{
    pv->Ds = arm_cos_f32(pv->Theta) * pv->Alpha + arm_sin_f32(pv->Theta) * pv->Beta;
    pv->Qs = arm_cos_f32(pv->Theta) * pv->Beta - arm_sin_f32(pv->Theta) * pv->Alpha;
}

/*
**IPark变换
*/
void IPARK_Cale(M_IPARK pv)
{
    pv->Alpha = arm_cos_f32(pv->Theta) * pv->Ds - arm_sin_f32(pv->Theta) * pv->Qs;
    pv->Beta = arm_sin_f32(pv->Theta) * pv->Ds + arm_cos_f32(pv->Theta) * pv->Qs;
}

/*
**SVPWM计算
// SVPWM是7段式矢量调试，扇区与输出电压占空比经过简化后为对称输出，1和4，2和5，3和6对称
// 文档给出是按照7段式SVPWM一点点按照矢量调制原理推到，中间推到有Uabc的三相电压
// T1和T1作用时间，Txyz时间最终等效Tabc三相输入占空比,
// Udc/√3=1，  单位1
// 调制利用率等于(1-0.04)=0.96=96%
// Ualpha和Ubeta是和uq ud等幅值变换
*/

void SvpwmCal(M_SVPWM pv)
{
    pv->u1 = pv->Ubeta; // 相当于二相静止坐标--到三相静止变换出Uabc
    pv->u2 = pv->Ubeta * 0.5f + pv->Ualpha * 0.8660254f;
    pv->u3 = pv->u2 - pv->u1;

    // 根据三相电压符号计算矢量扇区
    pv->VecSector = 3;
    pv->VecSector = (pv->u2 > 0) ? (pv->VecSector - 1) : pv->VecSector;
    pv->VecSector = (pv->u3 > 0) ? (pv->VecSector - 1) : pv->VecSector;
    pv->VecSector = (pv->u1 < 0) ? (7 - pv->VecSector) : pv->VecSector;

    // 根据矢量扇区计算矢量占空比Tabc
    if ((pv->VecSector == 1) || (pv->VecSector == 4))
    {
        pv->Ta = pv->u2;
        pv->Tb = pv->u1 - pv->u3;
        pv->Tc = -pv->u2;
    }
    else if ((pv->VecSector == 2) || (pv->VecSector == 5))
    {
        pv->Ta = pv->u3 + pv->u2;
        pv->Tb = pv->u1;
        pv->Tc = -pv->u1;
    }
    else if ((pv->VecSector == 3) || (pv->VecSector == 6))
    {
        pv->Ta = pv->u3;
        pv->Tb = -pv->u3;
        pv->Tc = -(pv->u1 + pv->u2);
    }
    else // 异常状态下的判断出的扇区 0---7或者其他就执行0电压矢量
    {
        pv->Ta = 0;
        pv->Tb = 0;
        pv->Tc = 0;
    }

    // Tabc是带正负的浮点型数据，电压， pv->Tabc/SVPWM_KM是标幺值计算，将Tabc计算成-1 —— +1
    // 将占空比调节为-1 —— +1， 再将PWM的半周期占空比值提出 （-1 —— +1 ）X 50% + 50% = 0 —— 100%
    TIM1->CCR1 = (uint16_t)((pv->Ta * SVPWM_KM_BACKW * TIM1_PERIOD_HALF) + TIM1_PERIOD_HALF);
    TIM1->CCR2 = (uint16_t)((pv->Tb * SVPWM_KM_BACKW * TIM1_PERIOD_HALF) + TIM1_PERIOD_HALF);
    TIM1->CCR3 = (uint16_t)((pv->Tc * SVPWM_KM_BACKW * TIM1_PERIOD_HALF) + TIM1_PERIOD_HALF);
}
