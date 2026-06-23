#include "motor_ctrl.h"

// 电机归零
void PositionToZero(void)
{
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE |
                 TIM_CCER_CC2E | TIM_CCER_CC2NE |
                 TIM_CCER_CC3E | TIM_CCER_CC3NE);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_Delay(100);

    Svpwm.Ualpha = 1; // 调零电压
    Svpwm.Ubeta = 0;
    SvpwmCal((M_SVPWM)&Svpwm); // 将Alpha和Beta电压带入计算SVPWM的占空比
    HAL_Delay(1000);
    TIM3->CNT = 0;     // 清除编码器当前计数值
    Motor.E_theta = 0; // 电角度清零
    HAL_Delay(500);

    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                  TIM_CCER_CC2E | TIM_CCER_CC2NE |
                  TIM_CCER_CC3E | TIM_CCER_CC3NE);
    HAL_Delay(500);
}

// 二次定位调零
void PostionToZeroDouble(void)
{
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE |
                 TIM_CCER_CC2E | TIM_CCER_CC2NE |
                 TIM_CCER_CC3E | TIM_CCER_CC3NE);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    HAL_Delay(100);

    Svpwm.Ualpha = 0.0f; // 调零电压
    Svpwm.Ubeta = 0.5f;
    SvpwmCal((M_SVPWM)&Svpwm); // 将Alpha和Beta电压带入计算SVPWM的占空比
    HAL_Delay(1000);

    Svpwm.Ualpha = 0.5f; // 调零电压
    Svpwm.Ubeta = 0.0f;
    SvpwmCal((M_SVPWM)&Svpwm); // 将Alpha和Beta电压带入计算SVPWM的占空比
    HAL_Delay(1000);

    TIM3->CNT = 0; // 清除编码器当前计数值
    Motor.E_theta = 0;

    HAL_Delay(1000);

    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                  TIM_CCER_CC2E | TIM_CCER_CC2NE |
                  TIM_CCER_CC3E | TIM_CCER_CC3NE);
    HAL_Delay(1000);

    TIM1->CCER |= TIM_CCER_CC4E;

    MotorStop();
}

// 有感,电流闭环
void CurCloseloop(float IqRef, float Theta)
{
    pi_id.Ref = 0.0f; // id=0，d轴电流闭环，力矩大
    pi_id.Fbk = Park.Ds;
    PiController((M_PI_Control)&pi_id); // Id轴闭环

    pi_iq.Ref = IqRef;
    pi_iq.Fbk = Park.Qs;
    PiController((M_PI_Control)&pi_iq); // Iq轴闭环

    Ipark.Theta = Theta;
    Ipark.Ds = pi_id.Out;
    Ipark.Qs = pi_iq.Out;
    IPARK_Cale((M_IPARK)&Ipark); // IPARK函数计算
}

// UseRamp: 1-使用梯度计算（用于纯速度模式）, 0-直接给定（用于位置模式）
void SpeedCloseloop(int16_t SpeedRef, uint8_t UseRamp)
{
    float final_spd_ref = 0.0f;

    if (UseRamp)
    {
		SpdRefGXieLv.XieLv_Grad = SPD_REF_OD;	 // 定义速度的梯度值
		SpdRefGXieLv.Grad_Timer = SPD_REF_TIMER; // 定义速度的梯度时间
        SpdRefGXieLv.XieLv_X = SpeedRef;
        SpdRefGXieLv.Timer_Count++;
        if (my_abs(pi_spd.Ref - pi_spd.Fbk) >= 1000)
            SpdRefGXieLv.Timer_Count--;

        if (SpdRefGXieLv.Timer_Count > SpdRefGXieLv.Grad_Timer)
        {
            SpdRefGXieLv.Timer_Count = 0;
            GradXieLv((p_GXieLv)&SpdRefGXieLv);
        }
        final_spd_ref = Limit_Sat(SpdRefGXieLv.XieLv_Y, SPD_REF_MAX, -SPD_REF_MAX);
    }
    else
    {
        final_spd_ref = (float)SpeedRef;
        final_spd_ref = Limit_Sat(final_spd_ref, 100, -100);//位置环速度限制
    }

    float iq_ref = 0.0f;                    // 最终送入电流环的 q 轴电流给定
    float active_damp_iq = 0.0f;            // 有功阻尼产生的 q 轴电流修正量
#if (SPEED_CTRL_USE_SMC == 1)
    static float smc_iq_ref_u = 0.0f;       // 保存图中控制量 u，实际为 SMC 输出的 q 轴电流给定
    SMC_SPEED_INPUT smc_input;              // SMC 输入量结构体，用于隔离速度环和 SMC 模块
#endif
    // 速度环计算频率控制
    if (++Para.SpeedCalTime >= SPEED_CNT)
    {
        Para.SpeedCalTime = 0;

        pi_spd.Ref = final_spd_ref * Motor.P;
        pi_spd.Fbk = Motor.speed_E_rpm;

#if (SPEED_CTRL_USE_SMC == 1)
        smc_input.omega_ref_elec_rpm = pi_spd.Ref;   // 将原速度环电角速度给定传入 SMC
        smc_input.omega_fbk_elec_rpm = pi_spd.Fbk;   // 将原速度环电角速度反馈传入 SMC
        smc_input.iq_fbk_u = Park.Qs;                // 将当前 q 轴电流反馈作为图中 u 的反馈量传入观测器
        smc_iq_ref_u = SmcSpeed_Update(&SmcSpeed, &smc_input); // 调用 SMC 模块计算图中的控制量 u
#else
        PiController((M_PI_Control)&pi_spd); // 速度PI计算

        pi_spd.OutF = pi_spd.OutF * LPF_I_RUN_B + pi_spd.Out * LPF_I_RUN_A;
#endif
    }

    #if SPD_ACTIVE_DAMP_EN
        /* 有功阻尼项：
        * 1. 当前速度环内部使用的是电角速度 rpm，因此这里直接基于
    Motor.speed_E_rpm 构造阻尼项；
        * 2. 阻尼项始终与当前转速方向相反，相当于在速度升高时主动减小 q
    轴给定电流；
        * 3. 通过限幅防止高速区阻尼项过大，导致输出电流被过度压制。
        */
    active_damp_iq = SPD_ACTIVE_DAMP_KE * Motor.speed_E_rpm;
    active_damp_iq = Limit_Sat(active_damp_iq, SPD_ACTIVE_DAMP_MAX,-SPD_ACTIVE_DAMP_MAX);
    #endif
    
    /* 速度环最终输出 = PI 输出 - 有功阻尼项 */
#if (SPEED_CTRL_USE_SMC == 1)
    iq_ref = smc_iq_ref_u - active_damp_iq;    // SMC 输出的 u 叠加有功阻尼修正后形成电流环给定
    iq_ref = Limit_Sat(iq_ref, SMC_IQ_LIMIT, -SMC_IQ_LIMIT); // 按 SMC 电流限幅保护 q 轴给定
#else
    iq_ref = pi_spd.OutF - active_damp_iq;     // PI 输出叠加有功阻尼修正后形成电流环给定
    iq_ref = Limit_Sat(iq_ref, PI_MAX_SPD, -PI_MAX_SPD); // 按原 PI 电流限幅保护 q 轴给定
#endif
    
    CurCloseloop(iq_ref, Motor.E_theta);
}

// 有感，位置环
#define ENC_MIN 1
#define ENC_MAX 3999
#define ENC_RANGE (ENC_MAX - ENC_MIN + 1) // 3999
#define POS_MAX_SPEED 3000                // 位置环输出的最大转速限制 (RPM)
#define POS_I_LIMIT 500                   // 位置环积分限幅，防止过冲

void PosCloseloop(void)
{
    static uint32_t PosLoopDivCnt = 0;
    float SpeedRef = 0.0f;

    Pos.P_Fdb = TIM3->CNT;

    if (++PosLoopDivCnt >= 4)
    {
        PosLoopDivCnt = 0;

        // 目标位置映射 (Para.PosRef: 0..4095 -> mapped: 3999..1)
        uint32_t t = ((uint32_t)Para.PosRef * (ENC_RANGE - 1)) >> 12;
        uint16_t mapped = (uint16_t)(ENC_MAX - t);
        Pos.P_Ref = (int32_t)mapped;

        int32_t err = (int32_t)mapped - (int32_t)Pos.P_Fdb;
        int32_t half = ENC_RANGE / 2;
        if (err > half)
            err -= ENC_RANGE;
        else if (err < -half)
            err += ENC_RANGE;

        // 比例项
        float up = Pos.Kp * (float)err;

        // 积分项
        /*
        Pos.P_Integral += Pos.Ki * (float)err;
        Pos.P_Integral = Limit_Sat(Pos.P_Integral, POS_I_LIMIT, -POS_I_LIMIT); // 抗饱和
        SpeedRef = up + Pos.P_Integral;
        */

        SpeedRef = up; // 目前仅使用 P 控制

        // 输出限幅：防止位置偏差过大导致给出的速度参考值溢出或过快
        Pos.P_Out = (int16_t)Limit_Sat(SpeedRef, POS_MAX_SPEED, -POS_MAX_SPEED);
    }

    SpeedCloseloop(Pos.P_Out, 0);
}
