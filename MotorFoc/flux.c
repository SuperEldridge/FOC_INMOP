#include "motor_ctrl.h"
#include "arm_math.h" /* arm_sin_f32 / arm_cos_f32 */
#include <math.h>
#include <string.h>

#define TORPM_FACTOR (30.0f / PI) // rad/s 转 RPM 的系数

FLUX_DRV flux;

/* wrap [0,2π) */
static inline float wrap_0_2pi(float x)
{
    x = fmodf(x, 2.0f * PI);
    if (x < 0.0f)
        x += 2.0f * PI;
    return x;
}

static inline float sat(float x, float hi, float lo)
{
    if (x > hi)
        return hi;
    if (x < lo)
        return lo;
    return x;
}

/* ============================================================================
 * 初始化磁链参数
 * ============================================================================
 */
void FluxInit(FLUX_DRV *f)
{
    /* 电机参数 */
    f->Rs = MOTOR_Rs; /* 电阻 Ω */
    f->Ld = MOTOR_Ls; /* H */
    f->Lq = MOTOR_Ls; /* H */
    f->poles = MOTOR_P;

    /* 采样周期 50us */
    f->Ts = TS;

    /* 漏磁衰减系数 Ke，典型 5~100 rad/s */
    f->Ke = 2.0f * PI * FLUX_KE;

    /* 磁链限幅（Wb） */
    f->psi_limit = PSI_LIMIT;

    /* 速度一阶滤波器 */
    float cutoff = 20.0f; /* Hz */
    float tau = 2.0f * PI * cutoff;
    f->speed_alpha = (tau * f->Ts) / (1.0f + tau * f->Ts);

    /* 初始化 PLL 参数 */
    f->pll_kp = PLL_KP_DEF;
    f->pll_ki = PLL_KI_DEF;

    FluxReset(f);
}

/* 状态清零 */
void FluxReset(FLUX_DRV *f)
{
    f->Psia = f->Psib = 0.0f;
    f->Pra = f->Prb = 0.0f;

    f->dPsia_prev = 0.0f;
    f->dPsib_prev = 0.0f;

    f->theta_e = 0.0f;
    f->theta_prev = 0.0f;

    f->w_e_rpm = 0.0f;
    f->w_m_rpm = 0.0f;

    /* 清零 PLL 状态 */
    f->pll_int = 0.0f;
    f->w_e_rad = 0.0f;
}

void FluxSync(FLUX_DRV *f, float theta, float omega_rad)
{
    f->theta_e = wrap_0_2pi(theta);
    f->theta_prev = f->theta_e;
    f->w_e_rad = omega_rad;
    f->w_e_rpm = omega_rad * TORPM_FACTOR;
    f->w_m_rpm = f->w_e_rpm / f->poles;
    f->pll_int = omega_rad;
}

/* ============================================================================
 * 磁链观测器主计算
 * Iα/Iβ: A
 * Vα/Vβ: V
 * ============================================================================
 */
void FluxCale(FLUX_DRV *f,
              float Ialpha, float Ibeta,
              float Valpha, float Vbeta)
{
    /* ================================
     * A) 计算 dψs/dt = v - R*i - Ke*ψ
     * ================================ */
    float dPsia = (Valpha - f->Rs * Ialpha) - f->Ke * f->Psia;
    float dPsib = (Vbeta - f->Rs * Ibeta) - f->Ke * f->Psib;

    /* ================================
     * B) 梯形积分 ψ_new = ψ_old + Ts*(dψ + dψ_prev)/2
     * ================================ */
    float Psia_new = f->Psia + 0.5f * f->Ts * (dPsia + f->dPsia_prev);
    float Psib_new = f->Psib + 0.5f * f->Ts * (dPsib + f->dPsib_prev);

    /* 限幅（Wb） */
    Psia_new = sat(Psia_new, f->psi_limit, -f->psi_limit);
    Psib_new = sat(Psib_new, f->psi_limit, -f->psi_limit);

    f->Psia = Psia_new;
    f->Psib = Psib_new;

    f->dPsia_prev = dPsia;
    f->dPsib_prev = dPsib;

    /* ================================
     * C) Rotor flux: ψr = ψs - L * i
     * αβ 坐标中 Ld/Lq 当作等效使用
     * ================================ */
    f->Pra = f->Psia - f->Ld * Ialpha;
    f->Prb = f->Psib - f->Lq * Ibeta;

    /* ================================
     * D) 锁相环 (PLL) 角度获取
     * 原理：误差 e = ψr_β * cos(θ) - ψr_α * sin(θ)
     * ================================ */
    float sin_theta = arm_sin_f32(f->theta_e);
    float cos_theta = arm_cos_f32(f->theta_e);

    // 计算叉乘误差
    float pll_error = f->Prb * cos_theta - f->Pra * sin_theta;

    // 磁链幅值归一化（能解耦磁链幅值对 PLL 带宽的影响）
    float flux_mag = sqrtf(f->Pra * f->Pra + f->Prb * f->Prb);
    if (flux_mag > 0.0001f)
    {
        pll_error /= flux_mag;
    }
    else
    {
        pll_error = 0.0f;
    }

    // PI 控制器获取电角速度 (rad/s)
    f->pll_int += f->pll_ki * pll_error * f->Ts;

    f->w_e_rad = f->pll_kp * pll_error + f->pll_int;

    // 速度积分得到估算电角度
    f->theta_e += f->w_e_rad * f->Ts;
    f->theta_e = wrap_0_2pi(f->theta_e);

    /* ================================
     * E) 速度转换与滤波
     * ================================ */
    // 转化为电角速度
    float we_rpm = f->w_e_rad * TORPM_FACTOR;

    // 如果 PLL 参数调得较硬（带宽大），这里的一阶滤波仍然可以保留；
    // 如果 PLL 参数偏柔和，这段滤波可以直接去掉，直接赋值 f->w_e_rpm = we_rpm;
    f->w_e_rpm = f->speed_alpha * we_rpm + (1.0f - f->speed_alpha) * f->w_e_rpm;

    /* ================================
     * F) 机械角速度
     * ================================ */
    f->w_m_rpm = f->w_e_rpm / f->poles;

    f->theta_prev = f->theta_e;
}

// FLUX 速度闭环控制
void FluxObserver(float Id, float Iq)
{
    if (Para.SpeedCalTime == SPD_CAL_PERIOD)
    {
        pi_spd.Ref = SensorLess.Speed_Ref;
        pi_spd.Fbk = SensorLess.Speed_Fbk;
        PiController((M_PI_Control)&pi_spd);                                // 速度环
        pi_spd.OutF = pi_spd.OutF * LPF_I_RUN_B + pi_spd.Out * LPF_I_RUN_A; // 环路滤波后输出
    }

    pi_id.Ref = 0; // id=0，d轴电流闭环，力矩最大
    pi_id.Fbk = Id;
    PiController((M_PI_Control)&pi_id); // Id 轴闭环
    pi_id.OutF = pi_id.OutF * LPF_I_RUN_B + pi_id.Out * LPF_I_RUN_A;

    pi_iq.Ref = pi_spd.OutF; // Iq 的设定和电机的额定转矩有关，IF 启动一般 Iq 选择比额定转矩下的 Iq 略大
    pi_iq.Fbk = Iq;
    PiController((M_PI_Control)&pi_iq); // Iq 轴闭环
    pi_iq.OutF = pi_iq.OutF * LPF_I_RUN_B + pi_iq.Out * LPF_I_RUN_A;

    Ipark.Theta = SensorLess.theta;
    Ipark.Ds = pi_id.OutF;
    Ipark.Qs = pi_iq.OutF;
    IPARK_Cale((M_IPARK)&Ipark); // IPARK 函数计算
}
