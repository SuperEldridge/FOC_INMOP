#ifndef FLUX_DRV_H
#define FLUX_DRV_H

#include "main.h"
#include <math.h>
#include <string.h>

typedef struct FLUX_DRV
{
    /* 电机参数（真实单位） */
    float Rs;      /* 电阻 Ω */
    float Ld;      /* d轴电感 H */
    float Lq;      /* q轴电感 H */
    float poles;   /* 极对数 */

    /* 采样周期（单位秒） */
    float Ts;

    /* 观测器参数 */
    float Ke;           /* 漏磁衰减系数 1/s */
    float psi_limit;    /* 磁链积分限幅（Wb） */

    /* 状态变量（真实物理量） */
    float Psia;     /* α轴定子磁链 (Wb) */
    float Psib;     /* β轴定子磁链 (Wb) */
    float Pra;      /* α轴转子磁链 (Wb) */
    float Prb;      /* β轴转子磁链 (Wb) */

    /* 历史导数，用于梯形积分 */
    float dPsia_prev;
    float dPsib_prev;

    /* 输出 */
    float theta_e;      /* 电角度 rad */
    float theta_prev;

    float w_e_rpm;      /* 电角速度 rpm */
    float w_m_rpm;      /* 机械角速度 rpm */

    /* 速度滤波参数 */
    float speed_alpha;  
    
    float pll_kp;    // PLL 比例增益
    float pll_ki;    // PLL 积分增益
    float pll_int;   // PLL 积分项
    float w_e_rad;   // 估计的电角速度 (rad/s)
    
} FLUX_DRV;

extern FLUX_DRV flux;

void FluxInit(FLUX_DRV* f);
void FluxReset(FLUX_DRV* f);
void FluxSync(FLUX_DRV* f, float theta, float omega_rad);
void FluxCale(FLUX_DRV* f,
              float Ialpha, float Ibeta,
              float Valpha, float Vbeta);
void FluxObserver(float Id, float Iq);

#endif
