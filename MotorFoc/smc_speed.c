#include "smc_speed.h"                         // 引入 SMC 速度控制器接口
#include "pid.h"                               // 引入 Limit_Sat 限幅函数接口
#include <math.h>                              // 引入 sqrtf，用于超螺旋观测器计算

SMC_SPEED_CTRL SmcSpeed;                        // 定义全局 SMC 速度控制器实例

// 计算浮点数绝对值，避免依赖额外通用库函数
static float SmcAbs(float value)
{
    return (value >= 0.0f) ? value : -value;     // 大于等于零时直接返回，小于零时取反后返回
}

// 计算浮点符号，边界层退化时使用
static float SmcSign(float value)
{
    if (value > 0.0f)                            // 大于零时返回正号
        return 1.0f;                             // 返回 1 表示正方向
    if (value < 0.0f)                            // 小于零时返回负号
        return -1.0f;                            // 返回 -1 表示负方向
    return 0.0f;                                 // 等于零时返回零
}

// 饱和函数，用边界层替代符号函数以减小抖振
static float SmcSat(float value, float boundary_layer)
{
    if (boundary_layer <= 0.0f)                  // 边界层无效时退化为符号函数
        return SmcSign(value);                   // 返回符号函数结果
    return Limit_Sat(value / boundary_layer, 1.0f, -1.0f); // 归一化并限幅到 -1 到 1
}

// 电角速度 rpm 转换为机械角速度 rad/s
static float SmcElecRpmToMechRad(float omega_elec_rpm, float pole_pairs)
{
    if (pole_pairs <= 0.0f)                      // 极对数异常时直接返回零
        return 0.0f;                             // 返回零速度，保证后续计算不除零
    return omega_elec_rpm * PIX2 / (60.0f * pole_pairs); // 电角速度除以极对数转换为机械 rad/s
}

// 更新误差状态，后续更换状态定义时优先修改本函数
static void SmcSpeed_UpdateErrorState(SMC_SPEED_CTRL *ctrl, float omega_ref_rad, float omega_fbk_rad)
{
    float speed_err = omega_ref_rad - omega_fbk_rad; // 本拍机械角速度误差 e=目标-实际

    ctrl->out.x2_speed_err = speed_err;          // x2=误差本身 e，作为积分滑模面的比例部分

    if (!ctrl->diff_ready)                       // 首次进入时只锁存误差，积分从 0 起算，避免初值冲击
    {
        ctrl->out.x1_speed_err = 0.0f;           // x1=误差积分∫e dt，启动清零防止历史残留
        ctrl->out.x1_speed_err_prev = 0.0f;      // 记录积分历史值，供箍位抗饱和回退使用
        ctrl->diff_ready = 1;                    // 标记误差状态已完成初始化
    }
    else
    {
        ctrl->out.x1_speed_err_prev = ctrl->out.x1_speed_err; // 先保存积分上一拍值，便于触限时回退冻结
        ctrl->out.x1_speed_err += speed_err * ctrl->cfg.ts;   // x1 积分累加∫e dt(误差反向时自然泄放)
        ctrl->out.x1_speed_err = Limit_Sat(ctrl->out.x1_speed_err, // 仿 PI 对积分项直接限幅
                                           ctrl->cfg.integral_limit, // 积分正向上限
                                           -ctrl->cfg.integral_limit);
    }
}

// 计算滑模面，后续替换终端滑模面或模糊滑模面时优先修改本函数
static float SmcSpeed_CalcSlidingSurface(SMC_SPEED_CTRL *ctrl)
{
    /* 参考速度恒定时，x2 近似等于 -omega_dot；参考速度经过斜坡时，x2 表示误差变化率。 */
    return ctrl->cfg.slide_c * ctrl->out.x1_speed_err + ctrl->out.x2_speed_err; // s=c*积分+误差 // s=c*x1+x2
}

// 指数趋近律计算，对应 q*s+epsilon*sat(s)
static float SmcExpReachingLawUpdate(SMC_SPEED_CTRL *ctrl, float slide_s)
{
    float sat_s = SmcSat(slide_s, ctrl->cfg.reaching.boundary_layer); // 对滑模面做边界层饱和处理

    return ctrl->cfg.reaching.q * slide_s + ctrl->cfg.reaching.epsilon * sat_s; // 计算 q*s+epsilon*sat(s)
}

// 趋近律统一分发入口，后续增加模糊自适应趋近律时只扩展本函数
static float SmcSpeed_CalcReachingLaw(SMC_SPEED_CTRL *ctrl, float slide_s)
{
    switch (ctrl->cfg.reaching.mode)             // 根据配置选择趋近律算法
    {
    case SMC_REACH_EXP:                          // 当前默认指数趋近律
    default:                                     // 未识别模式退回默认算法，保证输出可控
        return SmcExpReachingLawUpdate(ctrl, slide_s); // 返回指数趋近律结果
    }
}

// 超螺旋扰动观测器更新，后续替换观测器时只替换本函数或分发入口
static void SmcSuperTwistingObserverUpdate(SMC_SPEED_CTRL *ctrl, float omega_fbk_rad, float iq_fbk_u)
{
    float observer_err = 0.0f;                   // 速度观测误差
    float sat_observer_err = 0.0f;               // 经过边界层处理后的观测误差
    float sto_v1 = 0.0f;                         // 超螺旋一阶注入项
    float sto_v2 = 0.0f;                         // 超螺旋扰动估计变化率
    float omega_hat_dot = 0.0f;                  // 速度观测值微分

    if (!ctrl->observer_ready)                   // 首次运行时观测器还没有初值
    {
        ctrl->out.omega_hat = omega_fbk_rad;     // 用当前反馈速度作为观测速度初值
        ctrl->out.disturbance_hat = 0.0f;        // 扰动估计从零开始，避免初值突变
        ctrl->observer_ready = 1;                // 标记观测器已经完成初值同步
        return;                                  // 首次只同步状态，不注入补偿量
    }

    observer_err = ctrl->out.omega_hat - omega_fbk_rad; // 计算观测速度与反馈速度之间的误差
    sat_observer_err = SmcSat(observer_err, ctrl->cfg.reaching.boundary_layer); // 对观测误差做边界层饱和处理
    sto_v1 = -ctrl->cfg.observer.k1 * sqrtf(SmcAbs(observer_err)) * sat_observer_err; // 计算超螺旋一阶注入项
    sto_v2 = -ctrl->cfg.observer.k2 * sat_observer_err; // 计算扰动估计变化率

    omega_hat_dot = ctrl->cfg.input_gain_d * iq_fbk_u - // D*u，表示 q 轴电流产生的机械角加速度
                    (ctrl->cfg.damping / ctrl->cfg.inertia) * ctrl->out.omega_hat + // 模型阻尼产生的角加速度
                    ctrl->out.disturbance_hat + sto_v1; // 正向叠加扰动估计和超螺旋注入项

    ctrl->out.omega_hat += omega_hat_dot * ctrl->cfg.ts; // 离散积分得到速度观测值
    ctrl->out.disturbance_hat += sto_v2 * ctrl->cfg.ts; // 离散积分得到扰动估计值
    ctrl->out.disturbance_hat = Limit_Sat(ctrl->out.disturbance_hat, // 对扰动估计进行限幅保护
                                          ctrl->cfg.observer.disturbance_limit, // 扰动估计正向限幅
                                          -ctrl->cfg.observer.disturbance_limit); // 扰动估计反向限幅
}

// 观测器统一更新入口，用开关保证可拆卸验证
static void SmcSpeed_UpdateObserver(SMC_SPEED_CTRL *ctrl, float omega_fbk_rad, float iq_fbk_u)
{
    if (!ctrl->cfg.observer.enable)              // 观测器关闭时退化为纯 SMC
    {
        ctrl->out.omega_hat = omega_fbk_rad;     // 同步观测速度，避免后续重新开启时冲击
        ctrl->out.disturbance_hat = 0.0f;        // 清零扰动估计，确保无观测器对比不受旧值影响
        ctrl->observer_ready = 0;                // 重新开启时重新进行初值同步
        return;                                  // 不更新观测器
    }

    switch (ctrl->cfg.observer.mode)             // 根据配置选择观测器算法
    {
    case SMC_OBSERVER_SUPER_TWISTING:            // 当前默认超螺旋扰动观测器
    default:                                     // 未识别模式退回默认观测器
        SmcSuperTwistingObserverUpdate(ctrl, omega_fbk_rad, iq_fbk_u); // 更新超螺旋观测器
        break;
    }
}

// 获取扰动补偿项，关闭观测器时返回零以便验证纯滑模控制
static float SmcSpeed_GetDisturbanceComp(SMC_SPEED_CTRL *ctrl)
{
    if (!ctrl->cfg.observer.enable)              // 观测器未使能时不做扰动补偿
        return 0.0f;                             // 返回零补偿
    return ctrl->out.disturbance_hat;            // 返回观测到的总等效扰动
}

// 主流等效 SMC 控制律，扰动在对象中正向进入，控制器中反向补偿
static float SmcSpeed_CalcEquivalentControl(SMC_SPEED_CTRL *ctrl, float omega_fbk_rad, float disturbance_comp)
{
    return (ctrl->out.reach_law +                // 趋近律项决定滑模面收敛速度
            (ctrl->cfg.damping / ctrl->cfg.inertia) * omega_fbk_rad - // 已知阻尼模型补偿 omega_fbk_rad
            disturbance_comp) / ctrl->cfg.input_gain_d; // 通过 1/D 转换为 q 轴电流给定
}

// 控制律统一分发入口，后续接入积分型或自适应控制律时只扩展本函数
static float SmcSpeed_CalcControlLaw(SMC_SPEED_CTRL *ctrl, float omega_fbk_rad)
{
    float disturbance_comp = SmcSpeed_GetDisturbanceComp(ctrl); // 按观测器开关取得扰动补偿项

    switch (ctrl->cfg.control_mode)              // 根据配置选择控制律
    {
    case SMC_CONTROL_EQUIVALENT:                 // 主流等效控制 + 趋近律 + 扰动补偿
    default:                                     // 未识别模式退回默认等效控制
        return SmcSpeed_CalcEquivalentControl(ctrl, omega_fbk_rad, disturbance_comp); // 返回未限幅电流指令
    }
}

// 初始化 SMC 控制器配置及内部状态
void SmcSpeed_Init(SMC_SPEED_CTRL *ctrl)
{
    ctrl->cfg.ts = SPEED_TIME;                   // 设置速度环离散周期
    ctrl->cfg.pole_pairs = (float)MOTOR_P;       // 设置电机极对数
    ctrl->cfg.kt = Kt;                           // 设置转矩常数
    ctrl->cfg.inertia = J;                       // 设置转动惯量
    ctrl->cfg.damping = B;                       // 设置粘性阻尼系数
    ctrl->cfg.input_gain_d = Kt / J;             // 设置 D，q 轴电流到机械角加速度的增益
    ctrl->cfg.slide_c = SMC_SLIDE_C;             // 设置滑模面系数 c
    ctrl->cfg.iq_limit = SMC_IQ_LIMIT;           // 设置 SMC 输出限幅
    ctrl->cfg.integral_limit = SMC_INTEGRAL_LIMIT; // 设置积分项限幅，仿 PI 防 windup 残留
    ctrl->cfg.control_mode = (SMC_CONTROL_MODE)SMC_CONTROL_MODE_CFG; // 设置控制律模式

    ctrl->cfg.reaching.q = SMC_REACH_Q;          // 设置 q，对应速度误差线性趋近系数
    ctrl->cfg.reaching.epsilon = SMC_REACH_EPSILON; // 设置 epsilon，对应符号切换趋近系数
    ctrl->cfg.reaching.boundary_layer = SMC_BOUNDARY_LAYER; // 设置滑模边界层宽度
    ctrl->cfg.reaching.mode = SMC_REACH_EXP;     // 选择当前趋近律为指数趋近律

    ctrl->cfg.observer.k1 = SMC_STO_K1;          // 设置超螺旋观测器一阶增益
    ctrl->cfg.observer.k2 = SMC_STO_K2;          // 设置超螺旋观测器二阶增益
    ctrl->cfg.observer.disturbance_limit = SMC_DISTURBANCE_LIMIT; // 设置扰动估计限幅
    ctrl->cfg.observer.mode = SMC_OBSERVER_SUPER_TWISTING; // 选择当前观测器为超螺旋观测器
    ctrl->cfg.observer.enable = SMC_STO_ENABLE;  // 设置观测器使能，0 时退化为纯 SMC

    SmcSpeed_Reset(ctrl);                        // 初始化后清空控制器内部动态状态
}

// 清空 SMC 内部状态，用于停机、故障停机或重新启动前调用
void SmcSpeed_Reset(SMC_SPEED_CTRL *ctrl)
{
    ctrl->out.iq_ref_u = 0.0f;                   // 清空 q 轴电流给定
    ctrl->out.x1_speed_err = 0.0f;               // 清空速度误差 x1
    ctrl->out.x1_speed_err_prev = 0.0f;          // 清空 x1 上一拍状态
    ctrl->out.x2_speed_err = 0.0f;               // 清空误差变化率 x2
    ctrl->out.slide_s = 0.0f;                    // 清空滑模面 s
    ctrl->out.reach_law = 0.0f;                  // 清空趋近律输出
    ctrl->out.disturbance_hat = 0.0f;            // 清空扰动估计
    ctrl->out.omega_hat = 0.0f;                  // 清空速度观测值
    ctrl->observer_ready = 0;                    // 清空观测器初值同步标志
    ctrl->diff_ready = 0;                        // 清空 x1 微分初始化标志
}

// SMC 速度环更新函数，输入速度参考和反馈，输出 q 轴电流给定
float SmcSpeed_Update(SMC_SPEED_CTRL *ctrl, const SMC_SPEED_INPUT *input)
{
    float omega_ref_rad = SmcElecRpmToMechRad(input->omega_ref_elec_rpm, ctrl->cfg.pole_pairs); // 速度参考转为机械 rad/s
    float omega_fbk_rad = SmcElecRpmToMechRad(input->omega_fbk_elec_rpm, ctrl->cfg.pole_pairs); // 速度反馈转为机械 rad/s
    float iq_ref_u = 0.0f;                       // 未限幅 q 轴电流给定

    SmcSpeed_UpdateObserver(ctrl, omega_fbk_rad, input->iq_fbk_u); // 按开关更新扰动观测器
    SmcSpeed_UpdateErrorState(ctrl, omega_ref_rad, omega_fbk_rad); // 更新速度误差状态

    ctrl->out.slide_s = SmcSpeed_CalcSlidingSurface(ctrl); // 计算滑模面，便于后续替换滑模面
    ctrl->out.reach_law = SmcSpeed_CalcReachingLaw(ctrl, ctrl->out.slide_s); // 计算趋近律输出

    iq_ref_u = SmcSpeed_CalcControlLaw(ctrl, omega_fbk_rad); // 计算主流 SMC 控制律输出

    ctrl->out.iq_ref_u = Limit_Sat(iq_ref_u, ctrl->cfg.iq_limit, -ctrl->cfg.iq_limit); // 对 q 轴输出做限幅

    return ctrl->out.iq_ref_u;                   // 返回限幅后的 q 轴电流给定
}
