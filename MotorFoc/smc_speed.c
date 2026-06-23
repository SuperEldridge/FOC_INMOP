#include "smc_speed.h"                         // 引入 SMC 速度控制器接口
#include "pid.h"                               // 引入 Limit_Sat 限幅函数声明

SMC_SPEED_CTRL SmcSpeed;                        // 定义全局 SMC 速度控制器实例

// 计算浮点绝对值，避免依赖额外库函数
static float SmcAbs(float value)
{
    return (value >= 0.0f) ? value : -value;     // 正数直接返回，负数取反后返回
}

// 计算浮点符号，供边界层退化时使用
static float SmcSign(float value)
{
    if (value > 0.0f)                            // 大于零时返回正号
        return 1.0f;                             // 正号用 1 表示
    if (value < 0.0f)                            // 小于零时返回负号
        return -1.0f;                            // 负号用 -1 表示
    return 0.0f;                                 // 等于零时返回零
}

// 饱和函数，用边界层替代纯符号函数以减小滑模抖振
static float SmcSat(float value, float boundary_layer)
{
    if (boundary_layer <= 0.0f)                  // 边界层无效时退化为符号函数
        return SmcSign(value);                   // 返回符号函数结果
    return Limit_Sat(value / boundary_layer, 1.0f, -1.0f); // 将归一化误差限制在 -1 到 1
}

// 将电角速度 rpm 换算为机械角速度 rad/s
static float SmcElecRpmToMechRad(float omega_elec_rpm, float pole_pairs)
{
    if (pole_pairs <= 0.0f)                      // 极对数异常时避免除零
        return 0.0f;                             // 返回零速度，保证控制器不会产生非法数
    return omega_elec_rpm * PIX2 / (60.0f * pole_pairs); // 电角速度除以极对数后换算为机械 rad/s
}

// 指数趋近律计算，对应图中的 εsgn(s)+qs
static float SmcExpReachingLawUpdate(SMC_SPEED_CTRL *ctrl, float slide_s)
{
    float sat_s = SmcSat(slide_s, ctrl->cfg.reaching.boundary_layer); // 对滑模面做边界层饱和处理

    return ctrl->cfg.reaching.q * slide_s + ctrl->cfg.reaching.epsilon * sat_s; // 计算 εsgn(s)+qs
}

// 超螺旋扰动观测器更新，后续替换观测器时优先替换本函数
static void SmcSuperTwistingObserverUpdate(SMC_SPEED_CTRL *ctrl, float omega_fbk_rad, float iq_fbk_u)
{
    float observer_err = 0.0f;                   // 速度观测误差
    float sat_observer_err = 0.0f;               // 经过边界层处理后的观测误差
    float sto_v1 = 0.0f;                         // 超螺旋一阶修正项
    float sto_v2 = 0.0f;                         // 超螺旋二阶扰动更新项
    float omega_hat_dot = 0.0f;                  // 速度观测值的微分项

    if (!ctrl->observer_ready)                   // 首次运行时观测器还没有初值
    {
        ctrl->out.omega_hat = omega_fbk_rad;     // 用当前反馈速度作为观测速度初值
        ctrl->out.disturbance_hat = 0.0f;        // 扰动估计从零开始，避免启动冲击
        ctrl->observer_ready = 1;                // 标记观测器已经完成初值同步
        return;                                  // 首次只同步状态，不计算补偿
    }

    observer_err = ctrl->out.omega_hat - omega_fbk_rad; // 计算观测速度与反馈速度之间的误差
    sat_observer_err = SmcSat(observer_err, ctrl->cfg.reaching.boundary_layer); // 对观测误差做边界层饱和处理
    sto_v1 = -ctrl->cfg.observer.k1 * sqrtf(SmcAbs(observer_err)) * sat_observer_err; // 计算超螺旋一阶修正项
    sto_v2 = -ctrl->cfg.observer.k2 * sat_observer_err; // 计算扰动估计的更新量

    omega_hat_dot = ctrl->cfg.input_gain_d * iq_fbk_u - // 图中 D*u，表示 q 轴电流产生的机械角加速度
                    (ctrl->cfg.damping / ctrl->cfg.inertia) * ctrl->out.omega_hat + // 阻尼造成的减速度
                    ctrl->out.disturbance_hat + sto_v1; // 叠加扰动估计和超螺旋修正项

    ctrl->out.omega_hat += omega_hat_dot * ctrl->cfg.ts; // 用离散积分更新速度观测值
    ctrl->out.disturbance_hat += sto_v2 * ctrl->cfg.ts; // 用离散积分更新扰动估计值
    ctrl->out.disturbance_hat = Limit_Sat(ctrl->out.disturbance_hat, // 对扰动估计做上限保护
                                          ctrl->cfg.observer.disturbance_limit, // 扰动估计正向限幅
                                          -ctrl->cfg.observer.disturbance_limit); // 扰动估计负向限幅
}

// 初始化 SMC 控制器参数，并清空运行状态
void SmcSpeed_Init(SMC_SPEED_CTRL *ctrl)
{
    ctrl->cfg.ts = SPEED_TIME;                   // 设置速度环离散周期
    ctrl->cfg.pole_pairs = (float)MOTOR_P;       // 设置电机极对数
    ctrl->cfg.kt = Kt;                           // 设置转矩常数
    ctrl->cfg.inertia = J;                       // 设置转动惯量
    ctrl->cfg.damping = B;                       // 设置阻尼系数
    ctrl->cfg.input_gain_d = Kt / J;             // 设置图中的 D，即 q 轴电流到机械角加速度的增益
    ctrl->cfg.iq_limit = SMC_IQ_LIMIT;           // 设置 SMC 输出电流限幅

    ctrl->cfg.reaching.q = SMC_REACH_Q;          // 设置图中的 q，对应线性趋近系数
    ctrl->cfg.reaching.epsilon = SMC_REACH_EPSILON; // 设置图中的 ε，对应符号趋近系数
    ctrl->cfg.reaching.boundary_layer = SMC_BOUNDARY_LAYER; // 设置滑模边界层宽度
    ctrl->cfg.reaching.mode = SMC_REACH_EXP;     // 选择当前趋近律为指数趋近律

    ctrl->cfg.observer.k1 = SMC_STO_K1;          // 设置超螺旋观测器一阶增益
    ctrl->cfg.observer.k2 = SMC_STO_K2;          // 设置超螺旋观测器二阶增益
    ctrl->cfg.observer.disturbance_limit = SMC_DISTURBANCE_LIMIT; // 设置扰动估计限幅
    ctrl->cfg.observer.mode = SMC_OBSERVER_SUPER_TWISTING; // 选择当前观测器为超螺旋观测器

    SmcSpeed_Reset(ctrl);                        // 初始化结束后清空内部运行状态
}

// 清空 SMC 控制器内部状态，用于停机、故障和重新启动
void SmcSpeed_Reset(SMC_SPEED_CTRL *ctrl)
{
    ctrl->out.iq_ref_u = 0.0f;                   // 清空图中的控制量 u，即 q 轴电流给定
    ctrl->out.x2_speed_err = 0.0f;               // 清空图中的 x2，即速度跟踪误差
    ctrl->out.slide_s = 0.0f;                    // 清空图中的滑模面 s
    ctrl->out.reach_law = 0.0f;                  // 清空图中的 εsgn(s)+qs 趋近律输出
    ctrl->out.disturbance_hat = 0.0f;            // 清空扰动估计
    ctrl->out.omega_hat = 0.0f;                  // 清空速度观测值
    ctrl->observer_ready = 0;                    // 清除观测器初值同步标志
}

// SMC 速度环主更新函数，输入速度参考和反馈，输出 q 轴电流给定
float SmcSpeed_Update(SMC_SPEED_CTRL *ctrl, const SMC_SPEED_INPUT *input)
{
    float omega_ref_rad = SmcElecRpmToMechRad(input->omega_ref_elec_rpm, ctrl->cfg.pole_pairs); // 将速度给定换算为机械 rad/s
    float omega_fbk_rad = SmcElecRpmToMechRad(input->omega_fbk_elec_rpm, ctrl->cfg.pole_pairs); // 将速度反馈换算为机械 rad/s
    float iq_ref_u = 0.0f;                       // 图中的控制量 u，实际为未限幅的 q 轴电流给定

    SmcSuperTwistingObserverUpdate(ctrl, omega_fbk_rad, input->iq_fbk_u); // 先更新扰动观测器状态

    ctrl->out.x2_speed_err = omega_ref_rad - omega_fbk_rad; // 计算图中的 x2，这里对应速度跟踪误差
    ctrl->out.slide_s = ctrl->out.x2_speed_err;  // 当前工程先采用一阶速度误差滑模面，后续可扩展为 s=cx1+x2
    ctrl->out.reach_law = SmcExpReachingLawUpdate(ctrl, ctrl->out.slide_s); // 计算图中的 εsgn(s)+qs

    iq_ref_u = (ctrl->out.reach_law +            // 加入趋近律给出的收敛需求
                (ctrl->cfg.damping / ctrl->cfg.inertia) * omega_fbk_rad - // 补偿阻尼项
                ctrl->out.disturbance_hat) / ctrl->cfg.input_gain_d; // 按图中的 1/D 换算为 q 轴电流

    ctrl->out.iq_ref_u = Limit_Sat(iq_ref_u, ctrl->cfg.iq_limit, -ctrl->cfg.iq_limit); // 对 q 轴电流给定限幅
    return ctrl->out.iq_ref_u;                   // 返回限幅后的 q 轴电流给定
}
