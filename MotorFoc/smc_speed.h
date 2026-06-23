#ifndef __SMC_SPEED_H
#define __SMC_SPEED_H

#include "motor_para.h"                          // 引入电机参数和 SMC 参数宏
#include <stdint.h>                               // 引入 uint8_t 等标准整数类型

// 趋近律类型枚举，后续新增模糊自适应趋近律时从这里扩展
typedef enum
{
    SMC_REACH_EXP = 0,                            // 当前使用指数趋近律
} SMC_REACHING_MODE;

// 扰动观测器类型枚举，后续新增其他观测器时从这里扩展
typedef enum
{
    SMC_OBSERVER_SUPER_TWISTING = 0,              // 当前使用超螺旋扰动观测器
} SMC_OBSERVER_MODE;

// 趋近律参数结构体，对应图中的 ε 和 q
typedef struct
{
    float q;                                      // 图中 q，滑模面线性趋近系数
    float epsilon;                                // 图中 ε，符号趋近项系数
    float boundary_layer;                         // 滑模边界层宽度，用饱和函数替代纯符号函数
    SMC_REACHING_MODE mode;                       // 趋近律模式，便于后续替换算法
} SMC_REACHING_CFG;

// 扰动观测器参数结构体，只保存观测器自身需要的参数
typedef struct
{
    float k1;                                     // 超螺旋一阶修正增益，作用在速度观测误差上
    float k2;                                     // 超螺旋二阶修正增益，用来更新扰动估计
    float disturbance_limit;                      // 扰动估计限幅，防止估计值失控
    SMC_OBSERVER_MODE mode;                       // 观测器模式，便于后续替换观测器
} SMC_OBSERVER_CFG;

// SMC 速度控制器总配置结构体，集中保存模型参数、限幅和可替换模块配置
typedef struct
{
    float ts;                                     // 速度环更新周期，来自 SPEED_TIME
    float pole_pairs;                             // 电机极对数，用于电角速度和机械角速度换算
    float kt;                                     // 转矩常数，用于把 q 轴电流换算成电磁转矩
    float inertia;                                // 转动惯量，对应机械模型中的 J
    float damping;                                // 粘滞阻尼系数，对应机械模型中的 B
    float input_gain_d;                           // 图中 D，对应 q 轴电流到机械角加速度的输入增益
    float iq_limit;                               // SMC 输出的 q 轴电流限幅
    SMC_REACHING_CFG reaching;                    // 趋近律配置，当前为指数趋近律
    SMC_OBSERVER_CFG observer;                    // 扰动观测器配置，当前为超螺旋观测器
} SMC_SPEED_CFG;

// SMC 速度控制器输入结构体，只接收速度环必须的外部量
typedef struct
{
    float omega_ref_elec_rpm;                     // 电角速度给定，单位 rpm，沿用原速度环单位
    float omega_fbk_elec_rpm;                     // 电角速度反馈，单位 rpm，来自 Motor.speed_E_rpm
    float iq_fbk_u;                               // 图中 u 的反馈量，实际为当前 q 轴电流 Park.Qs
} SMC_SPEED_INPUT;

// SMC 速度控制器输出和内部观测状态结构体，便于按图调试观察
typedef struct
{
    float iq_ref_u;                               // 图中控制量 u，实际输出为 q 轴电流给定
    float x2_speed_err;                           // 图中 x2，这里对应速度跟踪误差
    float slide_s;                                // 图中 s，滑模面
    float reach_law;                              // 图中 εsgn(s)+qs，对应趋近律输出
    float disturbance_hat;                        // 扰动估计值，用于补偿负载扰动
    float omega_hat;                              // 速度观测值，用于超螺旋观测器内部迭代
} SMC_SPEED_OUTPUT;

// SMC 速度控制器对象，包含配置、输出状态和观测器就绪标志
typedef struct
{
    SMC_SPEED_CFG cfg;                            // 控制器配置参数
    SMC_SPEED_OUTPUT out;                         // 控制器输出和观测状态
    uint8_t observer_ready;                       // 观测器初始化标志，避免首次运行产生跳变
} SMC_SPEED_CTRL;

extern SMC_SPEED_CTRL SmcSpeed;                   // 全局 SMC 速度控制器实例

void SmcSpeed_Init(SMC_SPEED_CTRL *ctrl);         // 初始化 SMC 参数并清空内部状态
void SmcSpeed_Reset(SMC_SPEED_CTRL *ctrl);        // 清空 SMC 内部状态，用于停机和重启
float SmcSpeed_Update(SMC_SPEED_CTRL *ctrl, const SMC_SPEED_INPUT *input); // 更新 SMC 并返回 q 轴电流给定

#endif
