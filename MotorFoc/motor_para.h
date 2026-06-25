#ifndef __MOTOR_PARA_H
#define __MOTOR_PARA_H

#include "delay.h"
#include "arm_math.h"

// 系统参数
#define TIM1_PERIOD /*******************/ 8400                       // 定时器1的重装载值
#define TIM1_PERIOD_HALF /**************/ (TIM1_PERIOD / 2)          // 定时器1重装载值的一半
#define PWM_FRQ /***********************/ 20000                      // 定时器1的中断频率
#define TS /****************************/ 0.00005f                   // SVPWM的方波周期
#define SPEED_CNT /*********************/ 2                          // 有感速度环计算频率，1K
#define ENCODER_FRQ /*******************/ (PWM_FRQ / SPEED_CNT) * 60 // PWM_FRQ/SPEED_CNT*60
#define SPEED_TIME /********************/ TS *SPEED_CNT
#define SPD_CAL_PERIOD /****************/ 20             // 速度环计算周期 2k
#define PIX2 /**************************/ 6.28318530718f // 2PI
#define DEG_10 /************************/ 0.1745329252f  // 10°角度制对应的弧度制

// 编码器参数
#define ENCODE_LINES /******************/ 1024
#define ENCODE_PULSE /******************/ (ENCODE_LINES * 4) // ENCO_LINES*4
#define TIM3_PULSE /********************/ ENCODE_PULSE       // 定时器3重装载值

// 电机参数
#define MOTOR_P /***********************/ 7         // 电机极对数
#define MOTOR_Ls /**********************/ 0.00004625f // 电机自感
#define MOTOR_Rs /**********************/ 0.224f       // 电机内阻

// ADC采样参数
#define ADC_REFV /**********************/ 3.3f      // 单位：V，ADC参考电压
#define ADC_REF_VALUE /*****************/ 4095      // 12-bit ADC
#define I_GAIN /************************/ 0.018315f // 电流计算系数  (3.3/4095) / 0.044 其中0.044为电流传感器系数
// #define I_GAIN /************************/ 0.0061f     // 电流计算系数  (3.3/4095) / 0.132  其中0.132为电流传感器系数
#define VBUS_GAIN /******************* */ 0.0088653554f // 母线电压计算系数 (3.3/4095) / (4.7/(4.7+47)) 其中4.7K和47K为分压电阻阻值

// FOC参数
#define SART_OF_3 /*********************/ 0.57735f
#define SVPWM_KM /**********************/ (STAN_VOLTAGE_V * SART_OF_3)
#define SVPWM_KM_BACKW /****************/ 0.1443376f // 倒数  1/(STAN_VOLTAGE_V*SART_OF_3)
#define VBUS_S /************************/ 6.651f     //  (STAN_VOLTAGE_V*SART_OF_3*0.96)
#define PI_MAX_Ud /*********************/ 6.0f       //  (STAN_VOLTAGE_V*SART_OF_3*0.92)   // 13.5  //12.347
#define PI_MAX_Uq /*********************/ 6.0f       //  (STAN_VOLTAGE_V*SART_OF_3*0.92)
#define PI_MAX_SPD /********************/ 10.0f      // 最大电流限制

// 速度环控制器选择：0 使用原 PI，1 使用 SMC
#define SPEED_CTRL_USE_SMC /************/ 1           // 速度环一键切换宏，调试异常时改为 0 可回退 PI
#define SMC_STO_ENABLE /***************/ 1           // SMC 超螺旋扰动观测器使能，0 为纯滑模速度环
#define SMC_CONTROL_MODE_CFG /**********/ 0           // SMC 控制律模式，0 为主流等效控制律
#define SMC_ACTIVE_DAMP_WHEN_SMC /*****/ 0           // SMC 模式是否叠加外层主动阻尼，默认关闭避免重复补偿
#define SMC_IQ_LIMIT /******************/ PI_MAX_SPD // SMC 的 q 轴电流限幅，当前复用原速度 PI 限幅
#define SMC_INTEGRAL_LIMIT /*************/ 200   // 积分项 x1 限幅(仿 PI 积分限幅)，防 windup 残留导致过冲缓跌
#define SMC_SLIDE_C /*******************/ 2.0f       // 图中 c，滑模面 s=c*x1+x2 的速度误差系数22
#define SMC_REACH_Q /*******************/ 1000.0f       // 图中 q，指数趋近律线性项系数20.7
#define SMC_REACH_EPSILON /*************/ 400.0f       // 指数趋近律饱和项系数，增强靠近滑模面的收敛速度
#define SMC_BOUNDARY_LAYER /************/ 1.0f        // 滑模边界层宽度，用于减小符号函数抖振
#define SMC_STO_K1 /********************/ 400.0f       // 超螺旋观测器一阶增益，修正速度观测误差 80
#define SMC_STO_K2 /********************/ 40000.0f      // 超螺旋观测器二阶增益，更新扰动估计值 500
#define SMC_DISTURBANCE_LIMIT /*********/ 30000.0f     // 扰动估计限幅，防止观测器输出过大
#define kserver /********************/ 0.5f       // 服务器系数
#define Umod_NOW /********************/ (sqrtf(Svpwm.Ualpha * Svpwm.Ualpha + Svpwm.Ubeta *Svpwm.Ubeta))
#define SVPWM_KM_NOW /********************/ (AdcPara.Vbus * SART_OF_3)
#define SVPWM_KM_NOW_BACKW /********************/ (1.0f / SVPWM_KM_NOW) // 倒数  1/(AdcPara.Vbus*SART_OF_3)
#define CCR4_MAX /********************/   8325//((TIM1_PERIOD-1)-(SVPWM_KM_NOW-Umod_NOW)*SVPWM_KM_NOW_BACKW*TIM1_PERIOD_HALF*kserver)    // 定时器1_CH4的重装载值
#define Kt /**************************/ 0.0007404636f // 电机转矩常数
#define B /**************************/  0.0000016247f // 黏性摩擦系数B
#define Wm /**************************/ ((Motor.speed_M_rpm/60)*PIX2) // 电机转速
#define W_alpha /**************************/ 3090.588f
#define J /**************************/ 0.0000000958346f // 电机惯量
#define Psi_f /**************************/ 0.00007052f // 电机磁通量常数

//死区补偿参数
#define DT_COMP_EN /*******************/ 1     // 死区补偿使能：1-使能，0-关闭
#define DT_COMP_CCR /******************/ 30    // 死区补偿计数值，初始建议从 40 开始调试
#define DT_COMP_CUR_TH /***************/ 0.02f //电流符号判定死区，避免过零附近抖动翻转
#define DT_COMP_CCR_MARGIN /***********/ 3     // CCR限幅保护边界，防止补偿后贴边

//速度环有功阻尼参数
#define SPD_BETA /************************/ 20.0f
#define SPD_ACTIVE_DAMP_BA /**************/ ((SPD_BETA * J - B) / (1.5f * MOTOR_P * Psi_f))
#define SPD_ACTIVE_DAMP_KE /**************/ (SPD_ACTIVE_DAMP_BA * PIX2 / (60.0f * MOTOR_P))
#define SPD_ACTIVE_DAMP_MAX /*************/ 5.0f
#define SPD_ACTIVE_DAMP_EN /**************/ 1


// 电机运行模式
#define M_ERR /*************************/ 11 // 电机错误状态
#define STOP /**************************/ 0  // 电机停止状态
#define SPEED_LOOP /********************/ 1  // 速度闭环控制模式
#define CUR_LOOP /**********************/ 2  // 电流闭环控制模式
#define POS_LOOP /**********************/ 3  // 位置闭环控制模式
#define TO_ZERO /***********************/ 10  // 校准模式
#define IF_CTRL /***********************/ 4  // IF控制
#define HFI_CTRL /**********************/ 5  // HFI控制
#define HFI_SMO /***********************/ 6  // 低速HFI高速SMO
#define HFI_FLUX /**********************/ 7  // 低速HFI高速磁链
#define IF_SMO /************************/ 8  // IF启动高速SMO
#define IF_FLUX /***********************/ 9  // IF启动高速磁链
#define V_F /**************************/  12  // 压频比开环控制

// IF强拖参数
#define QT_ANGLE /**********************/ 210  // 强拖至角度（0——360）
#define QT_FRQ /************************/ 20   // 强拖频率=PWM_FRQ/QT_FRQ
#define QT_IF_MODE /********************/ 3    // 1——强拖至角度；2——强拖旋转；3——IF启功
#define IF_F_GRAD_TIMER /***************/ 10   // 定义角度改变频率的梯度时间
#define IF_F_GRAD_0D1HZ /***************/ 0.1f // 定义角度改变频率的梯度值
#define IF_FRE_MAX /********************/ 100  // IF角度改变上限值
#define IF_CUR_MAX /********************/ 2.0f // 速度开环，电流闭环中的电流上限值

#define MOTOR_FRE_MIN /*****************/ 2   // 定义频率的最小值
#define MOTOR_FRE_MAX /*****************/ 200 // 定义频率最大值

// 滤波参数
#define LPF_I_STOP_A /******************/ 0.00314159265f // 截止频率10Hz
#define LPF_I_STOP_B /******************/ 0.99685840735f

#define LPF_I_RUN_A /*******************/ 0.37699f // 截止频率1200Hz
#define LPF_I_RUN_B /*******************/ 0.62301f

#define LPF_U_A /***********************/ 0.015465039f // 截止频率100Hz
#define LPF_U_B /***********************/ 0.984534961f

// 速度环参数
#define SPD_REF_TIMER /*****************/ 30   // 定义目标速度改变的梯度时间
#define SPD_REF_OD /********************/ 5.0f // 定义目标速度改变的梯度值
#define SPD_REF_MAX /*******************/ 4000 // 目标速度上限值，机械角速度

// HFI高频注入参数
#define HFI_UIN_OFFSET /****************/ 1.0f // HFI注入高频电压偏置
#define HFI_ID_OFFSET /*****************/ 2.0f // HFI初始位置检测时，注入的Id电流偏置

#define HFI_SPD_REF_MAX /***************/ 350 // HFI最大转速速度限制

#define LPF_HFI_A /*********************/ 0.011f // 低通滤波系数，截止频率300Hz
#define LPF_HFI_B /*********************/ 0.989f
#define LPF_HFI_I_A /*******************/ 0.9424778f // 截止频率3KHz
#define LPF_HFI_I_B /*******************/ 0.0575222f

// SMO滑膜参数
#define LPF_SMO_A /*********************/ 0.23905722361f // 低通滤波系数，截止频率1000Hz
#define LPF_SMO_B /*********************/ 0.76094277639f
#define SMO_KSLIDE /********************/ 1.868f      // 滑膜增益K
#define SMO_KSLF /**********************/ 0.12566f    // 低通滤波系数
#define wc /****************************/ 1200 * PIX2 // wc=截止频率*2Pi

#define LPF_PLL_A /*********************/ 0.09425f // 锁相环速度低通滤波系数，截止频率300Hz
#define LPF_PLL_B /*********************/ 0.90575f

// 磁链参数
#define FLUX_KE /***********************/ 20.0f    // 漏散衰减系数,单位： rad/s
#define PSI_LIMIT /*********************/ 0.05f    // 磁链限幅，单位：Wb
#define PLL_KP_DEF /********************/ 540.35f  // PLL参数
#define PLL_KI_DEF /********************/ 72995.6f // PLL参数

// 电压保护参数
#define LOW_VOLTAGE_V /*****************/ 9.0f  // 低压保护
#define STAN_VOLTAGE_V /****************/ 12.0f // 标准工作电压
#define HIHG_VOLTAGE_V /****************/ 15.0f // 高压保护
#define LV_PROTECT_TIME /***************/ 500   // 欠压保护检测时间, 单位: ms
#define OV_PROTECT_TIME /***************/ 500   // 过压保护检测时间, 单位: ms

// 温度保护参数
#define EN_MOS_TEMP_DETECT /************/ (1)   // MOS温度保护检测使能
#define MOS_TEMP_UP_VOL /***************/ 3.3f  // MOS温度检测上拉电压，单位：V
#define MOS_TEMP_UP_RES /***************/ 10.0f // MOS温度检测上拉电阻，单位：KΩ
#define MOS_TEMP_OVER_RES /*************/ 0.98f // MOS过温时NTC阻值，100℃对应1.0KΩ
#define RSM_MOS_TEMP_OVER_RES /*********/ 1.66f // MOS过温恢复NTC阻值，90℃对应1.5KΩ

#define MOS_TEMP_OVER_TIME /************/ 500 // 单位：ms
#define RSM_MOS_TEMP_OVER_TIME /********/ 300 // 单位：ms
#define MOS_TEMP_TIME /*****************/ 300 // 单位：ms
/* 将用户设定值转换为ADC采样值 */
#define MOS_TEMP_OVER_RES_RATIO /*******/ (float)(MOS_TEMP_OVER_RES / (MOS_TEMP_OVER_RES + MOS_TEMP_UP_RES))         // 分压比 (下拉电阻/(上拉电阻+下拉电阻))
#define RSM_MOS_TEMP_OVER_RES_RATIO /***/ (float)(RSM_MOS_TEMP_OVER_RES / (RSM_MOS_TEMP_OVER_RES + MOS_TEMP_UP_RES)) // 分压比 (下拉电阻/(上拉电阻+下拉电阻))

#define MOS_TEMP_OVER_THD_ADC /*********/ (uint16_t)(MOS_TEMP_OVER_RES_RATIO * MOS_TEMP_UP_VOL * ADC_REF_VALUE / ADC_REFV)     // 计算出过温保护门槛值
#define RSM_MOS_TEMP_OVER_THD_ADC /*****/ (uint16_t)(RSM_MOS_TEMP_OVER_RES_RATIO * MOS_TEMP_UP_VOL * ADC_REF_VALUE / ADC_REFV) // 计算出解除过温保护门槛值

// 堵转保护参数
#define LOCK_TIME /*********************/ 500  // 堵转时间，单位：ms
#define LOCK_CUR /**********************/ 2.0f // 堵转电流，单位：A
#define SPEED_PRO_TIME /****************/ 100  // 速度保护时间，单位：ms

// 过流保护参数
#define OVER_CURRENT /*******************/ 2.0f // 过流保护阈值

#define RGB_Luminance /*****************/ 4 // RGB LED亮度调节，一共8档(0~7)
#define VR_OR_PC /**********************/ 0 // 电机命令方式: VR: 1 或 PC调速: 0

#endif
