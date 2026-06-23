#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H

#include "stm32f407xx.h"
#include "task.h"
#include "delay.h"
#include "arm_math.h"
#include "foc.h"
#include "motor_para.h"
#include "pid.h"
#include "main.h"
#include "hmi_app.h"
#include "filter.h"
#include "stdbool.h"
#include "hfi.h"
#include "flux.h"
#include "sen.h"
#include "smo.h"
#include "smc_speed.h"
#include "fault.h"

typedef struct
{
    float M_theta;             // 电机转子机械角度
    float E_theta;             // 电机转子电角度
    float speed_M_rpm;         // 电机当前机械角速度	rpm/min
    float speed_E_rpm;         // 电机当前电角速度		rpm/min
    volatile uint8_t CtrlMode; // 电机控制模式
    float Rs;                  // 电机的相电阻
    float Ls;                  // 电机的相电感
    float Ib;                  // 电机相电流
    float Vb;                  // 电机相电压
    uint8_t P;                 // 电机极对数
} motor_p;

#define MOTOR_DEFAULTS {0.0f, 0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0} // 电机状态参数初始化

typedef struct
{
    volatile uint8_t Observer; // 无感控制时的观测器选择，1——HFI；3——SMO
    volatile uint8_t Observer_last;
    volatile float theta;     // 观测器角度
    volatile float Omega;     // 观测器电角速度，弧度制		rad/s
    volatile float Speed_Fbk; // 观测器速度反馈值
    float Speed_Ref;          // 观测器速度目标值
    int16_t Speed_Max;        // 观测器目标速度最大值
    int16_t w1;               // 观测器低速与过渡阶段临界值
    int16_t w2;               // 观测器高速与过渡阶段临界值
} Sensorless_Control;

#define Sensorless_DEFAULTS {1, 1, 0, 0, 0, 0, 0, 0, 0} // 观测器融合参数初始化

typedef struct
{                  // 指令参数的斜率处理
    float XieLv_X; // 指令参数斜率输入变量x
    float XieLv_Y;
    float XieLv_Grad;
    uint8_t Timer_Count;
    uint8_t Grad_Timer;
} GXieLv, *p_GXieLv;

#define GXieLv_DEFAULTS {0, 0, 0, 0, 0}

typedef struct
{
    uint16_t IfFrq;       // 强拖频率
    uint16_t IfAngle;     // 强拖角度值
    uint16_t Locktime;    // 堵转检测时间
    uint8_t SpeedCalTime; // 速度计算时间
    float IfTheta;        // 强拖角度
    float IfCurrent;      // 强拖电流
    int16_t SpeedRef;     // 目标速度
    float CurRef;         // 目标电流
    int32_t PosRef;       // 目标位置
    double RpmCmd;
    float FluxErr;
    float HfiSmoErr;
    float FocAngle;
    uint16_t FluxSpeedCnt;
    uint8_t ToZeroState;
    uint16_t UpTime;
    uint16_t DownTime;
    uint8_t CalTime;
    bool ObsSwFlag;
    bool SmoSwFlag;
    float SpdSenKp;
    float SpdSenKi;
    float SpdLessKp;
    float SpdLessKi;
} PARA;
extern PARA Para;

typedef struct
{
    int32_t P_Ref_Err;
    int32_t P_Ref_Old;
    int32_t P_Fdb;
    int32_t P_Ref;
    int32_t P_Err;
    int32_t P_Err_Max;
    float Kp;
    int32_t Ki;
    int16_t P_Out;

} POS_DRV;
extern POS_DRV Pos;

extern CLARKE Clarke;
extern PARK Park;
extern IPARK Ipark;
extern GXieLv IfFreqGXieLv;
extern GXieLv SpdRefGXieLv;
extern IPARK IparkSmoPv;

extern bool SMO_flag;
extern bool SMO_Switch_flag;
extern bool HFI_Init_Angle_flag;
extern bool Init_Over;

extern bool HFI_Init_Angle_flag;

extern motor_p Motor;
extern const float ubeta_tab[360];
extern const float ualpha_tab[360];
extern Sensorless_Control SensorLess;

void GradXieLv(p_GXieLv pv); // 梯度加减
void SvpwmCal(M_SVPWM pv);   // SVPWM调制算法函数
void MotorStop(void);        // 电机停止
void MotorRun(void);         // 电机启动
void AxisDq(void);           // 坐标轴变换
void FocSvpwm(void);         // SVPM计算
void QtMotor(void);          // 强制拖动
void MotorModeSel(void);     // 控制模式选择
void VariableClear(void);    // 变量清零
void IFStartControl(void);   // IF启动
void IFObserverStartInit(void); // IF观测器启动初始化

void MotorInit(void);             // 电机参数总初始化
float my_atan2(float x, float y); // 反正切函数		atan2(x,y)
float my_atan(float x);           // 反正切计算		atan(x)
int8_t Sign(float x);             // 符号函数
float my_abs(float x);            // 取绝对值
float Sat(float s, float delta);  // 饱和函数
float my_max(float a, float b);   // 取最大值函数
// double myln(double a);								//实现ln计算

#endif
