#include "motor_ctrl.h"

DiagFlag Fault;

uint32_t MotorOVState = 0;
uint32_t MotorLVState = 0;
uint32_t Locktime = 0;
uint32_t OverSpeedTime = 0;
uint32_t DownSpeedTime = 0;
uint32_t MotorOtpState = 0;
uint32_t MotorDisOtoState = 0;
// 故障检测
void FaultCheck(void)
{
    static uint16_t i;

    if (Init_Over) // 初始化结束后进行电机状态异常检测
    {
        // 过压
        if (AdcPara.Vbus > HIHG_VOLTAGE_V)
        {
            if (MotorOVState < OV_PROTECT_TIME)
            {
                MotorOVState++;
            }
            else
            {
                Fault.bit.OverVbusFlag = 1;
                MotorStop();            // 停机
                Motor.CtrlMode = M_ERR; // 电机状态报错
                LedTask(1, 0, 1);       // 紫色灯报警
            }
        }
        else
        {
            MotorOVState = 0;
        }

        // 欠压
        if (AdcPara.Vbus < LOW_VOLTAGE_V)
        {
            if (MotorLVState < LV_PROTECT_TIME)
            {
                MotorLVState++;
            }
            else
            {
                Fault.bit.UnderVbusFlag = 1;
                MotorStop();            // 停机
                Motor.CtrlMode = M_ERR; // 电机状态报错
                LedTask(1, 1, 0);       // RGB灯常亮报警
            }
        }
        else
        {
            MotorLVState = 0;
        }

        // 防堵转检测
        if (my_abs(pi_spd.Fbk) < my_abs(pi_spd.Ref / 2)) // 当反馈速度低于目标速度的一半，开启堵转检测
        {
            if (my_abs(Park.Qs) >= LOCK_CUR)
            {
                Locktime++;
            }
        }
        else
        {
            Locktime = 0;
        }

        if (Locktime == LOCK_TIME) // 堵转500ms
        {
            MotorStop(); // 停机
            Fault.bit.LockedFlag = 1;
            Locktime = 0;
            Motor.CtrlMode = M_ERR; // 电机状态报错
            LedTask(1, 0, 0);       // RGB灯常亮报警
        }
    }

    // 转速偏差过大检测
    if (my_abs(pi_spd.Fbk) > 400 * Motor.P) // 反馈速度大于200RPM后，开启速度偏差检测
    {
        if (pi_spd.err < 0) // 速度高于一倍的目标速度开启保护
        {
            if (my_abs(pi_spd.err) > my_abs(pi_spd.Ref))
            {
                if (OverSpeedTime < SPEED_PRO_TIME)
                {
                    OverSpeedTime++;
                }
                else
                {
                    MotorStop();                 // 停机
                    Fault.bit.OverSpeedFlag = 1; // 失速报警
                    Motor.CtrlMode = M_ERR;      // 电机状态报错
                    LedTask(0, 0, 1);            // RGB灯常亮报警
                }
            }
            else
            {
                OverSpeedTime = 0;
            }
        }
        if ((Sign(pi_spd.Ref) + Sign(pi_spd.Fbk)) == 0) // 与目标速度反方向转报警
        {
            if (DownSpeedTime < SPEED_PRO_TIME)
            {
                DownSpeedTime++;
            }
            else
            {
                MotorStop();                 // 停机
                Fault.bit.OverSpeedFlag = 1; // 失速报警
                Motor.CtrlMode = M_ERR;      // 电机状态报错
                LedTask(0, 0, 1);            // RGB灯常亮报警
            }
        }
         else
        {
            DownSpeedTime = 0;
        }
    }

    // 高温报警
    if (AdcPara.Temp < MOS_TEMP_OVER_THD_ADC) // 过温
    {
        if (MotorOtpState < MOS_TEMP_OVER_TIME)
        {
            MotorOtpState++;
        }
        else
        {
            MotorStop();
            Fault.bit.OverTempFlag = 1;
            Motor.CtrlMode = M_ERR; // 电机状态报错
            MotorDisOtoState = 0;
        }
    }
    else if (AdcPara.Temp > RSM_MOS_TEMP_OVER_THD_ADC) // 过温恢复
    {
        if (Fault.bit.OverTempFlag == 1)
        {
            if (MotorDisOtoState < MOS_TEMP_OVER_TIME)
            {
                MotorDisOtoState++;
            }
            else
            {
                Fault.bit.OverTempFlag = 0;
                MotorOtpState = 0;
            }
        }
        else
        {
            MotorOtpState = 0;
        }
    }

    if (Motor.CtrlMode != M_ERR)
    {
        i++;
        Fault.Byte = 0;     

        if (i < 200)
            LedTask(0, 1, 0); // 系统正常运行，绿色灯闪烁
        else
            LedTask(0, 0, 0);

        i %= 400;
    }
}
