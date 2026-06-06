/*
***********************************************************************
* 文件名：hmi_app.c
* 作  者：？？？
* 日  期：2024/10/07
* 说  明：人机交互任务
***********************************************************************
*/

#include "hmi_app.h"
#include "led_drv.h"
#include "delay.h"
#include "string.h"
#include "motor_ctrl.h"
#include "key_drv.h"

FloatToChar fc;
USART_DATA Usart;

const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f}; // JustFloat规定{0x00, 0x00, 0x80, 0x7f}为一组数据的结束

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) // 如果是串口1
	{
		Usart.RxLine++;											 // 每接收到一个数据，进入回调数据长度加1
		Usart.UsartRxBuf[Usart.RxLine - 1] = Usart.aRxBuffer[0]; // 把每次接收到的数据保存到缓存数组
		if (Usart.aRxBuffer[0] == 0xfa)							 // 接收结束标志位，这个数据可以自定义，根据实际需求
		{
			Usart.UsartRxSta = 1; // 接收完成标志位置一
		}
		Usart.aRxBuffer[0] = 0; // 清空串口接收缓存
	}
}

/*JustFloat*/
// 定义一个联合体，将传入的浮点型参数存入到传入的unsigned char数组里
void float_turn_u8(float f, uint8_t *c)
{
	uint8_t x;
	FloatLongType data;
	data.fdata = f;

	for (x = 0; x < 4; x++)
	{
		c[x] = (uint8_t)(data.ldata >> (x * 8));
	}
}

/*

*JustFloat:
*fdata:传入数组指针
*fdata_num:数组数据个数
*Usart_choose:发送的串口地址

前两个for将 float类型转换位四位unsigned char类型并发送出去，第三个for循环将尾部发出去
*/

static void JustFloatSend(float *fdata, uint16_t fdata_num, USART_TypeDef *Usart_choose)
{
	uint16_t x;
	uint8_t y;
	for (x = 0; x < fdata_num; x++)
	{
		float_turn_u8(fdata[x], Usart.Cdata);
		for (y = 0; y < 4; y++)
		{
			Usart_choose->DR = Usart.Cdata[y];
			while ((Usart_choose->SR & 0X40) == 0)
				;
		}
	}

	for (y = 0; y < 4; y++)
	{
		Usart_choose->DR = tail[y];
		while ((Usart_choose->SR & 0X40) == 0)
			;
	}
}

/*
 * 根据串口信息进行PID调参
 */
static void VofaPiAdjust(uint8_t *DataBuff, uint8_t on)
{
	static uint8_t reg = 0;
	memcpy(&fc.c_data, DataBuff, 4); // 遵循IEEE 754协议，将数据前四位赋予共用体

	reg = DataBuff[6]; // 命令

	switch (reg)
	{
	case 0x01:
		pi_spd.Kp = fc.f_data;
		break;
	case 0x02:
		pi_spd.Ki = fc.f_data;
		break;
	case 0x03:
		pi_iq.Kp = fc.f_data;
		break;
	case 0x04:
		pi_iq.Ki = fc.f_data;
		break;
	case 0x05:
		Para.CurRef = fc.f_data;
		break;
	case 0x06:
		Para.PosRef = fc.f_data;
		break;
	case 0x07:
		Para.SpeedRef = fc.f_data;
		break;
	case 0x08:
	{
		switch ((uint8_t)fc.f_data)
		{
		case STOP:
		{
			MotorStop();
			Motor.CtrlMode = STOP;
			break;
		}
		case SPEED_LOOP:
		{
			MotorRun();					 // 启动PWM
			Motor.CtrlMode = SPEED_LOOP; // 有感，速度&电流闭环
			break;
		}
		case CUR_LOOP:
		{
			MotorRun();				   // 启动PWM
			Motor.CtrlMode = CUR_LOOP; // 有感，电流闭环
			break;
		}
		case POS_LOOP:
		{
			MotorRun();				   // 启动PWM
			Motor.CtrlMode = POS_LOOP; // 有感，位置闭环
			break;
		}
		case IF_CTRL:
		{
			MotorRun();				  // 启动PWM
			Motor.CtrlMode = IF_CTRL; // IF启动
			break;
		}

		case HFI_CTRL:
		{
			PllHfiPara.Theta = 0;
			MotorRun();				   // 启动PWM
			Motor.CtrlMode = HFI_CTRL; // HFI模式
			break;
		}

		case HFI_SMO:
		{
			MotorRun();				  // 启动PWM
			Motor.CtrlMode = HFI_SMO; // 低速HFI，高速滑膜
			break;
		}

		case HFI_FLUX:
		{
			MotorRun();				   // 启动PWM
			Motor.CtrlMode = HFI_FLUX; // 低速HFI，高速磁链
			break;
		}

		case IF_SMO:
		{
			IFObserverStartInit();
			MotorRun();
			Motor.CtrlMode = IF_SMO; // IF启动，高速滑膜
			break;
		}

		case IF_FLUX:
		{
			IFObserverStartInit();
			MotorRun();
			Motor.CtrlMode = IF_FLUX; // IF启动，高速磁链
			break;
		}
		case TO_ZERO:
		{
			Motor.CtrlMode = TO_ZERO;
			PostionToZeroDouble(); // 二次调零
			Motor.CtrlMode = STOP;
			break;
		}
		default:
			break;
		}
	}
	break;

	default:
		break;
	}
}

void RecevToVofa(void)
{
	if (Usart.UsartRxSta)
	{
		if (Usart.UsartRxBuf[5] == 0xfb) // 校验位，检测接收到的数据是否正确
		{
			VofaPiAdjust(Usart.UsartRxBuf, 1); // 数据解析和参数赋值函数
		}
		memset(Usart.UsartRxBuf, 0, 8); // 清空缓存数组
		Usart.RxLine = 0;				// 清空接收长度
		Usart.UsartRxSta = 0;
	}
}

// 数据发送至上位机观测
void SendToVofa(void)
{
	Usart.VofaData[0] = pi_iq.Ref;	  // 电角度
	Usart.VofaData[1] = pi_iq.Fbk;		  	  // 目标电流
	Usart.VofaData[2] = pi_iq.Out; 			  // 实际电流
	Usart.VofaData[3] = CCR4_MAX;		  // 编码器角度
	Usart.VofaData[4] = AdcFilPara.CurU;		  // 磁链观测角度
	Usart.VofaData[5] = AdcFilPara.CurV;	  // HFI角度
	Usart.VofaData[6] = AdcFilPara.CurW;	  // 滑膜观测角度
	Usart.VofaData[7] = pi_id.Ref;		  // IF强拖角度
	Usart.VofaData[8] = pi_id.Fbk;	  // U相电流
	Usart.VofaData[9] = pi_spd.OutF;	  // V相电流
	Usart.VofaData[10] = Wm;	  // W相电流
	Usart.VofaData[11] = Park.Ds;			  // 实际位置
	Usart.VofaData[12] = Park.Qs;			  // 目标位置
	JustFloatSend(Usart.VofaData, 13, USART1);
}

/**
***********************************************************
* @brief LED应用程序
* @param
* @return
***********************************************************
*/
#define LED_B_INDEX 0
#define LED_G_INDEX 1
#define LED_R_INDEX 2
void LedTask(uint8_t r, uint8_t g, uint8_t b)
{
	/* 红 */
	if (r)
		TurnOnLed(LED_R_INDEX);
	else
		TurnOffLed(LED_R_INDEX);

	/* 绿 */
	if (g)
		TurnOnLed(LED_G_INDEX);
	else
		TurnOffLed(LED_G_INDEX);

	/* 蓝 */
	if (b)
		TurnOnLed(LED_B_INDEX);
	else
		TurnOffLed(LED_B_INDEX);
}

/**
***********************************************************
* @brief 电位器应用程序
* @param
* @return
***********************************************************
*/
void VrTask(void)
{
	static float AdcVr = 0;

	switch (Motor.CtrlMode)
	{
	case STOP:
	{
		Para.SpeedRef = 0;
		Para.CurRef = 0;
		break;
	}
	case SPEED_LOOP:
	{
		AdcVr = (float)AdcPara.Vr;
		Para.SpeedRef = (int16_t)AdcVr;
		Para.SpeedRef = Limit_Sat(Para.SpeedRef, SPD_REF_MAX, 100.0f);
		break;
	}
	case CUR_LOOP:
	{
		AdcVr = (float)(AdcPara.Vr / 2000.0f);
		Para.CurRef = (float)AdcVr;
		Para.CurRef = Limit_Sat(Para.CurRef, 1.0f, 0.05f);
		break;
	}
	case POS_LOOP:
	{
		Para.PosRef = (int32_t)AdcPara.Vr;
		break;
	}
	case IF_CTRL:
	{
		AdcVr = (float)(AdcPara.Vr / 1000.0f);
		Para.CurRef = (float)AdcVr;
		Para.CurRef = Limit_Sat(Para.CurRef, 4.0f, 0.05f);
		break;
	}

	case HFI_CTRL:
	{
		AdcVr = (float)(AdcPara.Vr / 10.0f);
		Para.SpeedRef = (int16_t)AdcVr;
		Para.SpeedRef = Limit_Sat(Para.SpeedRef, HFI_SPD_REF_MAX, 10.0f);
		break;
	}

	case HFI_SMO:
	case HFI_FLUX:
	case IF_SMO:
	case IF_FLUX:
	{
		AdcVr = (float)AdcPara.Vr;
		Para.SpeedRef = (int16_t)AdcVr;
		Para.SpeedRef = Limit_Sat(Para.SpeedRef, SPD_REF_MAX, 500.0f);
		break;
	}
	case TO_ZERO:
	{
		break;
	}
	default:
		break;
	}
}

/**
***********************************************************
* @brief 按键应用程序
* @param
* @return
***********************************************************
*/
void HmiTask(void)
{
	uint8_t keyVal;

	keyVal = GetKeyVal();

	switch (keyVal)
	{
	case KEY4_SHORT_PRESS:
	{
	}
	break;

	case KEY4_DOUBLE_PRESS:
	{
	}
	break;

	case KEY4_LONG_PRESS:
	{
		HAL_GPIO_WritePin(PWR_GPIO_Port, PWR_Pin, GPIO_PIN_RESET); // 长按关机
	}
	break;
	case KEY1_SHORT_PRESS:
	{
		MotorRun();				  // 启动PWM
		Motor.CtrlMode = HFI_SMO; // 低速HFI，高速滑膜
	}
	break;

	case KEY1_DOUBLE_PRESS:
	{
		MotorRun();				 // 启动PWM
		Motor.CtrlMode = IF_SMO; // 低速IF高速SMO
	}
	break;

	case KEY1_LONG_PRESS:
	{
		MotorRun();				   // 启动PWM
		Motor.CtrlMode = HFI_FLUX; // 低速HFI，高速磁链
	}
	break;

	case KEY2_SHORT_PRESS:
	{
		MotorRun();					 // 启动PWM
		Motor.CtrlMode = SPEED_LOOP; // 有感，速度&电流闭环
	}
	break;

	case KEY2_DOUBLE_PRESS:
	{
		MotorRun();				   // 启动PWM
		Motor.CtrlMode = CUR_LOOP; // 有感，电流闭环
	}
	break;

	case KEY2_LONG_PRESS:
	{
		MotorRun();				   // 启动PWM
		Motor.CtrlMode = POS_LOOP; // 有感，位置闭环
	}
	break;

	case KEY3_SHORT_PRESS:
	{
		MotorStop();
		Motor.CtrlMode = STOP;
	}
	break;

	case KEY3_DOUBLE_PRESS:
	{
		MotorRun();				  // 启动PWM
		Motor.CtrlMode = IF_FLUX; // 低速IF，高速FLUX
	}
	break;

	case KEY3_LONG_PRESS:
	{
		Motor.CtrlMode = TO_ZERO;
		PostionToZeroDouble(); // 二次调零
		Motor.CtrlMode = STOP;
	}
	break;

	default:
		break;
	}

	if (Motor.CtrlMode == POS_LOOP)
	{
		pi_spd.Kp = Para.SpdSenKp;
		pi_spd.Ki = Para.SpdSenKi;
	}
	else
	{
		pi_spd.Kp = Para.SpdLessKp;
		pi_spd.Ki = Para.SpdLessKi;
	}
}
