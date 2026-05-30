/*
***********************************************************************
* 文件名：task.c
* 作  者：???
* 日  期：2024/05/16
* 说  明：程序任务
***********************************************************************
*/
#include "task.h"

typedef struct
{
	uint8_t run;			   // 调度标志，1：调度，0：挂起
	uint16_t timCount;		   // 时间片计数值
	uint16_t timRload;		   // 时间片重载值
	void (*pTaskFuncCb)(void); // 函数指针变量，用来保存业务功能模块函数地址
} TaskComps_t;

static TaskComps_t g_taskComps[] =
	{
#if VR_OR_PC
		{0, 1, 1, VrTask}, // 电位器处理任务
#else
		{0, 10, 10, RecevToVofa}, // 接收数据任务
#endif
		{0, 5, 5, HmiTask},		 // 按键处理任务
		//{0, 10, 10, SendToVofa}, // 发送数据任务
		{0, 1, 1, FaultCheck},	 // 故障检测任务

		/* 添加业务功能模块 */
};

#define TASK_NUM_MAX (sizeof(g_taskComps) / sizeof(g_taskComps[0]))

/**
***********************************************************
* @brief 任务调度函数
* @param
* @return
***********************************************************
*/
static void TaskHandler(void)
{
	for (uint8_t i = 0; i < TASK_NUM_MAX; i++)
	{
		if (g_taskComps[i].run) // 判断时间片标志
		{
			g_taskComps[i].run = 0; // 标志清零
			if (g_taskComps[i].pTaskFuncCb == NULL)
			{
				continue;
			}
			g_taskComps[i].pTaskFuncCb(); // 执行调度业务功能模块
		}
	}
}

/**
***********************************************************
* @brief 在定时器中断服务函数中被间接调用，设置时间片标记，
		 需要定时器1ms产生1次中断
* @param
* @return
***********************************************************
*/
static void TaskScheduleCb(void)
{
	for (uint8_t i = 0; i < TASK_NUM_MAX; i++)
	{
		if (g_taskComps[i].timCount)
		{
			g_taskComps[i].timCount--;
			if (g_taskComps[i].timCount == 0)
			{
				g_taskComps[i].run = 1;
				g_taskComps[i].timCount = g_taskComps[i].timRload;
			}
		}
	}
}

/**
***********************************************************
* @brief 驱动初始化
* @param
* @return
***********************************************************
*/
static void DrvInit()
{
	// DelayInit();
	KeyDrvInit();
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); // receive interrrupt，空闲时间开启中断
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);	 // 避免中断已开启就进入中断
	HAL_TIM_Base_Start_IT(&htim1);				 // 开启定时器1中断
}

/**
***********************************************************
* @brief 应用初始化
* @param
* @return
***********************************************************
*/
static void AppInit()
{
	TaskScheduleCbReg(TaskScheduleCb);
}

/**
***********************************************************
* @brief 初始化
* @param
* @return
***********************************************************
*/
void task_init(void)
{
	DrvInit();
	AppInit();

	MotorInit(); // 电机参数初始化

	HAL_Delay(100);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_Value, 5);		  // 开启ADC-DMA转换
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2); // 开启编码器使能函数

	AdcSampleOffset(); // 零点校准

	Init_Over = 1;

	Motor.CtrlMode = TO_ZERO;
	PostionToZeroDouble(); // 调零
	Motor.CtrlMode = STOP;
}

/**
***********************************************************
* @brief 主任务循环
* @param
* @return
***********************************************************
*/
void task_loop(void)
{
	TaskHandler(); // 任务调度
    SendToVofa(); // 发送数据任务
}
