/*
***********************************************************************
 * 文件名：led_drv.c
 * 作  者：？？？
 * 日  期：2024/05/09
 * 说  明：LED驱动
***********************************************************************
*/

#include "led_drv.h"

typedef struct
{
	GPIO_TypeDef *gpio;
	uint32_t pin;
} Led_GPIO_t;

static Led_GPIO_t g_gpioList[] =
{
	{LED_B_GPIO_Port, LED_B_Pin},
    {LED_G_GPIO_Port, LED_G_Pin},
    {LED_R_GPIO_Port, LED_R_Pin}
};

#define LED_NUM_MAX (sizeof(g_gpioList) / sizeof(g_gpioList[0]))

/**
***********************************************************
* @brief 点亮LED
* @param ledNo，LED标号，0~2
* @return 
***********************************************************
*/
void TurnOnLed(uint8_t ledNo)
{
	if (ledNo >= LED_NUM_MAX)
	{
		return;
	}
	HAL_GPIO_WritePin(g_gpioList[ledNo].gpio, g_gpioList[ledNo].pin, GPIO_PIN_RESET);
}

/**
***********************************************************
* @brief 熄灭LED
* @param ledNo，LED标号，0~2
* @return 
***********************************************************
*/
void TurnOffLed(uint8_t ledNo)
{
	if (ledNo >= LED_NUM_MAX)
	{
		return;
	}
	HAL_GPIO_WritePin(g_gpioList[ledNo].gpio, g_gpioList[ledNo].pin, GPIO_PIN_SET);
}

/**
***********************************************************
* @brief 翻转LED
* @param ledNo，LED标号，0~2
* @return 
***********************************************************
*/
void ToggleLed(uint8_t ledNo)
{
	if (ledNo >= LED_NUM_MAX)
	{
		return;
	}
	HAL_GPIO_TogglePin(g_gpioList[ledNo].gpio, g_gpioList[ledNo].pin);
}




