/*
***********************************************************************
 * 文件名：key_drv.c
 * 作  者：???
 * 日  期：2024/09/26
 * 说  明：按键驱动
***********************************************************************
*/
#include "key_drv.h"

typedef struct
{
	GPIO_TypeDef *gpio;
	uint32_t pin;
} Key_GPIO_t;

static Key_GPIO_t g_gpioList[] = 
{
	{GPIOB, GPIO_PIN_12},   // key1
	{GPIOB, GPIO_PIN_13},   // key2
	{GPIOB, GPIO_PIN_14},   // key3
    {GPIOB, GPIO_PIN_11},   // ONOFF
};

#define KEY_NUM_MAX (sizeof(g_gpioList) / sizeof(g_gpioList[0]))

typedef enum
{
	KEY_RELEASE = 0,         // 释放松开
	KEY_CONFIRM,             // 消抖确认
	KEY_SHORTPRESS,          // 短按
	KEY_LONGPRESS            // 长按
} KEY_STATE;

#define CONFIRM_TIME                 10       // 按键消抖时间窗10ms
#define DOUBLE_CLICK_INTERVAL_TIME   300      // 两次抬起时间窗300ms，用来判断是否双击
#define LONGPRESS_TIME               2000     // 长按时间窗2000ms

typedef struct
{
	KEY_STATE keyState;
	uint8_t singleClickNum;
	uint64_t firstIoChangeSysTime;
	uint64_t firstReleaseSysTime;
} Key_Info_t;

static Key_Info_t g_keyInfo[KEY_NUM_MAX];

/**
***********************************************************
* @brief 按键硬件初始化
* @param
* @return 
***********************************************************
*/
void KeyDrvInit(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitTypeDef gpioInitStruct;
	gpioInitStruct.Mode = GPIO_MODE_INPUT;
	gpioInitStruct.Pull = GPIO_PULLUP;
	
	for (uint8_t i = 0; i < KEY_NUM_MAX; i++)
	{
		gpioInitStruct.Pin = g_gpioList[i].pin;
		HAL_GPIO_Init(g_gpioList[i].gpio, &gpioInitStruct);
	}
}

static uint8_t KeyScan(uint8_t keyIndex)
{
	uint64_t curSysTime;
	uint8_t keyPress;
	
	keyPress = HAL_GPIO_ReadPin(g_gpioList[keyIndex].gpio, g_gpioList[keyIndex].pin);

	switch (g_keyInfo[keyIndex].keyState)
	{
		case KEY_RELEASE:                                          //  释放状态：判断有无按键按下
			if (!keyPress)                                         //  有按键按下
			{ 
				g_keyInfo[keyIndex].keyState = KEY_CONFIRM;        //  获取系统运行时间
				g_keyInfo[keyIndex].firstIoChangeSysTime = HAL_GetTick();
				break;
			}

			if (g_keyInfo[keyIndex].singleClickNum != 0)
			{
				curSysTime = HAL_GetTick();
				if (curSysTime - g_keyInfo[keyIndex].firstReleaseSysTime > DOUBLE_CLICK_INTERVAL_TIME)
				{
					g_keyInfo[keyIndex].singleClickNum = 0;
					return (keyIndex + 1); //  返回单击按键码值，四个按键短按对应0x01, 0x02, 0x03, 0x04
				}
			}

			break;
			
		case KEY_CONFIRM:
			if (!keyPress)
			{
				curSysTime = HAL_GetTick();
				if (curSysTime - g_keyInfo[keyIndex].firstIoChangeSysTime > CONFIRM_TIME) //  超过时间窗，切换到短按
				{
					g_keyInfo[keyIndex].keyState = KEY_SHORTPRESS;
				}
			}
			else
			{
				g_keyInfo[keyIndex].keyState = KEY_RELEASE;
			}
			break;
			
		case KEY_SHORTPRESS:           //  短按状态：继续判定是短按，还是双击，还是长按
			if (keyPress)
			{
				g_keyInfo[keyIndex].keyState = KEY_RELEASE;
				
				g_keyInfo[keyIndex].singleClickNum++;
				if (g_keyInfo[keyIndex].singleClickNum == 1)
				{
					g_keyInfo[keyIndex].firstReleaseSysTime = HAL_GetTick();
					break;
				}
				else
				{
					curSysTime = HAL_GetTick();
					if (curSysTime - g_keyInfo[keyIndex].firstReleaseSysTime <= DOUBLE_CLICK_INTERVAL_TIME) //  超过双击时间间隔窗，返回双击码值
					{
						g_keyInfo[keyIndex].singleClickNum = 0;
						return (keyIndex + 0x51); //  返回按键码值，四个按键双击对应0x51, 0x52, 0x53, 0x54
					}
				}
			}
			else
			{
				curSysTime = HAL_GetTick();
				if (curSysTime - g_keyInfo[keyIndex].firstIoChangeSysTime > LONGPRESS_TIME)
				{	
					g_keyInfo[keyIndex].keyState = KEY_LONGPRESS;
				}
			}
			break;
		case KEY_LONGPRESS:
			if (keyPress)
			{
				g_keyInfo[keyIndex].keyState = KEY_RELEASE;
				return (keyIndex + 0x81); //返回按键码值，四个按键长按对应0x81, 0x82, 0x83, 0x84
			}
			break;
		default:
			g_keyInfo[keyIndex].keyState = KEY_RELEASE;
			break;
	}
	return 0;
}

/**
***********************************************************
* @brief 获取按键码值
* @param
* @return 四个按键码值，短按0x01 0x02 0x03 0x04, 双击0x51 
          0x52 0x53 0x54, 长按0x81 0x82 0x83 0x84
***********************************************************
*/
uint8_t GetKeyVal(void)
{
	uint8_t res = 0;

	for (uint8_t i = 0; i < KEY_NUM_MAX; i++)
	{
		res = KeyScan(i);
		if (res != 0)
		{
			break;
		}
	}
	return res;
}
