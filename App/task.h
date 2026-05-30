/*
***********************************************************************
 * 文件名：task.h
 * 作  者：???
 * 日  期：2024/05/16
 * 说  明：程序任务
***********************************************************************
*/

#ifndef __TASK_H__
#define __TASK_H__

#include "main.h"
#include <stdlib.h>
#include "delay.h"
#include "stm32f4xx_it.h"
#include "hmi_app.h"
#include "led_drv.h"
#include "tim_drv.h"
#include "adc_drv.h"
#include "key_drv.h"
#include "motor_ctrl.h"

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

void task_init(void);
void task_loop(void);

extern volatile uint16_t AccMotionTim;

#endif
