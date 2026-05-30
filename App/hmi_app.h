/*
***********************************************************************
* 文件名：hmi_app.c
* 作  者：???
* 日  期：2024/10/07
* 说  明：人机交互任务
***********************************************************************
*/

#ifndef __HMI_APP_H__
#define __HMI_APP_H__

#include "main.h"

#define USART_REC_LEN 200 // 定义最大接收字节数 200
#define RXBUFFERSIZE 1    // 缓存大小

typedef union // 浮点数据与四字节的转换
{
    float fdata;
    unsigned long ldata;
} FloatLongType;

typedef union // 浮点数与四字节的转换
{
    float f_data;
    uint8_t c_data[4];
} FloatToChar;

extern FloatToChar fc;

typedef struct
{
    uint8_t UsartRxBuf[USART_REC_LEN]; // 接收缓冲,最大USART_REC_LEN个字节.末字节为换行符
    uint8_t aRxBuffer[RXBUFFERSIZE];   // HAL库USART接收Buffer
    uint16_t RxLine;                   // 指令长度
    uint16_t UsartRxSta;               // 接收状态标记
    uint8_t Cdata[4];
    float VofaData[13];
} USART_DATA;
extern USART_DATA Usart;

// 发送数据到Vofa
void SendToVofa(void);
// 从Vofa接收数据
void RecevToVofa(void);

/**
***********************************************************
* @brief LED应用程序
* @param
* @return
***********************************************************
*/
void LedTask(uint8_t r, uint8_t g, uint8_t b);

/**
***********************************************************
* @brief 电位器应用程序
* @param
* @return
***********************************************************
*/
void VrTask(void);

/**
***********************************************************
* @brief 按键应用程序
* @param
* @return
***********************************************************
*/
void HmiTask(void);

#endif
