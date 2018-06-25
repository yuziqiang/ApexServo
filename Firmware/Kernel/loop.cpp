/******
	************************************************************************
	******
	** @file		: loop.cpp/h
	** @brief		: 主程序环
	************************************************************************
	******
	** @versions	: 1.0.0
	** @time		: 2018/5/20
	************************************************************************
	******
	** @authors		: unli (China WuHu)
	************************************************************************
	******
	** @explain		:
	** 	1.主程序环
	** 	2.The main program loop
	************************************************************************
	************************************************************************
******/

#include "loop.h"

//stm32
#include "main.h"
#include "stm32f1xx_hal.h"
//Interaction
#include "Button.h"
#include "usart_txrx.h"
#include "stockpile.h"
#include "IS.h"
//Kernel
#include "controller.h"
#include "AS5048.h"

//时基
static uint32_t time_100us = 0;
static uint32_t time_1ms = 0;
static uint32_t time_10ms = 0;
static uint32_t time_20ms = 0;
static uint32_t time_50ms = 0;
static uint32_t time_100ms = 0;
static uint32_t time_200ms = 0;
static uint32_t time_500ms = 0;
static uint32_t time_1s = 0;
//变量
static char usart_it_data;

//HAL 类定义
extern DAC_HandleTypeDef hdac;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart3;
extern PCD_HandleTypeDef hpcd_USB_FS;

//
//	类
//
Button Key1(KEY1_GPIO_Port, KEY1_Pin, GPIO_PIN_RESET, 500);
Button Key2(KEY2_GPIO_Port, KEY2_Pin, GPIO_PIN_RESET, 500);
Button Key3(KEY3_GPIO_Port, KEY3_Pin, GPIO_PIN_RESET, 500);
AS5048 encode;
Controller control;


void time_1ms_serve(void);
void time_10ms_serve(void);
void time_20ms_serve(void);
void time_50ms_serve(void);
void time_100ms_serve(void);
void time_200ms_serve(void);
void time_500ms_serve(void);
void time_1s_serve(void);


void Loop_Init()
{	
	//编码器配置
	AS5048_Config_TypeDef as5048_config;
	as5048_config._spi = &hspi3;
	as5048_config._cs_port = AS5048A_CS_GPIO_Port;
	as5048_config._cs_pin = AS5048A_CS_Pin;
	encode.Config(&as5048_config);
	
	//控制器配置
	Controller_Config_TypeDef controller_config;
	controller_config._dac = &hdac;
	controller_config._encode = &encode;
	controller_config._in1_port = A4954_IN1_GPIO_Port;
	controller_config._in1_pin  = A4954_IN1_Pin;
	controller_config._in2_port = A4954_IN2_GPIO_Port;
	controller_config._in2_pin  = A4954_IN2_Pin;
	controller_config._in3_port = A4954_IN3_GPIO_Port;
	controller_config._in3_pin  = A4954_IN3_Pin;
	controller_config._in4_port = A4954_IN4_GPIO_Port;
	controller_config._in4_pin  = A4954_IN4_Pin;
	control.Config(&controller_config);

	//控制器启动
	control.Start();
	
	//卡尔曼滤波初始化
//	kalman_init();
	
	//DAC初始化
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	
	//LED初始化
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

}

void loop()
{
	//读取Flash数据并使用
	Store_Flash_To_RAM(true);

	//初始化	
	Loop_Init();

	//使能中断
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_UART_Receive_IT(&huart3, (uint8_t*)&usart_it_data, 1);
	
	while (1)
	{
		if(time_1ms){time_1ms--;time_1ms_serve();}
		if(time_10ms){time_10ms--;time_10ms_serve();}
		if(time_20ms){time_20ms--;time_20ms_serve();}
		if(time_50ms){time_50ms--;time_50ms_serve();}
		if(time_100ms){time_100ms--;time_100ms_serve();}
		if(time_200ms){time_200ms--;time_200ms_serve();}
		if(time_500ms){time_500ms--;time_500ms_serve();}
		if(time_1s){time_1s--;time_1s_serve();}
		if(control.up_st)
		{
			control.up_st = false;
			HAL_TIM_Base_Stop_IT(&htim1);
			if(control.Standardizing())
				SerialPort3_Send_string((char*)"Standardizing_OK!!!\r\n");
			else
				SerialPort3_Send_string((char*)"Standardizing_ERROR!!!\r\n");
			HAL_TIM_Base_Start_IT(&htim1);
		}
	}
}

//
//	定时器1中断响应函数	10k
//
void TIM1_UP_IRQHandler(void)
{
	static uint16_t time5000 = 0;
	
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
	
	___TEST1_H;
	
	control.Callback();
	
	if(++time5000 > 4999)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		//HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		time5000 = 0;
	}
	
	time_100us++;
	if(!(time_100us % 10))
	{
		time_1ms++;
		if(!(time_100us % 100))
		{
			time_10ms++;
			if(!(time_100us % 200))
				time_20ms++;
			if(!(time_100us % 500))
			{
				time_50ms++;
				if(!(time_100us % 1000))
				{
					time_100ms++;
					if(!(time_100us % 2000))
						time_200ms++;
					if(!(time_100us % 5000))
					{
						time_500ms++;
						if(!(time_100us % 10000))
						{
							time_1s++;
						}
					}
				}
			}
		}
	}
	___TEST1_L;
}

//
//	中断9_5
//
void EXTI9_5_IRQHandler(void)
{
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
		//HAL_GPIO_EXTI_Callback(GPIO_PIN_6);
	}
	
	if ((DIR_GPIO_Port->IDR & DIR_Pin) != (uint32_t)GPIO_PIN_RESET)
	{
		//bitstatus = GPIO_PIN_SET;
		if(control.dir_pm)
			control.step_count++;
		else 
			control.step_count--;
	}
	else
	{
		//bitstatus = GPIO_PIN_RESET;
		if(control.dir_pm)
			control.step_count--;
		else
			control.step_count++;
	}
}

//
//
//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		Receive_Char(usart_it_data);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		HAL_UART_Receive_IT(&huart3, (uint8_t*)&usart_it_data, 1);
	}
}

//
//普通计时服务
//
void time_1ms_serve(void)
{

}

void time_10ms_serve(void)
{
	IS_Discern();	//IS指令扫描
}

void time_20ms_serve(void)
{

}

void time_50ms_serve(void)
{

}

void time_100ms_serve(void)
{

}

void time_200ms_serve(void)
{

}

void time_500ms_serve(void)
{

}

void time_1s_serve(void)
{

}
