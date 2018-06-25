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

#ifndef LOOP_H
#define LOOP_H

#include <stdint.h>

//引用的实例
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
 extern "C" {
#endif

void loop(void);
	 
void TIM1_UP_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
//void TIM2_IRQHandler(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
	 
#ifdef __cplusplus
}
#endif

#endif	//LOOP_H
