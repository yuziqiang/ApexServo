/******
	************************************************************************
	******
	** @file		: Button.cpp/h
	** @brief		: 按键驱动
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
	** 	1.扫描按键电平的以获得每个按键状态的
	** 	2.Scan the key level to get the status of each key
	************************************************************************
	************************************************************************
******/

#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>

#include "stm32f1xx_hal.h"

#define Suitable_Time_	20	//ms合适的去抖时间

#define Normal_Level_L	0
#define Normal_Level_H	1

enum{
	Key_bit_Up		= 0x00,	//Key状态-弹起
	Key_bit_Press_,			//Key状态-按下去抖检测
	Key_bit_Press,			//Key状态-按下
	Key_bit_Long,			//Key长按状态
	Key_bit_Drop,			//Key按下沿
	Key_bit_Rise,			//Key弹起沿
	Key_bit_LongRise,		//长按弹起沿
};

class Button
{
public:
	Button(GPIO_TypeDef *_Port, uint16_t _Pin, GPIO_PinState _Press_Level, uint16_t _Long_Time);
	void Gather(uint16_t interval_ms);
	uint8_t state;
	uint16_t time;
	GPIO_PinState level;
private:
	GPIO_TypeDef *port;
	uint16_t pin;
	GPIO_PinState press_level;
	uint16_t long_time;
};




#endif //	BUTTON_H

