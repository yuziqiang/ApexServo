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

#include "Button.h"

Button::Button(GPIO_TypeDef *_Port, uint16_t _Pin, GPIO_PinState _Press_Level, uint16_t _Long_Time)
{
	port = _Port;
	pin = _Pin;
	press_level = _Press_Level;
	long_time = _Long_Time;
	
	state = Key_bit_Up;
	time = 0;
	level = _Press_Level;
}

void Button::Gather(uint16_t interval_ms)
{
	bool press;

	level = HAL_GPIO_ReadPin(port, pin);
	if(level == press_level)
		press = true;
	else
		press = false;

	switch(state)
	{
		case Key_bit_Up:
			if(press){
				state = Key_bit_Press_;
				time = 0;
			}
		break;
		case Key_bit_Press_:
			if(press){
				time += interval_ms;
				if(time > Suitable_Time_)
				{
					state = Key_bit_Drop;
					time = 0;
				}
			}
			else{
				state = Key_bit_Up;
				time = 0;
			}
		break;
		case Key_bit_Press:
			if(press){
				time += interval_ms;
				if(time > long_time)
				{
					state = Key_bit_Long;
					time = 0;
				}
			}
			else{
				state = Key_bit_Rise;
				time = 0;
			}
		break;
		case Key_bit_Long:
			if(press){
				state = Key_bit_Long;
			}
			else{
				state = Key_bit_LongRise;
			}
		break;
		case Key_bit_Drop:
			if(press){
				state = Key_bit_Press;
				time = 0;
			}
			else{
				state = Key_bit_Up;
			}
		break;
		case Key_bit_Rise:
			state = Key_bit_Up;
		break;
		case Key_bit_LongRise:
			state = Key_bit_Up;
		break;
	}
}

