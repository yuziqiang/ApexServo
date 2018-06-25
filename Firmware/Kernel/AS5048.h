/******
	************************************************************************
	******
	** @file		: AS5048.cpp/h
	** @brief		: AS5048编码器
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
	** 	1.编码器数据读写
	** 	2.Encoder data read and write
	************************************************************************
	************************************************************************
******/

#ifndef AS5048_H
#define AS5048_H

#include <stdint.h>

#include "stm32f1xx_hal.h"

#define AS5048_DPI	((int32_t)0x4000)

//
//	AS5048_Config_TypeDef
//
typedef struct{
	SPI_HandleTypeDef *_spi;
	GPIO_TypeDef      *_cs_port;
	uint16_t          _cs_pin;
}AS5048_Config_TypeDef;

//
//	AS5048 Class
//
class AS5048
{
public:
	void Config(AS5048_Config_TypeDef *config);					//编码器配置
	bool ReadEncoderData(uint16_t *data);						//读取编码器数据
	bool ReadSmoothingEncoderData(uint16_t *data);				//读取编码器数据(多读)
	bool ReadAddress(uint16_t addr, uint16_t *data);			//读取地址数据
private:
	SPI_HandleTypeDef	*spi;
	GPIO_TypeDef		*cs_port;
	uint16_t			cs_pin;
};

#endif

