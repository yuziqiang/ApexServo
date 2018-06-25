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

#include "AS5048.h"

//SPI Communication Command Package
//Every command sent to the AS5048A is represented with the following layout
//PAR - Parity bit (EVEN)
//RWn - Indicates read(1) or write(0) command

//Read Package (Value Read from AS5048A)
//The read frame always contains two alarm bits, the parity and error flags and the addressed data of the previous read command
//PAR - Parity bit (EVEN)
//EF  - Error flag indicating a transmission error in a previous host transmission

//Write Data Package (Value Written to AS5048A)
//The write frame is compatible to the read frame and contains two additional bits, parity flag and R flag.
//If the previous command was a write command a second package has to be transmitted.
//PAR - Parity bit (EVEN)
//R   - Has to be 0

#define AS5048_SPI_NOP			(0x0000)
#define AS5048_SPI_Error		(0x0001)
#define AS5048_SPI_Program		(0x0003)
#define AS5048_SPI_OTPZeroH		(0x0016)
#define AS5048_SPI_OTPZeroL		(0x0017)
#define AS5048_SPI_AGC			(0x3FFD)
#define AS5048_SPI_Magnitude	(0x3FFE)
#define AS5048_SPI_Angle		(0x3FFF)


#define gpio_h(GPIOx, GPIO_Pin)		GPIOx->BSRR = GPIO_Pin						// The migration needs to be changed.
#define gpio_l(GPIOx, GPIO_Pin)		GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U		// The migration needs to be changed.
#define cs_h()	gpio_h(cs_port, cs_pin)
#define cs_l()	gpio_l(cs_port, cs_pin)


//
//	奇偶校验
//
static uint16_t getParity(uint16_t data)
{
	uint16_t i,bits;

	data=data & 0x7FFF;
	bits = 0;
	for(i=0; i<16; i++)
	{
		bits += (data & 0x0001);
		data = data >> 1;
	}
	bits = bits & 0x0001;
	return bits;
}

//
//	编码器配置
//
void AS5048::Config(AS5048_Config_TypeDef *config)
{
	spi = config->_spi;
	cs_port = config->_cs_port;
	cs_pin = config->_cs_pin;
}

//
//	读取编码器数据
//
bool AS5048::ReadEncoderData(uint16_t *data)
{
	 return ReadAddress(AS5048_SPI_Angle, data);
}

//
//	读取编码器数据(多读)
//
bool AS5048::ReadSmoothingEncoderData(uint16_t *data)
{
	uint32_t sum;
	uint16_t read_data;
	uint16_t i;

	sum = 0;
	for(i = 0; i < 64 ; i++)
	{
		if(!ReadAddress(AS5048_SPI_Angle, &read_data))
			return false;
		sum += read_data;
	}
	*data = sum / i;
	
	return true;
}

//
//	读取地址数据
//
bool AS5048::ReadAddress(uint16_t addr, uint16_t *data)
{
	uint16_t read_data;
	uint16_t temp_data;
	HAL_StatusTypeDef flag;

	//写EF
	addr = addr | (0x4000);
	//写PAR
	addr = addr & (0x7FFF);
	if(getParity(addr))
		addr = addr | (0x8000);

	cs_l();
	flag = HAL_SPI_TransmitReceive(spi, (uint8_t*)&addr, (uint8_t*)&temp_data, 1, 1000000);
	cs_h();
	if(HAL_OK != flag)		return false;

	for(uint16_t i=10; i>0 ;i--){};
		
	temp_data = 0;
	cs_l();
	flag = HAL_SPI_TransmitReceive(spi, (uint8_t*)&temp_data, (uint8_t*)&read_data, 1, 1000000);
	cs_h();
	if(HAL_OK != flag)		return false;

	//检查R
	if(read_data & (0x4000))
		return false;
	//检查PAR
	if((read_data>>15) != getParity(read_data))
		return false;

	read_data = read_data & 0x3FFF;
	*data = read_data;
	return true;
}


