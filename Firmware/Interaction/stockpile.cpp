/******
	************************************************************************
	******
	** @file		: stockpile.cpp/h
	** @brief		: Flash存储控制
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
	** 	1.根据使用的芯片对Flash区域进行读写控制
	** 	2.Read and write control over Flash area according to the chip used
	************************************************************************
	************************************************************************
******/

/* Includes ------------------------------------------------------------------*/
#include "stockpile.h"

/********** IOC **********/
#include "main.h"
#include "stm32f1xx_hal.h"

/********** 特殊 **********/
#include <stdio.h>				//基本输入输出
#include <stdlib.h>				//通用工具
#include <string.h>				//字符数组
/********** math **********/
#include "arm_math.h"			//ARM_Math


/* 储存的数据声明 ------------------------------------------------------------------*/
#include "controller.h"
extern Controller control;
/* 储存的数据声明 ------------------------------------------------------------------*/

/*********************STM32F4xx_xE*************************/
////////////Flash
//扇区格式	16k + 16k + 16k + 16k + 64k + 128k + 128k + 128k
//ADDR		0x08000000 ~ 0x0807FFFF		0x00080000		512k
//Boot		0x08000000 ~ 0x0800BFFF		0x0000C000		48k
//Boot存储	0x0800C000 ~ 0x0800FFFF		0x00004000		16k
//程序使用	0x08010000 ~ 0x0803FFFF		0x00030000		192k
//Flash储存	0x08040000 ~ 0x0807FFFF		0x00040000		256k
////////////RAM
//ADDR		0x20000000 ~ 0x2001FFFF		0x00020000		128k
//程序使用	0x20000000 ~ 0x2001FFFF		0x00020000		128k
////////////CCM
//ADDR		0x10000000 ~ 0x1000FFFF		0x00010000		64k
//程序使用	0x10000000 ~ 0x1000FFFF		0x00010000		64k

/*********************STM32F1xx_xC*************************/
////////////Flash
//扇区格式	2K + 2K + 2K + 2K + ... ... + 2K + 2K + 2K + 2K
//ADDR		0x08000000 ~ 0x0803FFFF		0x00040000		256k
//程序使用	0x08000000 ~ 0x0800FFFF		0x00010000		64k
//Flash储存	0x08010000 ~ 0x0801FFFF		0x00010000		64k
//矫正储存	0x08020000 ~ 0x0803FFFF		0x00020000		128k
////////////RAM
//ADDR		0x20000000 ~ 0x2001FFFF		0x00010000		64k
//程序使用	0x20000000 ~ 0x2001FFFF		0x00010000		64k



/******************页段配置(更换芯片必须修改这个配置)***********************/
#define		Buff_Size			64U									//储存的数据量(4字节)
#define		FLASH_SAVE_ADDR		0x08010000							//首扇区地址
#define		FLASH_SAVE_SIZE		0x00010000							//用于存储的Flash大小
#define		Page_Size			0x00000800							//扇区大小(硬定义)
#define		Page_Quantity		(FLASH_SAVE_SIZE / Page_Size)		//扇区数(硬定义)
#define		Semgment_Size		(4*Buff_Size)						//段大小(软定义)
#define		Semgment_Quantity	(FLASH_SAVE_SIZE / Semgment_Size)	//段数(软定义)
#define		FLASH_CTA_ADDR		0x08020000							//CTA首扇区地址
#define		FLASH_CTA_SIZE		0x00020000							//CTA的Flash大小
#define		CTA_Page_Quantity	(FLASH_CTA_SIZE / Page_Size)		//CTA扇区数(硬定义)

//当前使用的段号
uint32_t	pitch_segment;

//Buff
uint32_t	buff_data[Buff_Size];						//读写缓冲区
extern void FLASH_PageErase(uint32_t PageAddress);		//库FLASH清除函数

/**********************************************************************/
/**********************************************************************/
uint32_t cta_write_add;

void CTA_Empty(void)
{
	uint16_t count;
	HAL_FLASH_Unlock();
	for(count = 0; count < CTA_Page_Quantity; count++)
	{
		FLASH_PageErase(FLASH_CTA_ADDR + (count * Page_Size));	
		FLASH_WaitForLastOperation(50000);
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}
	HAL_FLASH_Lock();
}

void CTA_Start()
{
	HAL_FLASH_Unlock();
	cta_write_add = FLASH_CTA_ADDR;
}

void CTA_End(void)
{
	HAL_FLASH_Lock();
}

void CTA_Write(uint32_t data)
{
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, cta_write_add, data) != HAL_OK)
	{
		_Error_Handler((char*)__FILE__, __LINE__);
	}
	cta_write_add += 4;
}
/**********************************************************************/
/**********************************************************************/

/***********************************
**	Read_Flash
**	ReadSemgment:	读段
** 	*pBuff:			数据指针
**	NumToRead:		读取数量
***********************************/
void Read_Flash(uint16_t ReadSemgment, uint32_t *pBuff, uint16_t NumToRead)
{
	uint32_t ReadAddr;
	uint16_t count;

	ReadAddr = FLASH_SAVE_ADDR + (ReadSemgment * Semgment_Size);

	for(count = 0; count < NumToRead; count++)
	{
		pBuff[count] = *(uint32_t*)(ReadAddr);
		ReadAddr += 4;
	}	
}

/***********************************
**	Write_Flash
**	WritSemgment:	写段
** 	*pBuff:			数据指针
**	NumToWrite:		读取数量
***********************************/
void Write_Flash(uint16_t WritSemgment, uint32_t *pBuff, uint16_t NumToWrite)
{
	uint32_t WriteAddr;
	uint16_t count;

	WriteAddr = FLASH_SAVE_ADDR + (WritSemgment * Semgment_Size);

	HAL_FLASH_Unlock();
	for(count = 0; count < NumToWrite; count++)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr, pBuff[count]) != HAL_OK)
		{
			_Error_Handler((char*)__FILE__, __LINE__);
		}
		WriteAddr += 4;
	}
	HAL_FLASH_Lock();
}

/***********************************
**	清空整个Flash储存区
***********************************/
void Empty_Flash(void)
{
//	STM32F4大页操作方法
//	FLASH_EraseInitTypeDef FlashEraseInit;
//	uint32_t SectorError=0;
//
//	HAL_FLASH_Unlock();
//
//	FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
//	FlashEraseInit.Sector = FLASH_SECTOR_6;
//	FlashEraseInit.NbSectors=1;
//	FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;
//	if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError) != HAL_OK) 
//	{
//		_Error_Handler((char*)__FILE__, __LINE__);
//	}
//	FLASH_WaitForLastOperation(50000);
//	FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
//	FlashEraseInit.Sector = FLASH_SECTOR_7;
//	FlashEraseInit.NbSectors=1;
//	FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;
//	if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError) != HAL_OK) 
//	{
//		_Error_Handler((char*)__FILE__, __LINE__);
//	}
//	FLASH_WaitForLastOperation(50000);
//
//	HAL_FLASH_Lock();	

//	STM32F1小页操作方法
	uint16_t count;
	HAL_FLASH_Unlock();
	for(count = 0; count < Page_Quantity; count++)
	{
		FLASH_PageErase(FLASH_SAVE_ADDR + (count * Page_Size));	
		FLASH_WaitForLastOperation(50000);
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}
	HAL_FLASH_Lock();
}

/***********************************
**	从Flash读出参数写入RAM
***********************************/
bool Store_Flash_To_RAM(bool updata)
{
	uint32_t count;
	uint32_t verify;		//校验

	//社区版中直接使用0段,均衡擦写在商业版中完整编写(可自行编辑与调试)
	pitch_segment = 0;
	
	//读出Flash
	Read_Flash(pitch_segment, buff_data, Buff_Size);

	//校验
	verify = 0;
	for(count = 1; count < Buff_Size; count++)
	{
		verify += buff_data[count];
	}
	count = 0;
	//校验数据
	if(	(buff_data[count++] != verify)						//数据校验
	||	(buff_data[count++] != pitch_segment)				//Flash段号校验
	||	(buff_data[count++] != Stockpile_Versions)			//固件版本号校验
	)
	{	//校验失败说明Flash错误则清空Flash并写入一次
		Empty_Flash();
		Store_RAM_To_Flash(true);
		return false;
	}
	//读取数据
	if(updata)
	{
		uint32_t data1		= *(uint32_t*)&	buff_data[count++];		control.up_spr(data1);
		control.sdb			= *(uint32_t*)&	buff_data[count++];
		control.dir_pm		= *(bool*)&		buff_data[count++];
		control.t_pm		= *(float*)&	buff_data[count++];
		control.iu_pm		= *(float*)&	buff_data[count++];

		control.pgv_pid.Kp	= *(float*)&	buff_data[count++];
		control.pgv_pid.Ki	= *(float*)&	buff_data[count++];
		control.pgv_pid.Kd	= *(float*)&	buff_data[count++];
		control.pgv_pid.Kq	= *(float*)&	buff_data[count++];
	
		control.p_pid.Kp	= *(float*)&	buff_data[count++];
		control.p_pid.Ki	= *(float*)&	buff_data[count++];
		control.p_pid.Kd	= *(float*)&	buff_data[count++];
		control.p_pid.Kq	= *(float*)&	buff_data[count++];
	
		control.v_pid.Kp	= *(float*)&	buff_data[count++];
		control.v_pid.Ki	= *(float*)&	buff_data[count++];
		control.v_pid.Kd	= *(float*)&	buff_data[count++];
		control.v_pid.Kq	= *(float*)&	buff_data[count++];
	}

	return true;
}

/***********************************
**	从RAM读出参数写入Flash
***********************************/
bool Store_RAM_To_Flash(bool updata)
{
	uint32_t count;
	uint32_t verify;

	//社区版中直接使用0段,均衡擦写在商业版中完整编写(可自行编辑与调试)
	pitch_segment = 0;

	//
	count = 0;
	//
	buff_data[count++] = 0;						//数据校验
	buff_data[count++] = pitch_segment;			//Flash段号校验
	buff_data[count++] = Stockpile_Versions;	//固件版本号校验
	//
	buff_data[count++]	= *(uint32_t*)&		control.spr;
	buff_data[count++]	= *(uint32_t*)&		control.sdb;
	buff_data[count++]	= *(uint32_t*)&		control.dir_pm;
	buff_data[count++]	= *(uint32_t*)&		control.t_pm;
	buff_data[count++]	= *(uint32_t*)&		control.iu_pm;

	buff_data[count++]	= *(uint32_t*)&		control.pgv_pid.Kp;
	buff_data[count++]	= *(uint32_t*)&		control.pgv_pid.Ki;
	buff_data[count++]	= *(uint32_t*)&		control.pgv_pid.Kd;
	buff_data[count++]	= *(uint32_t*)&		control.pgv_pid.Kq;

	buff_data[count++]	= *(uint32_t*)&		control.p_pid.Kp;
	buff_data[count++]	= *(uint32_t*)&		control.p_pid.Ki;
	buff_data[count++]	= *(uint32_t*)&		control.p_pid.Kd;
	buff_data[count++]	= *(uint32_t*)&		control.p_pid.Kq;

	buff_data[count++]	= *(uint32_t*)&		control.v_pid.Kp;
	buff_data[count++]	= *(uint32_t*)&		control.v_pid.Ki;
	buff_data[count++]	= *(uint32_t*)&		control.v_pid.Kd;
	buff_data[count++]	= *(uint32_t*)&		control.v_pid.Kq;
	//校验
	verify = 0;
	for(count = 1; count < Buff_Size; count++)
	{
		verify += buff_data[count];
	}
	buff_data[0] = verify;

	if(updata)
	{
		//写入Flash
		Write_Flash(pitch_segment, buff_data, Buff_Size);
	}

	return true;
}






















