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

#ifndef STOCKPILE_H
#define STOCKPILE_H

#include <stdint.h>

#define Stockpile_Versions	0x00000003

#ifdef __cplusplus
extern "C" {
#endif

//CTA
void CTA_Empty(void);
void CTA_Start(void);
void CTA_End(void);
void CTA_Write(uint32_t data);
	
bool Store_Flash_To_RAM(bool updata);
bool Store_RAM_To_Flash(bool updata);
	
#ifdef __cplusplus
}
#endif

#endif


