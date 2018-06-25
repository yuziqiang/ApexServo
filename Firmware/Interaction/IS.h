/******
	************************************************************************
	******
	** @file		: IS.cpp/h
	** @brief		: 指令识别
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
	** 	1.识别接收数据包含的指令
	** 	2.Identify the instructions contained in the receiving data
	************************************************************************
	************************************************************************
******/

#ifndef IS_H
#define IS_H

#include <stdint.h>		//数据标准规范

//C语言基本库
#include <stdio.h>           //基本输入输出
#include <stdlib.h>        //基础库
#include <string.h>        //字符串

#ifdef __cplusplus
extern "C" {
#endif

void IS_IN(char *str);				//IS指令系统输入	推荐不要直接使用
void IS_Discern(void);				//IS扫描
bool IS_Write_Loop(char *data);
bool IS_Read_Loop(char *data);

#ifdef __cplusplus
}
#endif

#endif
