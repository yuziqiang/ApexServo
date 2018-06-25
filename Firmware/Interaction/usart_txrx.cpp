/******
	************************************************************************
	******
	** @file		: usart_txrt.cpp/h
	** @brief		: 串口发送接收
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
	** 	1.串口发送接收
	** 	2.Serial port sending and receiving
	************************************************************************
	************************************************************************
******/


#include "usart_txrx.h"

// //C语言基本库
// #include <stdio.h>		//基本输入输出
// #include <stdlib.h>		//基础库
// #include <string.h>		//字符串

//调用的发送端口
#include "stm32f1xx_hal.h"
//调用IS主程序
#include "IS.h"


/*********************************************************************/
/****************************    发送    ******************************/
/*********************************************************************/

//串口缓冲区
uint16_t uart3_tx_size;
char uart3_tx_buff[129];
char uart3_rx_buff[129];

//
//
//
void SerialPort3_Send_0x32(uint32_t data)
{
	sprintf(uart3_tx_buff, "0x%4x ;", data);
	uart3_tx_size = strlen((char*)uart3_tx_buff);
	while(huart3.gState != HAL_UART_STATE_READY);//等待DMA
	//HAL_UART_Transmit(&huart3, (uint8_t*)uart3_tx_buff, uart3_tx_size, 1000000);
	HAL_UART_Transmit_DMA(&huart3, (uint8_t*)uart3_tx_buff, uart3_tx_size);
}

//
//
//
void SerialPort3_Send_float(float data)
{
	sprintf(uart3_tx_buff, "%3.6f ;", data);
	uart3_tx_size = strlen((char*)uart3_tx_buff);
	while(huart3.gState != HAL_UART_STATE_READY);//等待DMA
	//HAL_UART_Transmit(&huart3, (uint8_t*)uart3_tx_buff, uart3_tx_size, 1000000);
	HAL_UART_Transmit_DMA(&huart3, (uint8_t*)uart3_tx_buff, uart3_tx_size);
}

//
//
//
void SerialPort3_Send_string(char *spr)
{
	uart3_tx_size = strlen((char*)spr);
	strcpy(uart3_tx_buff, spr);
	while(huart3.gState != HAL_UART_STATE_READY);//等待DMA
	//HAL_UART_Transmit(&huart3, (uint8_t*)uart3_tx_buff, uart3_tx_size, 1000000);
	HAL_UART_Transmit_DMA(&huart3, (uint8_t*)uart3_tx_buff, uart3_tx_size);
}

void SerialPort3_Send_TX(void)
{
	uart3_tx_size = strlen((char*)uart3_tx_buff);
	while(huart3.gState != HAL_UART_STATE_READY);//等待DMA
	//HAL_UART_Transmit(&huart3, (uint8_t*)uart3_tx_buff, uart3_tx_size, 1000000);
	HAL_UART_Transmit_DMA(&huart3, (uint8_t*)uart3_tx_buff, uart3_tx_size);
}

bool SerialPort3_Refer_TX_Leisure(void)
{
	if(huart3.gState == HAL_UART_STATE_READY)
		return true;
	else
		return false;
}


/*********************************************************************/
/****************************    接收    ******************************/
/*********************************************************************/

State_Typedef receive_state = State_New;					//接收状态
uint16_t data_count = 0;									//已接收的数据量
char receive_buff[Standard_USART_Receive_Buff_Max];			//接收Buff
	
/*
**	接收一个字节(接收\r\n结尾)
*/
void Receive_Char(char data)
{
	receive_buff[data_count] = data;
	data_count++;

/********** 使用R&N结尾 **********/
/********** 使用R&N结尾 **********/
/********** 使用R&N结尾 **********/
#if (Receive_End == Receive_End_RandN)
	switch(receive_state)
	{
	case State_New:
		if((data == 0x0d) || (data == 0x0a)){
			receive_state = State_New;
			data_count = 0;
		}
		else{
			receive_state = State_Receive;
		}
	break;
	case State_Receive:
		if(data == 0x0d){
			receive_state = State_End_0x0A;
		}
		else if(data == 0x0a){
			receive_state = State_Error_0x0D;
			data_count = 0;
		}
		else if(data_count > (Standard_USART_Receive_Buff_Max-3)){
			receive_state = State_Error_0x0D;
			data_count = 0;
		}
		break;
	case State_End_0x0A:
		if(data == 0x0a){
			receive_buff[data_count] = 0x00;
			data_count++;
			Receive_Write_Loop();
			receive_state = State_New;
			data_count = 0;
		}
		else{
			receive_state = State_Error_0x0D;
			data_count = 0;
		}
		break;
	case State_Error_0x0D:
		if(data == 0x0d){
			receive_state = State_Error_0x0A;
		}
		else{
			receive_state = State_Error_0x0D;
			data_count = 0;
		}
		break;
	case State_Error_0x0A:
		if(data == 0x0a){
			receive_state = State_New;
			data_count = 0;
		}
		else{
			receive_state = State_Error_0x0D;
			data_count = 0;
		}
		break;
	}
/********** 使用R/N结尾 **********/
/********** 使用R/N结尾 **********/
/********** 使用R/N结尾 **********/
#elif (Receive_End == Receive_End_RorN)
	switch(receive_state)
	{
	case State_New:
		if((data == 0x0d) || (data == 0x0a)){
			receive_state = State_New;
			data_count = 0;
		}
		else{
			receive_state = State_Receive;
		}
	break;
	case State_Receive:
		if((data == 0x0d) || (data == 0x0a)){
			receive_buff[data_count] = 0x00;
			data_count++;
			Receive_Write_Loop();
			receive_state = State_New;
			data_count = 0;
		}
		else if(data_count > (Standard_USART_Receive_Buff_Max-3)){
			receive_state = State_Error;
			data_count = 0;
		}
		break;
	case State_Error:
		if((data == 0x0d) || (data == 0x0a)){
			receive_state = State_New;
			data_count = 0;
		}
		else{
			receive_state = State_Error;
			data_count = 0;
		}
		break;
	}
#endif
}

/*
**	接收一个字符串
*/
void Receive_String(char *data, uint32_t number)
{
	uint16_t count;
	for(count = 0; count < number; count++)
	{
		Receive_Char(data[count]);
	}
}

/*
**	写环函数
*/
void Receive_Write_Loop(void)
{
	IS_Write_Loop(receive_buff);
}

