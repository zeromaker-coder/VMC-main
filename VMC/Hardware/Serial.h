#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"
#include <string.h> 
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h> // 用于 isdigit 函数
#define DMA_Rec_Len1 100 
extern char Serial_RxPacket[];
extern float M0_Target;//串口接收存放
extern float M1_Target;//串口接收存放

void Serial_Init(u32 bound);
void MYDMA1_Enable(DMA_Channel_TypeDef*DMA_CHx);
void Serial_SendString(const char* str);
void Serial_SendByte(uint8_t Byte);
void Serial_SendFloatNumber(float FloatNumber,uint8_t Int_Length, uint8_t Float_Length);

#endif
