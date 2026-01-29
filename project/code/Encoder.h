#ifndef __ENCODER_H
#define __ENCODER_H

#include "zf_common_typedef.h"

////声明外部调用
//extern int16_t Encoder1_Count;
//extern int16_t Encoder2_Count;

void Encoder_Init(void);
//void Encoder_Tick(void);
	
int Get_Encoder1(void);
int Get_Encoder2(void);
	
#endif
