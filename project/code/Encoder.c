#include "zf_driver_encoder.h"

// 编码器初始化
void Encoder_Init(void)
{
	// B4+B5
	// CH1引脚（A相）：TIM3_ENCODER_CH1_B4（对应B4）
	// CH2引脚（B相）：TIM3_ENCODER_CH2_B5（对应B5）
	encoder_quad_init(TIM3_ENCODER, TIM3_ENCODER_CH1_B4, TIM3_ENCODER_CH2_B5);
    encoder_clear_count(TIM3_ENCODER); // 初始清零，避免初始值干扰
	// B6+B7
	// CH1引脚（A相）：TIM4_ENCODER_CH1_B6（对应B6）
	// CH2引脚（B相）：TIM4_ENCODER_CH2_B7（对应B7）
	encoder_quad_init(TIM4_ENCODER, TIM4_ENCODER_CH1_B6, TIM4_ENCODER_CH2_B7);
    encoder_clear_count(TIM4_ENCODER); // 初始清零
}

////计数值
//int16_t Encoder1_Count = 0;
//int16_t Encoder2_Count = 0;

//// 编码器读取及清空
//void Encoder_Tick(void)
//{
//	//读取
//	Encoder1_Count = encoder_get_count(TIM3_ENCODER); 
//	Encoder2_Count = encoder_get_count(TIM4_ENCODER); 
//	
//	//计数清零
//	encoder_clear_count(TIM3_ENCODER);
//	encoder_clear_count(TIM4_ENCODER);
//}

int Get_Encoder1(void)
{
	int16_t temp;
	temp = encoder_get_count(TIM3_ENCODER); 
	encoder_clear_count(TIM3_ENCODER);
	return temp;
}

int Get_Encoder2(void)
{
	int16_t temp;
	temp = encoder_get_count(TIM4_ENCODER); 
	encoder_clear_count(TIM4_ENCODER);
	return temp;
}
