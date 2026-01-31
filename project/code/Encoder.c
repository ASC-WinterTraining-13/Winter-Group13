#include "zf_driver_encoder.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器初始化
// 使用示例     Encoder_Init();
// 备注信息     只在"Menu.c"中的Peripheral_Init();函数调用一次
//-------------------------------------------------------------------------------------------------------------------

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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取编码器（电机1(左)）计数值
// 使用示例     LeftSpeed  = Get_Encoder1();
//				(LeftSpeed 为在"param_config.c"中定义的extern变量)
// 备注信息     一般作为速度环的必要调用，注意调用后计数值清零
//-------------------------------------------------------------------------------------------------------------------	

int Get_Encoder1(void)
{
	int16_t temp;
	temp = encoder_get_count(TIM3_ENCODER); 
	encoder_clear_count(TIM3_ENCODER);
	return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取编码器（电机2(右)）计数值
// 使用示例     RightSpeed  = Get_Encoder2();
//				(RightSpeed 为在"param_config.c"中定义的extern变量)
// 备注信息     一般作为速度环的必要调用，注意调用后计数值清零
//-------------------------------------------------------------------------------------------------------------------

int Get_Encoder2(void)
{
	int16_t temp;
	temp = encoder_get_count(TIM4_ENCODER); 
	encoder_clear_count(TIM4_ENCODER);
	return temp;
}
