#include "zf_common_headfile.h"

void motor_Init(void)//电机初始化
{
	pwm_init(TIM5_PWM_CH2_A1, 17000, 0);
	pwm_init(TIM5_PWM_CH4_A3, 17000, 0);//PWM端口设定为A1，A3
	
	gpio_init (A0, GPO, 0, GPO_PUSH_PULL);
	gpio_init (A2, GPO, 0, GPO_PUSH_PULL);//电机1驱动端口设定为A0,A2
	
	gpio_init (D5, GPO, 0, GPO_PUSH_PULL);
	gpio_init (D6, GPO, 0, GPO_PUSH_PULL);//电机2驱动端口设定为D5,D6
}

void motor_SetPWM(uint8_t motor_id, int PWM)
{

	if (motor_id == 1)
	{
		if (PWM >= 0)
		{
			gpio_set_level (A2,0);
			gpio_set_level (A0,1);//正反转可调.
			pwm_set_duty(TIM5_PWM_CH2_A1,PWM);
		}
		else
		{
			gpio_set_level (A2,1);
			gpio_set_level (A0,0);//正反转可调
			pwm_set_duty (TIM5_PWM_CH2_A1, -PWM);
		}
	}
	else if (motor_id == 2)
	{
		if (PWM >= 0)
		{
			gpio_set_level (D5,0);
			gpio_set_level (D6,1);//正反转可调.
			pwm_set_duty(TIM5_PWM_CH4_A3,PWM);
		}
		else
		{
			gpio_set_level (D5,1);
			gpio_set_level (D6,0);//正反转可调.
			pwm_set_duty(TIM5_PWM_CH4_A3,-PWM);
		}
	}
}