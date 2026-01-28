#ifndef _motor_h_
#define _motor_h_

#include "zf_common_typedef.h"

void motor_Init(void);
// motor_id取1（左）或2（右）;
//PWM有效值-10000~10000
void motor_SetPWM(uint8_t motor_id, int PWM);

#endif