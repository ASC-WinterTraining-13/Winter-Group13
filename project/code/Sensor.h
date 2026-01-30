#ifndef __SENSOR_H
#define __SENSOR_H

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

void Track_Sensor_Init(void);
uint8 Track_Sensor_Get_Status(gpio_pin_enum pin);
void Track_Sensor_Get_All_Status(uint8 status_buf[]);

#endif
