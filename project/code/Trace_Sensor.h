#ifndef __TRACE_SENSOR_H
#define __TRACE_SENSOR_H

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

void Trace_Sensor_Init(void);
void Trace_Sensor_Get_All_Status(uint8 status_buf[]);

#endif
