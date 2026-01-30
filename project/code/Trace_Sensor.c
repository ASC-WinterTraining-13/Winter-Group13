#include "zf_driver_gpio.h"

#define TRACK_X_1    E8     // X1(黑) → 接口E8
#define TRACK_X_2    E9     // X2(黄) → 接口E9
#define TRACK_X_3    E10    // X3(绿) → 接口E10
#define TRACK_X_4    E11    // X4(蓝) → 接口E11

// 四路循迹模块初始化
void Trace_Sensor_Init(void)
{
	// 初始化4路引脚为上拉输入模式
    gpio_init(TRACK_X_1, GPI, 0, GPI_PULL_UP);
    gpio_init(TRACK_X_2, GPI, 0, GPI_PULL_UP);
    gpio_init(TRACK_X_3, GPI, 0, GPI_PULL_UP);
    gpio_init(TRACK_X_4, GPI, 0, GPI_PULL_UP);	
}

// 单路读取
uint8 Trace_Sensor_Get_Status(gpio_pin_enum pin)
{
    return gpio_get_level(pin);
}

// 引脚物理位置与引脚对应
// 左		中		   右
// P2      P1  P3	   P4
// X2      X1  X3       X4
// E9      E8  E10      E11

// 多路读取
// 调用示例：
// uint8 sensor_data[4] = {0};  // 初始值全为0
// Track_Sensor_Get_All_Status(sensor_data);  // 调用函数
void Trace_Sensor_Get_All_Status(uint8 status_buf[])
{
    status_buf[0] = Trace_Sensor_Get_Status(TRACK_X_2);
    status_buf[1] = Trace_Sensor_Get_Status(TRACK_X_1);
    status_buf[2] = Trace_Sensor_Get_Status(TRACK_X_3);
    status_buf[3] = Trace_Sensor_Get_Status(TRACK_X_4);
}
