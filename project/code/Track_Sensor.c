#include "zf_driver_gpio.h"

#include "Track_Sensor.h"

#include "math.h"

// 引脚接口宏定义
#define TRACK_X_1    E8     // X1(黑) → 接口E8
#define TRACK_X_2    E9     // X2(黄) → 接口E9
#define TRACK_X_3    E10    // X3(绿) → 接口E10
#define TRACK_X_4    E11    // X4(蓝) → 接口E11

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     循迹传感器模块 初始化
// 使用示例     Track_Sensor_Init();
// 备注信息     初始化4路引脚为上拉输入模式
//-------------------------------------------------------------------------------------------------------------------

void Track_Sensor_Init(void)
{
	// 初始化4路引脚为上拉输入模式
    gpio_init(TRACK_X_1, GPI, 0, GPI_PULL_UP);
    gpio_init(TRACK_X_2, GPI, 0, GPI_PULL_UP);
    gpio_init(TRACK_X_3, GPI, 0, GPI_PULL_UP);
    gpio_init(TRACK_X_4, GPI, 0, GPI_PULL_UP);	
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     循迹传感器模块 读取单个传感器引脚状态
// 参数说明     pin             GPIO引脚枚举（如E8/E9/E10/E11）
// 返回参数     uint8           引脚原始电平状态（高=1，低=0）
// 使用示例     uint8_t status = Track_Sensor_Get_Status(E8);
// 备注信息     内部函数，仅模块内调用
//-------------------------------------------------------------------------------------------------------------------

uint8 Track_Sensor_Get_Status(gpio_pin_enum pin)
{
    return gpio_get_level(pin);
}

//   引脚物理位置与引脚对应关系
// 左2	     左1     右1	   右2
// P2        P1      P3	       P4
// X2        X1      X3        X4
// E9        E8      E10       E11

// 黑  1
// 白  0

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     循迹传感器模块 获取所有传感器状态
// 参数说明     status_buf       输出参数，长度为4的数组，依次存储左2、左1、右1、右2传感器状态（黑=1，白=0）
// 使用示例     uint8_t sensor_data[4] = {0};
//              Track_Sensor_Get_All_Status(sensor_data);
// 备注信息     调试函数，一般不调用；对原始引脚状态取反，保证黑=1、白=0的逻辑
//-------------------------------------------------------------------------------------------------------------------

void Track_Sensor_Get_All_Status(uint8 status_buf[])
{
    status_buf[0] = 1 - Track_Sensor_Get_Status(TRACK_X_2);// 左2
    status_buf[1] = 1 - Track_Sensor_Get_Status(TRACK_X_1);// 左1
    status_buf[2] = 1 - Track_Sensor_Get_Status(TRACK_X_3);// 右1
    status_buf[3] = 1 - Track_Sensor_Get_Status(TRACK_X_4);// 右2
}

#define OUTER_WEIGHT				8		//外侧传感器权重
#define INNER_WEIGHT				2		//内侧传感器权重
#define ON_LINE_THRESHOLD			1.5		//有线判定阈值
#define TRACK_CNT_ON_LINE			3		//有线判定计数阈值
#define TRACK_CNT_OFF_LINE			7		//无线判定计数阈值

// 巡线状态 
uint8_t Track_Sensor_State = TRACK_STATE_ON_LINE;
static uint8_t Track_on_line_cnt = 0;    // 有线计数（防抖）
static uint8_t Track_off_line_cnt = 0;   // 无线计数（防抖）

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     循迹传感器模块 获取加权误差值（并更新有线/无线状态）
// 返回参数     float           循迹误差值（左偏为负，右偏为正）
// 使用示例     float error = Track_Sensor_Get_Error();
// 备注信息     1. 内部包含三次采样平均滤波，提升数据稳定性
//              2. 根据误差值更新Track_Sensor_State（有线/无线），包含防抖计数逻辑
//-------------------------------------------------------------------------------------------------------------------

float Track_Sensor_Get_Error(void)
{
	// 用于存储三次读取的临时数组
	uint8 sensor_data_1[4] = {0};
	uint8 sensor_data_2[4] = {0};
	uint8 sensor_data_3[4] = {0};
	
	// 三次连续读取
	Track_Sensor_Get_All_Status(sensor_data_1);
	Track_Sensor_Get_All_Status(sensor_data_2);
	Track_Sensor_Get_All_Status(sensor_data_3);
	
    float sensor_avg[4] = {0};
	
	// 分别取算术平均，范围0~1
	for(uint8 i=0; i<4; i++)
    {       
        sensor_avg[i] = (sensor_data_1[i] + sensor_data_2[i] + sensor_data_3[i]) / 3.0f;
    }
	
	float Error = 
		- OUTER_WEIGHT * sensor_avg[0]
		- INNER_WEIGHT * sensor_avg[1]
		+ INNER_WEIGHT * sensor_avg[2]
		+ OUTER_WEIGHT * sensor_avg[3];
	
	// 疑似有线
	if ( fabs(Error) >  ON_LINE_THRESHOLD)
	{
		Track_on_line_cnt ++;
		Track_off_line_cnt = 0;
		if (Track_on_line_cnt > TRACK_CNT_ON_LINE)
		{
			Track_Sensor_State = TRACK_STATE_ON_LINE;
			Track_on_line_cnt = 0;	
		}
	}
	// 疑似无线
	else
	{
		Track_off_line_cnt ++;
		Track_on_line_cnt = 0;
		if (Track_off_line_cnt > TRACK_CNT_OFF_LINE)
		{
			Track_Sensor_State = TRACK_STATE_OFF_LINE;
			Track_off_line_cnt = 0;
		}		
	}
	
	return Error;
}
