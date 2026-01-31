#ifndef PARAM_STORAGE_H
#define PARAM_STORAGE_H

#include "zf_common_headfile.h"

// Flash 存储位置
#define PARAM_FLASH_SECTION     (127)
#define PARAM_FLASH_PAGE        (3)

//===== 五组PID参数在 flash_union_buffer[] 中的索引 =====
// Angle_PID (0-2)
#define FLASH_ANGLE_KP      0
#define FLASH_ANGLE_KI      1
#define FLASH_ANGLE_KD      2

// Speed_PID (3-5)
#define FLASH_2_KP      3
#define FLASH_2_KI      4
#define FLASH_2_KD      5

// Turn_PID (6-8)
#define FLASH_3_KP      6
#define FLASH_3_KI      7
#define FLASH_3_KD      8

// mode_4_pid (9-11)
#define FLASH_4_KP      9
#define FLASH_4_KI      10
#define FLASH_4_KD      11

// mode_5_pid (12-14)
#define FLASH_5_KP      12
#define FLASH_5_KI      13
#define FLASH_5_KD      14

// 预留扩展 (15-255)
// 可以继续添加其他参数...

//===== 简化访问宏（直接操作缓冲区）=====
#define ANGLE_KP    flash_union_buffer[FLASH_ANGLE_KP].float_type
#define ANGLE_KI    flash_union_buffer[FLASH_ANGLE_KI].float_type
#define ANGLE_KD    flash_union_buffer[FLASH_ANGLE_KD].float_type

#define SPEED_KP    flash_union_buffer[FLASH_2_KP].float_type
#define SPEED_KI    flash_union_buffer[FLASH_2_KI].float_type
#define SPEED_KD    flash_union_buffer[FLASH_2_KD].float_type
		
#define TURN_KP    flash_union_buffer[FLASH_3_KP].float_type
#define TURN_KI    flash_union_buffer[FLASH_3_KI].float_type
#define TURN_KD    flash_union_buffer[FLASH_3_KD].float_type
		
#define TRACK_KP    flash_union_buffer[FLASH_4_KP].float_type
#define TRACK_KI    flash_union_buffer[FLASH_4_KI].float_type
#define TRACK_KD    flash_union_buffer[FLASH_4_KD].float_type
		
#define TEMP_888_FUNC_5_KP    flash_union_buffer[FLASH_5_KP].float_type
#define TEMP_888_FUNC_5_KI    flash_union_buffer[FLASH_5_KI].float_type
#define TEMP_888_FUNC_5_KD    flash_union_buffer[FLASH_5_KD].float_type

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化参数系统
// 使用示例     Param_Init();  // 在 main() 函数开始时调用一次
// 备注信息     首次使用时自动写入默认值，后续启动自动从Flash加载
//-------------------------------------------------------------------------------------------------------------------

void Param_Init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存参数到Flash
// 使用示例     Param_Save();  // 在退出参数修改时调用
// 备注信息     将当前缓冲区的数据写入Flash
//-------------------------------------------------------------------------------------------------------------------

void Param_Save(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     擦除Flash存储区域
// 使用示例     Param_Erase();  // 擦除后下次启动会重新写入默认值（默认值在本文件可以找到）
// 备注信息     慎用！会清空所有保存的参数
//-------------------------------------------------------------------------------------------------------------------

void Param_Erase(void);		

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     同步pid参数至缓存区（仍然需要调用Param_Save();）
// 使用示例     Param_SyncToPID();
// 备注信息     一般是配合蓝牙调参使用的（针对需要将关键pid参数存入flash的要求）
//-------------------------------------------------------------------------------------------------------------------

void Param_SyncToPID(void);

#endif
