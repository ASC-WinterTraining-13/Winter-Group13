#ifndef PARAM_STORAGE_H
#define PARAM_STORAGE_H

#include "zf_common_headfile.h"

// PID Flash 存储位置
#define PARAM_FLASH_SECTION     (127)
#define PARAM_FLASH_PAGE        (3)

// PID参数缓存（与flash_union_buffer解耦）
extern float param_cache[21];

/* 六组PID参数在 param_cache[] 中的索引*/
// Rate__PID (0-2)角速度环
#define FLASH_RATE__KP      0
#define FLASH_RATE__KI      1
#define FLASH_RATE__KD      2

// Angle_PID (3-5)角度环
#define FLASH_ANGLE_KP      3
#define FLASH_ANGLE_KI      4
#define FLASH_ANGLE_KD      5

// Speed_PID (6-8)速度环
#define FLASH_SPEED_KP      6
#define FLASH_SPEED_KI      7
#define FLASH_SPEED_KD      8

// Turn__pid (9-11)转向环
#define FLASH_TURN__KP      9
#define FLASH_TURN__KI      10
#define FLASH_TURN__KD      11

// Track_pid (12-14)循迹环
#define FLASH_TRACK_KP      12
#define FLASH_TRACK_KI      13
#define FLASH_TRACK_KD      14

// Head__pid (15-17)航向角环
#define FLASH_HEAD__KP      15
#define FLASH_HEAD__KI      16
#define FLASH_HEAD__KD      17

// POSI__pid (18-20)位置环
#define FLASH_POSI__KP      18
#define FLASH_POSI__KI      19
#define FLASH_POSI__KD      20

// 预留扩展 (21-255)
// 可以继续添加其他参数...

// 简化访问宏（直接操作参数缓存）
#define RATE__KP    param_cache[FLASH_RATE__KP]
#define RATE__KI    param_cache[FLASH_RATE__KI]
#define RATE__KD    param_cache[FLASH_RATE__KD]

#define ANGLE_KP    param_cache[FLASH_ANGLE_KP]
#define ANGLE_KI    param_cache[FLASH_ANGLE_KI]
#define ANGLE_KD    param_cache[FLASH_ANGLE_KD]
		
#define SPEED_KP    param_cache[FLASH_SPEED_KP]
#define SPEED_KI    param_cache[FLASH_SPEED_KI]
#define SPEED_KD    param_cache[FLASH_SPEED_KD]
		
#define TURN__KP    param_cache[FLASH_TURN__KP]
#define TURN__KI    param_cache[FLASH_TURN__KI]
#define TURN__KD    param_cache[FLASH_TURN__KD]
		
#define TRACK_KP    param_cache[FLASH_TRACK_KP]
#define TRACK_KI    param_cache[FLASH_TRACK_KI]
#define TRACK_KD    param_cache[FLASH_TRACK_KD]

#define HEAD__KP    param_cache[FLASH_HEAD__KP]
#define HEAD__KI    param_cache[FLASH_HEAD__KI]
#define HEAD__KD    param_cache[FLASH_HEAD__KD]

#define POSI__KP    param_cache[FLASH_POSI__KP]
#define POSI__KI    param_cache[FLASH_POSI__KI]
#define POSI__KD    param_cache[FLASH_POSI__KD]

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     同步pid参数至缓存区（仍然需要调用Param_Save();）
// 使用示例     Param_SyncToPID();
// 备注信息     一般是配合蓝牙调参使用的（针对需要将关键pid参数存入flash的要求）
//-------------------------------------------------------------------------------------------------------------------
void Param_SyncToPID(void);

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

#endif
