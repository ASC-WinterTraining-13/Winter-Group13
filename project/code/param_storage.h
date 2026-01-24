#ifndef PARAM_STORAGE_H
#define PARAM_STORAGE_H

#include "zf_common_headfile.h"

// Flash 存储位置
#define PARAM_FLASH_SECTION     (127)
#define PARAM_FLASH_PAGE        (3)

//===== 五组PID参数在 flash_union_buffer[] 中的索引 =====
// angle_pid (0-2)
#define FLASH_ANGLE_KP      0
#define FLASH_ANGLE_KI      1
#define FLASH_ANGLE_KD      2

// mode_2_pid (3-5)
#define FLASH_MODE2_KP      3
#define FLASH_MODE2_KI      4
#define FLASH_MODE2_KD      5

// mode_3_pid (6-8)
#define FLASH_MODE3_KP      6
#define FLASH_MODE3_KI      7
#define FLASH_MODE3_KD      8

// mode_4_pid (9-11)
#define FLASH_MODE4_KP      9
#define FLASH_MODE4_KI      10
#define FLASH_MODE4_KD      11

// mode_5_pid (12-14)
#define FLASH_MODE5_KP      12
#define FLASH_MODE5_KI      13
#define FLASH_MODE5_KD      14

// 预留扩展 (15-255)
// 可以继续添加其他参数...

//===== 简化访问宏（直接操作缓冲区）=====
#define ANGLE_KP    flash_union_buffer[FLASH_ANGLE_KP].float_type
#define ANGLE_KI    flash_union_buffer[FLASH_ANGLE_KI].float_type
#define ANGLE_KD    flash_union_buffer[FLASH_ANGLE_KD].float_type

#define MODE2_KP    flash_union_buffer[FLASH_MODE2_KP].float_type
#define MODE2_KI    flash_union_buffer[FLASH_MODE2_KI].float_type
#define MODE2_KD    flash_union_buffer[FLASH_MODE2_KD].float_type

#define MODE3_KP    flash_union_buffer[FLASH_MODE3_KP].float_type
#define MODE3_KI    flash_union_buffer[FLASH_MODE3_KI].float_type
#define MODE3_KD    flash_union_buffer[FLASH_MODE3_KD].float_type

#define MODE4_KP    flash_union_buffer[FLASH_MODE4_KP].float_type
#define MODE4_KI    flash_union_buffer[FLASH_MODE4_KI].float_type
#define MODE4_KD    flash_union_buffer[FLASH_MODE4_KD].float_type

#define MODE5_KP    flash_union_buffer[FLASH_MODE5_KP].float_type
#define MODE5_KI    flash_union_buffer[FLASH_MODE5_KI].float_type
#define MODE5_KD    flash_union_buffer[FLASH_MODE5_KD].float_type

//===== 功能函数 =====
void Param_Init(void);     // 初始化参数系统（main函数启动时调用）
void Param_Save(void);     // 保存参数到Flash（退出修改时调用）
void Param_Erase(void);

#endif
