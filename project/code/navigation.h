/********************************************************************************************************************
* 惯性导航系统，移植自https://gitee.com/Emma321/navigation
* 移植涉及文件:
* "nag_flash.c"		"nag_flash.h"
* "kalman.c"		"kalman.h"
* "navigation.c"	"navigation.h"
* 
* 本文件功能说明：
* 惯性导航上层调用
********************************************************************************************************************/


#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "zf_common_typedef.h"

// *********************用户可配置区域（最终版）****************************//
#define MaxSize          256     // 每页最大存储数（MM32 Flash每页1K=256个int32）
#define Read_MaxSize     10000   // 最大读取缓存（10000*4=40KB，MM32 RAM足够）

// 核心：预留40个扇区给程序（0~39），惯导从40扇区开始
#define Nag_End_Page     0       // 扇区内起始页（0-3，固定0）
#define Nag_Start_Page   3       // 扇区内结束页（0-3，固定3）
#define Nag_Start_Sector 40      // 惯导起始扇区（避开0~39的程序预留区）
#define Nag_End_Sector   126     // 惯导结束扇区（避开127扇区的PID区）

#define Nag_Set_mileage  2100    // 里程计触发阈值（5cm触发一次）
#define Nag_Prev         200     // 前瞻数量
#define Nag_Yaw          angle_Z // 偏航角来源（陀螺仪解算值）

// 修正：替换为逐飞标准编码器接口（你可根据实际编码器通道修改）
// 编码器1=左轮，编码器2=右轮（逐飞库默认接口）
#define L_Mileage        encoder_get_count(1)
#define R_Mileage        encoder_get_count(2)
// ********************************************************//

// 惯导控制结构体
typedef struct{
    float Final_Out;                 // 最终偏差输出
    float Mileage_All;               // 累计里程
    float Angle_Run;                 // 读取的目标偏航角
    bool Nag_Stop_f;                 // 惯导停止标志
    uint8 Flash_read_f;              // 惯导读取标志
    uint16 size;                     // 数据索引
    uint16 Run_index;                // 复现索引
    uint16 Save_count;               // 保存计数
    uint16 Save_index;               // 保存索引
    uint8 Save_state;                // 保存状态
    uint8 End_f;                     // 结束标志
    uint8 Flash_sector_index;        // 扇区索引（0-127）
    uint8 Flash_page_index;          // 扇区内的页索引（0-3）
    uint8 Flash_Save_Page_Index;     // Flash保存页索引
    uint8 Nag_SystemRun_Index;       // 惯导运行索引
    int Prev_mile[Nag_Prev];         // 前瞻数组
}Nag;

// 全局变量
extern Nag N;
extern int32 Nav_read[Read_MaxSize];

// 函数声明
void Nag_Run();
void Run_Nag_GPS();
void NagFlashRead();
void Run_Nag_Save();
void Nag_Read();
void Init_Nag();
void Nag_System();

#endif
