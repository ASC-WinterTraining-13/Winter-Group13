/********************************************************************************************************************
* 惯性导航系统，移植自https://gitee.com/Emma321/navigation
* 移植涉及文件:
* "nag_flash.c"		"nag_flash.h"
* "navigation.c"	"navigation.h"
* 
* 本文件功能说明：
* 惯性导航上层调用
********************************************************************************************************************/


#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "zf_common_typedef.h"

// *********************用户可配置区域（最终版）****************************//
#define MaxSize          256     // 保留
#define Read_MaxSize     10000   // 保留

#define Nag_End_Page     0       // 保留
#define Nag_Start_Page   3       // 保留
#define Nag_Start_Sector 40      // 保留
#define Nag_End_Sector   126     // 保留

// 修改1：里程阈值改为浮点型（5cm=50mm，适配你的里程单位）
#define Nag_Set_mileage  50.0f   
#define Nag_Prev         200     // 保留
#define Nag_Yaw          angle_Z // 保留

// 修改2：删除原编码器宏定义（已整合到navigation.c的函数中）
// 注释掉这两行：
// #define L_Mileage        encoder_get_count(1)
// #define R_Mileage        encoder_get_count(2)
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
