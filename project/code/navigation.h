/********************************************************************************************************************
* 惯性导航系统，移植自https://gitee.com/Emma321/navigation
* 移植涉及文件:
* "nag_flash.c"		"nag_flash.h"
* "navigation.c"	"navigation.h"
* 
* 本文件功能说明：
* 惯性导航上层调用
********************************************************************************************************************/


// 原有的开源库声明
/*
 * nagivation.h
 *
 *  Created on: 2024年10月16日
 *      Author: Monst
 */
 

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_


#include "zf_common_typedef.h"
#include "param_config.h"
#include "mpu6050_Analysis.h"


//*********************用户设置区域****************************//
#define MaxSize 252    				// flash存储的最大页面
#define Read_MaxSize 10000			// 最大读取设置，1w个应该是够了

#define NAG_MIN_SECTOR 80    		// 最小写保护块
#define NAG_MAX_SECTOR 120   		// 最大写保护块
#define NAG_START_PAGE_IN_SECTOR 3  // 每个写保护块从3页开始用（0~2页预留）
#define NAG_END_PAGE_IN_SECTOR 0    // 每个写保护块到0页结束

// 逻辑页与写保护块的转换
#define SECTOR_TO_LOGIC_PAGE(sector)  ((sector - NAG_MIN_SECTOR) * 4 + NAG_START_PAGE_IN_SECTOR)
#define LOGIC_PAGE_TO_SECTOR(page)    (NAG_MIN_SECTOR + (page - NAG_START_PAGE_IN_SECTOR) / 4)
#define LOGIC_PAGE_TO_PAGE(page)      ((page - NAG_START_PAGE_IN_SECTOR) % 4)

// 起始/结束逻辑页
#define Nag_End_Page    SECTOR_TO_LOGIC_PAGE(NAG_MAX_SECTOR) + NAG_END_PAGE_IN_SECTOR  // 120号块0页
#define Nag_Start_Page  SECTOR_TO_LOGIC_PAGE(NAG_MIN_SECTOR) + NAG_START_PAGE_IN_SECTOR// 80号块3页

#define Nag_Set_mileage 2100 		// 里程计//5cm
#define Nag_Prev 200    			// 前瞻（目前没有使用）
#define Nag_Yaw Yaw_Result 			// 陀螺仪读取出来的偏航角

#define L_Mileage LeftSpeed   		// 左轮编码器
#define R_Mileage RightSpeed 		// 右轮编码器
//********************************************************//

typedef struct{
       float Final_Out; 				// 最终输出
       float Mileage_All;   			// 里程计数
       float Angle_Run; 				// 读取的偏航角
       bool Nag_Stop_f; 				// 惯导中止flag
       uint8 Flash_read_f;				// 惯导读取flag
       uint16 size; 					// 惯导数组索引通用计数
       uint16 Run_index;
       uint16 Save_count;
       uint16 Save_index;				// 保存的flag
       uint8 Save_state;
       uint8 End_f;						// 中止flag
       //与flash相关的
       uint8 Flash_page_index;			// flash页面索引
       uint8 Flash_Save_Page_Index;		// flash保存页码索引
       uint8 Nag_SystemRun_Index;   	// 惯导执行索引
       //暂时未开发部分
       int Prev_mile[Nag_Prev]; 		// 前瞻
}Nag;

extern Nag N;   						// 整个变量的结构体，方便开发和移植
extern int32 Nav_read[Read_MaxSize];	// 按5cm算的话,1000可以跑50m
void Nag_Run(); 						// 偏航角复现总函数
void Run_Nag_GPS();						// 偏航角复现

void NagFlashRead();   					// Flash读取到目标数组。
void Run_Nag_Save();    				// 偏航角读取函数
void Nag_Read();    					// 偏航角读取总函数

void Init_Nag();   					 	// 这个是参数初始化与flash的缓冲区初始化，请放到函数开始。
void Nag_System();  					// 这个是惯性导航最后的包装函数，请放到中断中。
#endif /* _NAVIGATION_H_ */
