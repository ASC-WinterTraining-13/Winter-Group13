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
#define MaxSize 252    				// flash每页最大存储条数
#define NAG_MIN_SECTOR 80    		// Flash最小写保护块（空闲区起始）
#define NAG_MAX_SECTOR 120   		// Flash最大写保护块（空闲区结束）
#define NAG_START_PAGE_IN_SECTOR 3  // 每个写保护块从3页开始用（0~2页预留）
#define NAG_END_PAGE_IN_SECTOR 0    // 每个写保护块到0页结束

// 逻辑页与写保护块的转换（核心：映射到80~120号块）
#define SECTOR_TO_LOGIC_PAGE(sector)  ((sector - NAG_MIN_SECTOR) * 4 + NAG_START_PAGE_IN_SECTOR)
#define LOGIC_PAGE_TO_SECTOR(page)    (NAG_MIN_SECTOR + (page - NAG_START_PAGE_IN_SECTOR) / 4)
#define LOGIC_PAGE_TO_PAGE(page)      ((page - NAG_START_PAGE_IN_SECTOR) % 4)

// 起始/结束逻辑页（80号块3页 ~ 120号块0页）
#define Nag_End_Page    SECTOR_TO_LOGIC_PAGE(NAG_MAX_SECTOR) + NAG_END_PAGE_IN_SECTOR
#define Nag_Start_Page  SECTOR_TO_LOGIC_PAGE(NAG_MIN_SECTOR) + NAG_START_PAGE_IN_SECTOR

#define Nag_Set_mileage 2100 		// 里程阈值（5cm）
#define Nag_Prev 200    			// 前瞻（未使用）
#define Nag_Yaw Yaw_Result 			// 陀螺仪实时偏航角

#define L_Mileage LeftSpeed   		// 左轮编码器
#define R_Mileage RightSpeed 		// 右轮编码器
//********************************************************//

// 惯导核心结构体（混合：标志位/计数值/索引/输入/输出）
// 仅实例化为全局变量N，所有惯导数据通过N访问
typedef struct{
    float Final_Out; 				// [输出]偏航角误差值（-180~180°）
    float Mileage_All;   			// [输入/计数]累计里程（触发存储）
    float Angle_Run; 				// [中间]复现用目标偏航角
    bool Nag_Stop_f; 				// [标志]惯导停止（1=停止纠偏）
    uint8 Flash_read_f;				// [标志]Flash读取状态（预留）
    uint16 size; 					// [索引]Flash缓冲区索引
    uint16 Run_index;				// [索引]复现路径索引
    uint16 Save_index;				// [计数]偏航角总存储条数
    uint8 Save_state;				// [标志]Flash读取完成（1=完成）
    uint8 End_f;					// [标志]记录状态（0=记录中，1=写最后一页，2=结束）
    // Flash相关
    uint8 Flash_page_index;			// [索引]当前操作的Flash逻辑页
    uint8 Nag_SystemRun_Index;   	// [标志]运行模式（0=停止，1=记录，3=复现）
}Nag;

// 全局变量声明
extern Nag N;

// 函数声明
void Nag_Run(void); 						// 偏航角复现总函数
void Run_Nag_GPS(void);						// 偏航角复现（按需读取Flash）
void NagFlashRead(void);   					// 废弃：一次性读取（保留定义）
void Run_Nag_Save(void);    				// 偏航角记录函数
void Nag_Read(void);    					// 偏航角记录总函数
void Init_Nag(void);   						// 惯导初始化
void Nag_System(void);  					// 惯导执行函数（中断调用）

#endif /* _NAVIGATION_H_ */
