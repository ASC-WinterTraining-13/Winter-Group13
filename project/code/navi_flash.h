/********************************************************************************************************************
* 惯性导航系统，移植自https://gitee.com/Emma321/navigation
* 移植涉及文件:
* "navi_flash.c"	"navi_flash.h"
* "navigation.c"	"navigation.h"
* 
* 本文件功能说明：
* flash读写操作三次包装
********************************************************************************************************************/


#ifndef _NAG_FLASH_H_
#define _NAG_FLASH_H_

#include "zf_common_typedef.h"
#include "navigation.h"


void flash_Navi_Write(void);  // 写入惯导数据到Flash
void flash_Navi_Read(void);   // 从Flash读取惯导数据

#endif
