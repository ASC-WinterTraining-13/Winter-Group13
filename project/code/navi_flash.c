/********************************************************************************************************************
* 惯性导航系统，移植自https://gitee.com/Emma321/navigation
* 移植涉及文件:
* "navi_flash.c"	"navi_flash.h"
* "navigation.c"	"navigation.h"
* 
* 本文件功能说明：
* flash读写操作三次包装
********************************************************************************************************************/


#include "zf_driver_flash.h"
#include "navi_flash.h"
#include "navigation.h"

#define FLASH_PAGE_LENGTH FLASH_DATA_BUFFER_SIZE

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查越界操作
//-------------------------------------------------------------------------------------------------------------------
static uint8 check_sector_valid(uint32 sector)
{
    return (sector >= NAG_MIN_SECTOR && sector <= NAG_MAX_SECTOR) ? 0 : 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯性导航写flash数据
//-------------------------------------------------------------------------------------------------------------------
void flash_Navi_Write(void)
{
	// 转换逻辑页为写保护块+页
	uint32 sector = LOGIC_PAGE_TO_SECTOR(N.Flash_page_index);
    uint32 page = LOGIC_PAGE_TO_PAGE(N.Flash_page_index);
	
	// 检查操作是否越界
	if(check_sector_valid(sector))
    {
        // 想想是不是得返回什么
        return;
    }
	
	// 校验 FLASH 是否有数据（1-有数据）
   if(flash_check(sector, page))
	{
		// 擦除当前页
		flash_erase_page(sector, page);
	}
                       
	// 写入缓冲区数据到Flash
     flash_write_page_from_buffer(sector, page);
	
	// 如果
    if(N.End_f == 1)// 惯导结构体的结束标志位
    {    
		flash_union_buffer[MaxSize+2].uint32_type = N.Save_index;
		// 转换结束页的写保护块+页
		uint32 end_sector = LOGIC_PAGE_TO_SECTOR(Nag_End_Page);
		uint32 end_page = LOGIC_PAGE_TO_PAGE(Nag_End_Page);
		flash_write_page_from_buffer(end_sector, end_page);
    }
    
    flash_buffer_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯性导航读flash数据
//-------------------------------------------------------------------------------------------------------------------
void flash_Navi_Read(void)
{
    flash_buffer_clear();
    static uint8 Index_R_f=0;

    if( 0 == Index_R_f)
    {
		// 转换结束页的写保护块+页
		uint32 end_sector = LOGIC_PAGE_TO_SECTOR(Nag_End_Page);
		uint32 end_page = LOGIC_PAGE_TO_PAGE(Nag_End_Page);
		flash_read_page_to_buffer(end_sector, end_page);

		N.Save_index = flash_union_buffer[MaxSize+2].uint32_type;       
		Index_R_f=1;
		flash_buffer_clear();
	}
	
   // 转换当前页的写保护块+页
    uint32 sector = LOGIC_PAGE_TO_SECTOR(N.Flash_page_index);
    uint32 page = LOGIC_PAGE_TO_PAGE(N.Flash_page_index);
    if(check_sector_valid(sector) == 0 && flash_check(sector, page))
    {        
        flash_read_page_to_buffer(sector, page);
    }
}
