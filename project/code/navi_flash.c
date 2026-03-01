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
	uint32 end_sector = LOGIC_PAGE_TO_SECTOR(Nag_End_Page);
	uint32 end_page = LOGIC_PAGE_TO_PAGE(Nag_End_Page);
	
	// 检查操作是否越界
	if(check_sector_valid(sector))
    {
        return;
    }
	
	// 在每页的固定位置保存该页实际存储的数据条数
	flash_union_buffer[MaxSize].uint32_type = N.size;

	// 结束记录且当前写入页正好是结束页：同页一次性写入Save_index，避免二次擦写覆盖数据
	if (N.End_f == 1 && N.Flash_page_index == Nag_End_Page)
	{
		flash_union_buffer[MaxSize+2].uint32_type = N.Save_index;
	}
	
	// 校验 FLASH 是否有数据（1-有数据）
   if(flash_check(sector, page))
	{
		// 擦除当前页
		flash_erase_page(sector, page);
	}
                       
	// 写入缓冲区数据到Flash
     flash_write_page_from_buffer(sector, page);
	
	// 如果是结束记录
    if(N.End_f == 1)
    {
		// 当前页不是结束页：需要在不破坏结束页原有轨迹数据的前提下更新Save_index
		if (N.Flash_page_index != Nag_End_Page)
		{
			flash_buffer_clear();
			if (check_sector_valid(end_sector) == 0)
			{
				if (flash_check(end_sector, end_page))
				{
					flash_read_page_to_buffer(end_sector, end_page);
				}
				flash_union_buffer[MaxSize+2].uint32_type = N.Save_index;
				if (flash_check(end_sector, end_page))
				{
					flash_erase_page(end_sector, end_page);
				}
				flash_write_page_from_buffer(end_sector, end_page);
			}
		}
    }
    
    flash_buffer_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯性导航读flash数据
//-------------------------------------------------------------------------------------------------------------------
void flash_Navi_Read(void)
{
    flash_buffer_clear();

    if( 0 == N.Index_R_f)
    {
		// 转换结束页的写保护块+页
		uint32 end_sector = LOGIC_PAGE_TO_SECTOR(Nag_End_Page);
		uint32 end_page = LOGIC_PAGE_TO_PAGE(Nag_End_Page);
		flash_read_page_to_buffer(end_sector, end_page);

		N.Save_index = flash_union_buffer[MaxSize+2].uint32_type;       
		N.Index_R_f=1;
		flash_buffer_clear();
	}
	
   // 转换当前页的写保护块+页
    uint32 sector = LOGIC_PAGE_TO_SECTOR(N.Flash_page_index);
    uint32 page = LOGIC_PAGE_TO_PAGE(N.Flash_page_index);
    if(check_sector_valid(sector) == 0 && flash_check(sector, page))
    {        
        flash_read_page_to_buffer(sector, page);
        // 读取当前页实际存储的数据条数
        N.current_page_size = flash_union_buffer[MaxSize].uint32_type;
        // 校验数据条数的有效性（防止非法数据）
        if(N.current_page_size > MaxSize)
        {
            N.current_page_size = MaxSize;
        }
    }
}
