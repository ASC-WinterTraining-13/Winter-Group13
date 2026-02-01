/********************************************************************************************************************
* 惯性导航系统，移植自https://gitee.com/Emma321/navigation
* 移植涉及文件:
* "nag_flash.c"		"nag_flash.h"
* "kalman.c"		"kalman.h"
* "navigation.c"	"navigation.h"
* 
* 本文件功能说明：
* flash读写操作三次包装
********************************************************************************************************************/


#include "zf_common_headfile.h"
#include "zf_driver_flash.h"  // 调用逐飞原厂Flash驱动
#include "nag_flash.h"
#include "navigation.h"

// 写入惯导数据到Flash（三次包装，业务层接口）
void nag_flash_write_data(void)
{
    // 调用逐飞原厂接口：校验当前页是否有数据
    if(flash_check(0, N.Flash_page_index))
        // 调用逐飞原厂接口：擦除当前页
        flash_erase_page(0, N.Flash_page_index);                  
                       
    // 调用逐飞原厂接口：写入缓冲区数据到Flash
    flash_write_page_from_buffer(0, N.Flash_page_index);
    
    // 写入结束标记（最后一页）
    if(N.End_f == 1)
    {    
        flash_union_buffer[MaxSize - 1].uint32_type = N.Save_index;
        flash_write_page_from_buffer(0, Nag_End_Page);
    }
    
    // 调用逐飞原厂接口：清空缓冲区
    flash_buffer_clear();
}

// 从Flash读取惯导数据（三次包装，业务层接口）
void nag_flash_read_data(void)
{
    // 调用逐飞原厂接口：清空缓冲区
    flash_buffer_clear();
    static uint8 Index_R_f=0;

    // 第一次读取：读取结束页的保存索引
    if( 0 == Index_R_f)
    {
        // 调用逐飞原厂接口：读取结束页数据到缓冲区
        flash_read_page_to_buffer(0, Nag_End_Page);
        N.Save_index = flash_union_buffer[MaxSize - 1].uint32_type;       
        Index_R_f=1;
        flash_buffer_clear();
    }
    
    // 调用逐飞原厂接口：读取当前页数据（如果有数据）
    if(flash_check(0, N.Flash_page_index))
    {        
        flash_read_page_to_buffer(0, N.Flash_page_index);
    }
}
