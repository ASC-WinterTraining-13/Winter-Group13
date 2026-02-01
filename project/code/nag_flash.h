#ifndef _NAG_FLASH_H_
#define _NAG_FLASH_H_

#include "zf_common_typedef.h"
#include "navigation.h"

// 惯导Flash操作接口（仅暴露业务层需要的接口）
void nag_flash_write_data(void);  // 写入惯导数据到Flash
void nag_flash_read_data(void);   // 从Flash读取惯导数据

#endif
