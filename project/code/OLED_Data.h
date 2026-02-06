/********************************************************************************************************************
* 江协科技OLED-V2.0/UFT-8/7针脚SPI接口+逐飞科技spi底层 整合项目
* 即基于逐飞科技spi底层迁移江协科技OLED上层功能
* 利用GPT-5进行整合，可以显示中文，使用格式化打印，和大部分线条绘制功能
* 整合涉及文件：
* "OLED.c"			"OLED.h"
* "OLED_Data.c"		"OLED_Data.h"
* 字符集添加操作与江协科技OLED驱动基本相同
********************************************************************************************************************/


#ifndef __OLED_DATA_H
#define __OLED_DATA_H

#include <stdint.h>

/*字符集定义*/
/*以下两个宏定义只可解除其中一个的注释*/
#define OLED_CHARSET_UTF8			//定义字符集为UTF8
//#define OLED_CHARSET_GB2312		//定义字符集为GB2312

/*字模基本单元*/
typedef struct 
{
	
#ifdef OLED_CHARSET_UTF8			//定义字符集为UTF8
	char Index[5];					//汉字索引，空间为5字节
#endif
	
#ifdef OLED_CHARSET_GB2312			//定义字符集为GB2312
	char Index[3];					//汉字索引，空间为3字节
#endif
	
	uint8_t Data[32];				//字模数据
} ChineseCell_t;

/*ASCII字模数据声明*/
extern const uint8_t OLED_F8x16[][16];
extern const uint8_t OLED_F6x8[][6];

/*汉字字模数据声明*/
extern const ChineseCell_t OLED_CF16x16[];

/*图像数据声明*/
extern const uint8_t Diode[];
/*按照上面的格式，在这个位置加入新的图像数据声明*/
//...

#endif


/*****************江协科技|版权所有****************/
/*****************jiangxiekeji.com*****************/
