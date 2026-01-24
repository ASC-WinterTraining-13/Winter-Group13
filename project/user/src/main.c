#include "zf_common_headfile.h"

#include "zf_device_oled.h"
#include "zf_driver_timer.h"

#include "Key.h"


// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完


#define SWITCH1                 (D3 )
#define SWITCH2                 (D4 )

#define PIT                     (TIM6_PIT )                                     	// 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY            (TIM6_IRQn)                                     	// 对应周期中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体

volatile uint8_t pit_state = 0;

//初始为无效值0
volatile uint8_t KeyNum = 0;

int main(void)       
{
    clock_init(SYSTEM_CLOCK_120M);                                              	// 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               	// 初始化默认 Debug UART

    //OLED初始化
    oled_init();
    //另一个可以使用的字体为8X16,但是坐标仍然是基于6X8标定的
    oled_set_font(OLED_6X8_FONT);    
	oled_clear();
    
    Key_Init();

//    //拨码开关初始化
//    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_FLOATING_IN);                        	// 初始化 SWITCH1 输入 默认高电平 浮空输入
//    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_FLOATING_IN);                        	// 初始化 SWITCH2 输入 默认高电平 浮空输入
    
    uint16_t i = 0, j1, j2, j3, j4;
    
	interrupt_set_priority(PIT_PRIORITY, 0);                                    	// 设置 PIT 对周期中断的中断优先级为 0
    pit_ms_init(PIT, 10);                                                     		// 初始化 PIT 为周期中断 10ms 周期
        
    while(1)
    {
        KeyNum = Key_GetState();
        
        j1 = j2= j3= j4 = 0;
        //按键按下为低电平q
        if (gpio_get_level(E2))
        {
            j1 = 1;
        }
        if (gpio_get_level(E3))
        {
            j2 = 1;
        }
        if (gpio_get_level(E4))
        {
            j3 = 1;
        }
        if (gpio_get_level(E5))
        {
            j4 = 1;
        }
//            
//            
//        //拨码开关拨下为高电平，拨上为低电平        
//        if(gpio_get_level(SWITCH2))//1
//        {
//            i = 1;
//        }
//        if(!gpio_get_level(SWITCH2))//0
//        {
//            j = 1;
//        }              
                
        if(pit_state)
        {
            i ++;    
            
            pit_state = 0;                                                      // 清空周期中断触发标志位         
        }
        

        oled_show_uint(0, 0, KeyNum, 4);
        oled_show_uint(0, 1, i, 4);
        oled_show_uint(0, 2, j1, 4);
        oled_show_uint(0, 3, j2, 4);
        oled_show_uint(0, 4, j3, 4);
        oled_show_uint(0, 5, j4, 4);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 的周期中断处理函数 这个函数将在 PIT 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{
    pit_state = 1;                                                              // 周期中断触发 标志位置位    
}

// **************************** 代码区域 ****************************