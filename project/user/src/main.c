#include "zf_common_headfile.h"

#include "zf_device_oled.h"

#include "Menu.h"


// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完


int main(void)       
{
	//最好别动
    clock_init(SYSTEM_CLOCK_120M);                                              	// 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               	// 初始化默认 Debug UART

	Peripheral_Init();
        
    while(1)
    {
        
		Menu_Show();

    }
}
