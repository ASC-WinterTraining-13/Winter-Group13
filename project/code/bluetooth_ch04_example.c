/*********************************************************************************************************************
* CH-04 蓝牙模块 应用示例
* 适配逐飞 MM32F327X 开源库
* 
* 功能说明：
* 1. 接收来自蓝牙的数据包（格式：[数据]）
* 2. 使用 strtok 分割字符串
* 3. 使用 strcmp 比对指令
* 4. 使用 atoi/atof 进行类型转换
* 5. 集成逐飞库的OLED显示功能
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "zf_device_bluetooth_ch04.h"
#include <stdio.h>
#include <string.h>

#include "param_config.h"
#include "param_storage.h"

// ========== 蓝牙接收缓冲区 ==========
static uint8 bluetooth_rx_buffer[100];
static uint32 bluetooth_rx_length = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙数据处理 - 处理接收到的蓝牙数据包
// 参数说明     data_packet     接收到的数据包（不含头尾）
// 参数说明     length          数据包长度
// 返回参数     void
// 使用示例     bluetooth_ch04_process_data(rx_buffer, 20);
// 备注信息     这个函数包含了你原来的 Serial_RxFlag 的所有处理逻辑
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_ch04_process_data (uint8 *data_packet, uint32 length)
{
    // 局部变量声明
    char *tag;				//标签
    char *name;				//名称
    char *action;			//（按键up/down）状态
    char *value;			//值
    int8 lh, lv, rh, rv;	//摇杆解析
    uint8 int_value;		//解析值
    float float_value;		//解析值
    
    // 创建可修改的副本（strtok 会修改原字符串）
    static char parse_buffer[100];
    
    // 复制数据到本地缓冲区
    if(length >= sizeof(parse_buffer))
    {
        length = sizeof(parse_buffer) - 1;
    }
    memcpy(parse_buffer, (char *)data_packet, length);
    parse_buffer[length] = '\0';
    
    // ===== 第一步：字符串分割 =====
    tag = strtok(parse_buffer, ",");
    
    if(tag == NULL)
    {
        return;  // 无效数据
    }
    
    // ===== 第二步：根据标签类型处理数据 =====
    
    // ========== 按键事件处理 ==========
    if(strcmp(tag, "key") == 0)
    {
        name = strtok(NULL, ",");
        action = strtok(NULL, ",");
        
        if(name != NULL && action != NULL)
        {
            if(strcmp(name, "ON") == 0 && strcmp(action, "down") == 0)
            {
				Run_Flag = 1;
            }
            else if(strcmp(name, "OF") == 0 && strcmp(action, "down") == 0)
            {
                Run_Flag = 0;
            }
        }
    }
    
    // ========== 滑杆事件处理 ==========
    else if(strcmp(tag, "slider") == 0)
    {
        name = strtok(NULL, ",");
        value = strtok(NULL, ",");
        
        if(name != NULL && value != NULL)
        {
            if(strcmp(name, "A_P") == 0)
            {
				float_value = (float)atof(value);
				ANGLE_KP = float_value;
            }
            else if(strcmp(name, "A_I") == 0)
            {
//              printf("slider 2 value: %.2f\r\n", float_value);
				float_value = (float)atof(value);
				ANGLE_KI = float_value;
            }
			else if(strcmp(name, "A_D") == 0)
            {
				float_value = (float)atof(value);
				ANGLE_KD = float_value;
            }
			else if(strcmp(name, "S_P") == 0)
            {
				float_value = (float)atof(value);
				SPEED_KP = float_value;
            }
            else if(strcmp(name, "S_I") == 0)
            {
				float_value = (float)atof(value);
				SPEED_KI = float_value;
            }
			else if(strcmp(name, "S_D") == 0)
            {
				float_value = (float)atof(value);
				SPEED_KD = float_value;
            }
			else if(strcmp(name, "T_P") == 0)
            {
				float_value = (float)atof(value);
				TURN_KP = float_value;
            }
            else if(strcmp(name, "T_I") == 0)
            {
				float_value = (float)atof(value);
				TURN_KI = float_value;
            }
			else if(strcmp(name, "T_D") == 0)
            {
				float_value = (float)atof(value);
				TURN_KD = float_value;
            }
			Param_SyncToPID();//同步pid到计算变量
			
        }

    }
    
    // ========== 摇杆事件处理 ==========
    else if(strcmp(tag, "joystick") == 0)
    {
        // 左摇杆横向值
        lh = (int8)atoi(strtok(NULL, ","));
        // 左摇杆纵向值
        lv = (int8)atoi(strtok(NULL, ","));
        // 右摇杆横向值
        rh = (int8)atoi(strtok(NULL, ","));
        // 右摇杆纵向值
        rv = (int8)atoi(strtok(NULL, ","));
		
		// 速度环测试摇杆
		Speed_PID.Target = lv / 10.0;	//控制目标速度
		// 转向环测试摇杆
		Turn_PID.Target = rh / 10.0;		
        
//        printf("Joystick: LH=%d, LV=%d, RH=%d, RV=%d\r\n", lh, lv, rh, rv);
//        // 例如：可用于控制机器人方向
    }
    
//    // ========== 其他自定义命令处理 ==========
//    else if(strcmp(tag, "LED") == 0)
//    {
//        action = strtok(NULL, ",");
//        
//        if(action != NULL)
//        {
//            if(strcmp(action, "ON") == 0)
//            {
//                printf("LED ON\r\n");
//                // gpio_set_level(LED_PIN, 1);
//                bluetooth_ch04_send_string("[LED_ON_OK]");
//            }
//            else if(strcmp(action, "OFF") == 0)
//            {
//                printf("LED OFF\r\n");
//                // gpio_set_level(LED_PIN, 0);
//                bluetooth_ch04_send_string("[LED_OFF_OK]");
//            }
//        }
//    }
//    
//    // ========== 未知命令 ==========
//    else
//    {
//        printf("Unknown command: %s\r\n", tag);
//        bluetooth_ch04_send_string("[ERROR_COMMAND]");
//    }
//// (由于蓝牙模块的使用)显示更新标志位
//	oled_Refresh = 1;
	
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙接收处理主函数
// 参数说明     void
// 返回参数     void
// 使用示例     bluetooth_ch04_handle_receive();
// 备注信息     在主循环中定期调用此函数
//-------------------------------------------------------------------------------------------------------------------

void bluetooth_ch04_handle_receive (void)
{
    // 检查是否接收到完整数据包
    if(bluetooth_ch04_get_rx_flag() == 1)
    {
        // 读取接收缓冲区中的数据
        bluetooth_rx_length = bluetooth_ch04_read_buffer(bluetooth_rx_buffer, sizeof(bluetooth_rx_buffer) - 1);
        
        if(bluetooth_rx_length > 0)
        {
            // 确保字符串以 null 结尾
            bluetooth_rx_buffer[bluetooth_rx_length] = '\0';
            
//            // 调试输出：显示原始接收数据
//            printf("BT RX: [");
//            for(uint32 i = 0; i < bluetooth_rx_length; i++)
//            {
//                printf("%c", bluetooth_rx_buffer[i]);
//            }
//            printf("]\r\n");
            
            // 处理数据
            bluetooth_ch04_process_data(bluetooth_rx_buffer, bluetooth_rx_length);
        }
        
        // 清除接收标志位（备用，bluetooth_ch04_read_buffer已清除）
        bluetooth_ch04_clear_rx_flag();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙初始化
// 参数说明     void
// 返回参数     void
// 使用示例     bluetooth_ch04_init_example();
// 备注信息     具有调试性质
//-------------------------------------------------------------------------------------------------------------------

void bluetooth_ch04_init_example (void)
{
    bluetooth_ch04_init();
    printf("Bluetooth CH-04 initialized!\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙测试函数
// 参数说明     void
// 返回参数     void
// 使用示例     bluetooth_ch04_test();
// 备注信息     可在主函数中调用进行功能测试
//-------------------------------------------------------------------------------------------------------------------

void bluetooth_ch04_test (void)
{
    // 测试发送各种类型的数据
    static uint32 test_count = 0;
    
    test_count++;
    
    if(test_count % 100 == 0)
    {
        printf("\r\n--- Bluetooth Test ---\r\n");
        printf("Waiting for BT commands...\r\n");
        printf("Send: [key,1,up] or [slider,1,50] or [joystick,10,20,30,40]\r\n");
    }
}
