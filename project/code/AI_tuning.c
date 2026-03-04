/*********************************************************************************************************************
* AI 调参模块 - 专门用于与上位机调参脚本通信
* 基于逐飞 MM32F327X-G8P 开源库
* 
* 功能说明：
* 1. 通过 Debug UART (UART1) 与上位机通信
* 2. 接收来自上位机的 PID 参数修改指令
* 3. 格式与蓝牙模块一致：[slider,Kp_2,23.5]
* 4. 输出平衡车数据格式：timestamp,setpoint,input,pwm,error,angle,rate,speed,Kp,Ki,Kd
********************************************************************************************************************/

#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_driver_gpio.h"
#include "zf_driver_uart.h"
#include "zf_driver_delay.h"

#include "AI_tuning.h"
#include "param_config.h"
#include "param_storage.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// ========== 接收相关静态变量 ==========
static  uint8           ai_tuning_rx_packet[AI_TUNING_BUFFER_SIZE];   // 接收缓冲数组
static  uint8           ai_tuning_rx_flag = 0;                             // 接收标志位
static  uint8           ai_tuning_rx_state = 0;                            // 接收状态机
static  uint8           ai_tuning_rx_index = 0;                            // 接收数据索引
static  uint8           ai_tuning_data = 0;                                // 临时数据变量

// ========== 发送FIFO（非阻塞发送）==========
static  fifo_struct     ai_tuning_tx_fifo;
static  uint8           ai_tuning_tx_buffer[AI_TUNING_TX_BUFFER_SIZE];    // 发送FIFO缓冲区

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AI 调参模块 发送单个字节（非阻塞）
// 参数说明     data            8bit 数据
// 返回参数     uint32          1-成功入队 0-发送FIFO已满
// 使用示例     ai_tuning_send_byte(0x5A);
// 备注信息     非阻塞：数据写入发送FIFO后立即返回，由TX中断完成实际发送
//-------------------------------------------------------------------------------------------------------------------
uint32 ai_tuning_send_byte (const uint8 data)
{
    fifo_state_enum result = fifo_write_element(&ai_tuning_tx_fifo, data);
    if(result == FIFO_SUCCESS)
        uart_tx_interrupt(AI_TUNING_INDEX, 1);                                  // 使能TX中断触发发送
    return (result == FIFO_SUCCESS) ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AI 调参模块 发送数组（非阻塞）
// 参数说明     buff            需要发送的数据地址
// 参数说明     len             发送长度
// 返回参数     uint32          返回未能入队的字节数（0表示全部入队成功）
// 使用示例     ai_tuning_send_buffer(buff, 16);
// 备注信息     非阻塞：数据写入发送FIFO后立即返回，由TX中断完成实际发送
//-------------------------------------------------------------------------------------------------------------------
uint32 ai_tuning_send_buffer (const uint8 *buff, uint32 len)
{
    zf_assert(NULL != buff);
    uint32 i;
    for(i = 0; i < len; i++)
    {
        if(fifo_write_element(&ai_tuning_tx_fifo, buff[i]) != FIFO_SUCCESS)
            break;                                                              // FIFO已满，停止入队
    }
    if(i > 0)
        uart_tx_interrupt(AI_TUNING_INDEX, 1);                                  // 使能TX中断触发发送
    return len - i;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AI 调参模块 发送字符串（非阻塞）
// 参数说明     *str            要发送的字符串地址
// 返回参数     uint32          返回未能入队的字节数（0表示全部入队成功）
// 使用示例     ai_tuning_send_string("Hello AI Tuning!");
// 备注信息     非阻塞：数据写入发送FIFO后立即返回，由TX中断完成实际发送
//-------------------------------------------------------------------------------------------------------------------
uint32 ai_tuning_send_string (const char *str)
{
    zf_assert(NULL != str);
    uint32 i = 0;
    while(str[i] != '\0')
    {
        if(fifo_write_element(&ai_tuning_tx_fifo, (uint8)str[i]) != FIFO_SUCCESS)
            break;                                                              // FIFO已满，停止入队
        i++;
    }
    if(i > 0)
        uart_tx_interrupt(AI_TUNING_INDEX, 1);                                  // 使能TX中断触发发送
    return (str[i] != '\0') ? 1 : 0;                                           // 返回0表示全部入队，1表示FIFO溢出
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AI 调参模块 读取接收的数据
// 参数说明     buff            存储的数据地址
// 参数说明     len             读取的长度
// 返回参数     uint32          实际读取字节数
// 使用示例     ai_tuning_read_buffer(buff, 16);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint32 ai_tuning_read_buffer (uint8 *buff, uint32 len)
{
    zf_assert(NULL != buff);
    
    if(ai_tuning_rx_flag == 0)
    {
        return 0;  // 没有接收到完整数据包
    }
    
    uint32 data_len = (ai_tuning_rx_index < len) ? ai_tuning_rx_index : len;
    
    // 复制数据到缓冲区
    if(data_len > 0)
    {
        for(uint32 i = 0; i < data_len; i++)
        {
            buff[i] = ai_tuning_rx_packet[i];
        }
    }
    
    // 清除标志位，准备接收下一个数据包
    ai_tuning_rx_flag = 0;
    
    return data_len;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AI 调参模块 获取接收标志位
// 参数说明     void
// 返回参数     uint8           1-接收到完整数据包 0-未接收到数据包
// 使用示例     if(ai_tuning_get_rx_flag()) { ... }
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 ai_tuning_get_rx_flag (void)
{
    return ai_tuning_rx_flag;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AI 调参模块 清除接收标志位
// 参数说明     void
// 返回参数     void
// 使用示例     ai_tuning_clear_rx_flag();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void ai_tuning_clear_rx_flag (void)
{
    ai_tuning_rx_flag = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AI 调参模块 从发送FIFO取出一个字节（供TX中断使用）
// 参数说明     data            取出字节的存储地址
// 返回参数     uint8           1-取出成功 0-FIFO为空
// 使用示例     在 UART1_IRQHandler 的TX中断中调用
// 备注信息     该函数仅应在TX中断服务函数中调用，实现非阻塞发送
//-------------------------------------------------------------------------------------------------------------------
uint8 ai_tuning_tx_dequeue (uint8 *data)
{
    return (fifo_read_element(&ai_tuning_tx_fifo, data, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS) ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AI 调参模块 数据处理 - 处理接收到的数据包
// 参数说明     data_packet     接收到的数据包（不含头尾）
// 参数说明     length          数据包长度
// 返回参数     void
// 使用示例     ai_tuning_process_data(rx_buffer, 20);
// 备注信息     这个函数参考了蓝牙模块的数据处理逻辑
//-------------------------------------------------------------------------------------------------------------------
void ai_tuning_process_data (uint8 *data_packet, uint32 length)
{
    // 局部变量声明
    char *tag;				//标签
    char *name;				//名称
    char *value;			//值
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
    
    // ========== 请求 Flash 参数指令 ==========
    if(strcmp(tag, "get_flash_params") == 0)
    {
        ai_tuning_send_flash_params();
        return;
    }
    
    // ========== 滑杆事件处理 - PID 参数调整 ==========
    if(strcmp(tag, "slider") == 0)
    {
        name = strtok(NULL, ",");
        value = strtok(NULL, ",");
        
        if(name != NULL && value != NULL)
        {
            if(strcmp(name, "Kp_1") == 0)
            {
                float_value = (float)atof(value);
                RATE__KP = float_value;
            }
            else if(strcmp(name, "Ki_1") == 0)
            {
                float_value = (float)atof(value);
                RATE__KI = float_value;
            }
            else if(strcmp(name, "Kd_1") == 0)
            {
                float_value = (float)atof(value);
                RATE__KD = float_value;
            }
            
            else if(strcmp(name, "Kp_2") == 0)
            {
                float_value = (float)atof(value);
                ANGLE_KP = float_value;
            }
            else if(strcmp(name, "Ki_2") == 0)
            {
                float_value = (float)atof(value);
                ANGLE_KI = float_value;
            }
            else if(strcmp(name, "Kd_2") == 0)
            {
                float_value = (float)atof(value);
                ANGLE_KD = float_value;
            }
            
            else if(strcmp(name, "Kp_3") == 0)
            {
                float_value = (float)atof(value);
                SPEED_KP = float_value;
            }
            else if(strcmp(name, "Ki_3") == 0)
            {
                float_value = (float)atof(value);
                SPEED_KI = float_value;
            }
            else if(strcmp(name, "Kd_3") == 0)
            {
                float_value = (float)atof(value);
                SPEED_KD = float_value;
            }
            
            else if(strcmp(name, "Kp_4") == 0)
            {
                float_value = (float)atof(value);
                TURN__KP = float_value;
            }
            else if(strcmp(name, "Ki_4") == 0)
            {
                float_value = (float)atof(value);
                TURN__KI = float_value;
            }
            else if(strcmp(name, "Kd_4") == 0)
            {
                float_value = (float)atof(value);
                TURN__KD = float_value;
            }
            
            else if(strcmp(name, "Kp_5") == 0)
            {
                float_value = (float)atof(value);
                TRACK_KP = float_value;
            }
            else if(strcmp(name, "Ki_5") == 0)
            {
                float_value = (float)atof(value);
                TRACK_KI = float_value;
            }
            else if(strcmp(name, "Kd_5") == 0)
            {
                float_value = (float)atof(value);
                TRACK_KD = float_value;
            }
            
            // 同步PID到计算变量
            Param_SyncToPID();
            
            // 保存参数到Flash，确保界面显示最新参数
            Param_Save();
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AI 调参模块 串口中断回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     
// 备注信息     该函数在 ISR 文件的串口中断程序被调用
//-------------------------------------------------------------------------------------------------------------------
void ai_tuning_uart_callback (void)
{
    // 读取一个字节数据
    if(uart_query_byte(AI_TUNING_INDEX, &ai_tuning_data) == 1)
    {
        // 状态机处理接收数据
        switch(ai_tuning_rx_state)
        {
            case 0:  // 等待包头
                if(ai_tuning_data == AI_TUNING_PACKET_HEADER && ai_tuning_rx_flag == 0)
                {
                    ai_tuning_rx_state = 1;
                    ai_tuning_rx_index = 0;
                }
                break;
                
            case 1:  // 接收数据
                if(ai_tuning_data == AI_TUNING_PACKET_TAIL)
                {
                    // 接收到包尾，标记数据包完成
                    ai_tuning_rx_state = 0;
                    ai_tuning_rx_packet[ai_tuning_rx_index] = '\0';  // 字符串结束符
                    ai_tuning_rx_flag = 1;  // 标记接收完成
                }
                else
                {
                    // 继续接收数据，防止缓冲区溢出
                    if(ai_tuning_rx_index < (AI_TUNING_BUFFER_SIZE - 1))
                    {
                        ai_tuning_rx_packet[ai_tuning_rx_index] = ai_tuning_data;
                        ai_tuning_rx_index++;
                    }
                    else
                    {
                        // 缓冲区溢出，重置状态
                        ai_tuning_rx_state = 0;
                        ai_tuning_rx_index = 0;
                    }
                }
                break;
                
            default:
                ai_tuning_rx_state = 0;
                break;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AI 调参模块 接收处理主函数
// 使用示例     ai_tuning_handle_receive();
// 备注信息     在主循环中定期调用此函数
//-------------------------------------------------------------------------------------------------------------------
void ai_tuning_handle_receive (void)
{
    static uint8 rx_buffer[100];
    uint32 rx_length = 0;
    
    // 检查是否接收到完整数据包
    if(ai_tuning_get_rx_flag() == 1)
    {
        // 读取接收缓冲区中的数据
        rx_length = ai_tuning_read_buffer(rx_buffer, sizeof(rx_buffer) - 1);
        
        if(rx_length > 0)
        {
            // 确保字符串以 null 结尾
            rx_buffer[rx_length] = '\0';
            
            // 处理数据
            ai_tuning_process_data(rx_buffer, rx_length);
        }
        
        // 清除接收标志位
        ai_tuning_clear_rx_flag();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AI 调参模块 初始化
// 参数说明     void
// 返回参数     uint8           初始化状态 0-成功 1-失败
// 使用示例     ai_tuning_init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 ai_tuning_init (void)
{
    uint8 return_state = 0;
    
    // 初始化接收状态机变量
    ai_tuning_rx_state = 0;
    ai_tuning_rx_index = 0;
    ai_tuning_rx_flag = 0;
    
    // 初始化发送 FIFO（非阻塞发送使用）
    fifo_init(&ai_tuning_tx_fifo, FIFO_DATA_8BIT, ai_tuning_tx_buffer, AI_TUNING_TX_BUFFER_SIZE);
    
    // 初始化 UART - 使用 Debug UART (UART1)
    // 注意：Debug UART 可能已经在 debug_init 中初始化了
    // 但为了确保，我们重新初始化一下
    uart_init(AI_TUNING_INDEX, AI_TUNING_BUAD_RATE, AI_TUNING_TX_PIN, AI_TUNING_RX_PIN);
    
    // 开启串口接收中断
    uart_rx_interrupt(AI_TUNING_INDEX, 1);
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AI 调参模块 格式化输出
// 参数说明     format          格式化字符串（同printf）
// 参数说明     ...             可变参数（数字、字符串等）
// 返回参数     uint32          发送的字节数（失败返回0）
// 使用示例     ai_tuning_printf("%.2f,%.2f,%.2f\n", target, actual, pwm);
//-------------------------------------------------------------------------------------------------------------------
uint32 ai_tuning_printf(const char *format, ...)
{
    // 1. 定义临时缓冲区存储格式化后的内容（大小可根据需求调整）
    char temp_buf[256] = {0};
    
    // 2. 检查格式化字符串是否为空（不用断言，直接返回错误）
    if(format == NULL)
    {
        return 0;
    }
    
    // 3. 处理可变参数，格式化内容到缓冲区
    va_list args;
    va_start(args, format);
    // vsnprintf：安全格式化，避免缓冲区溢出
    int len = vsnprintf(temp_buf, sizeof(temp_buf) - 1, format, args);
    va_end(args);
    
    // 4. 格式化失败/缓冲区不足时返回0
    if(len <= 0 || len >= (int)sizeof(temp_buf) - 1)
    {
        return 0;
    }
    
    // 5. 通过串口发送格式化后的字符串（非阻塞，写入发送FIFO）
    uint32 i = 0;
    while(temp_buf[i] != '\0')
    {
        if(fifo_write_element(&ai_tuning_tx_fifo, (uint8)temp_buf[i]) != FIFO_SUCCESS)
            break;                                                              // FIFO已满，停止入队
        i++;
    }
    if(i > 0)
        uart_tx_interrupt(AI_TUNING_INDEX, 1);                                  // 使能TX中断触发发送
    
    return (uint32)len;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AI 调参模块 上报Flash中存储的所有PID参数
// 返回参数     void
// 使用示例     ai_tuning_send_flash_params();
// 备注信息     在系统启动时调用，将所有PID环的参数发送给上位机
//-------------------------------------------------------------------------------------------------------------------
void ai_tuning_send_flash_params(void)
{
    ai_tuning_send_string("# FLASH_PARAMS_START\r\n");
    
    // 上报当前调整的 PID 环
    ai_tuning_printf("# TARGET_LOOP,%d\r\n", AI_TUNING_TARGET_PID_LOOP);
    
    // 上报角速度环 (Rate PID) - 角速度环PID参数
    ai_tuning_printf("# PID_RATE,Kp=%.4f,Ki=%.4f,Kd=%.4f\r\n", 
                     Rate__PID.Kp, Rate__PID.Ki, Rate__PID.Kd);
    
    // 上报角度环 (Angle PID) - 角度环PID参数
    ai_tuning_printf("# PID_ANGLE,Kp=%.4f,Ki=%.4f,Kd=%.4f\r\n", 
                     Angle_PID.Kp, Angle_PID.Ki, Angle_PID.Kd);
    
    // 上报速度环 (Speed PID) - 速度环PID参数
    ai_tuning_printf("# PID_SPEED,Kp=%.4f,Ki=%.4f,Kd=%.4f\r\n", 
                     Speed_PID.Kp, Speed_PID.Ki, Speed_PID.Kd);
    
    // 上报转向环 (Turn PID) - 转向环PID参数
    ai_tuning_printf("# PID_TURN,Kp=%.4f,Ki=%.4f,Kd=%.4f\r\n", 
                     Turn__PID.Kp, Turn__PID.Ki, Turn__PID.Kd);
    
    // 上报循迹环 (Track PID) - 循迹环PID参数
    ai_tuning_printf("# PID_TRACK,Kp=%.4f,Ki=%.4f,Kd=%.4f\r\n", 
                     Track_PID.Kp, Track_PID.Ki, Track_PID.Kd);
    
    // 上报航向角环 (Head PID) - 航向角环PID参数
    ai_tuning_printf("# PID_HEAD,Kp=%.4f,Ki=%.4f,Kd=%.4f\r\n", 
                     Head__PID.Kp, Head__PID.Ki, Head__PID.Kd);
    
    ai_tuning_send_string("# FLASH_PARAMS_END\r\n");
}

// 串口数据输出格式参考

//		/* 输出数据用于AI调参 (timestamp, setpoint, input, pwm, error, angle, rate, speed, Kp, Ki, Kd) */
//		static uint32_t timestamp = 0;
//		timestamp += 10; /* 每次调用增加10ms (约100Hz) */
//		
//#if AI_TUNING_TARGET_PID_LOOP == 1
//		/* 角速度环 (Rate PID) */
//		ai_tuning_printf("%lu,%3.2f,%3.2f,%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\r\n", 
//		       timestamp, 
//		       Rate__PID.Target, GyroRate_Result, AvePWM, Rate__PID.Target - GyroRate_Result,
//		       Angle_Result, GyroRate_Result, AveSpeed,
//		       Rate__PID.Kp, Rate__PID.Ki, Rate__PID.Kd);
//#elif AI_TUNING_TARGET_PID_LOOP == 2
//		/* 角度环 (Angle PID) */
//		ai_tuning_printf("%lu,%3.2f,%3.2f,%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\r\n", 
//		       timestamp, 
//		       Angle_PID.Target, Angle_Result, AvePWM, Angle_PID.Target - Angle_Result,
//		       Angle_Result, GyroRate_Result, AveSpeed,
//		       Angle_PID.Kp, Angle_PID.Ki, Angle_PID.Kd);
//#elif AI_TUNING_TARGET_PID_LOOP == 3
//		/* 速度环 (Speed PID) */
//		ai_tuning_printf("%lu,%3.2f,%3.2f,%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\r\n", 
//		       timestamp, 
//		       Speed_PID.Target, AveSpeed, AvePWM, Speed_PID.Target - AveSpeed,
//		       Angle_Result, GyroRate_Result, AveSpeed,
//		       Speed_PID.Kp, Speed_PID.Ki, Speed_PID.Kd);
//#elif AI_TUNING_TARGET_PID_LOOP == 4
//		/* 转向环 (Turn PID) */
//		ai_tuning_printf("%lu,%3.2f,%3.2f,%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\r\n", 
//		       timestamp, 
//		       Turn__PID.Target, DifSpeed, DifPWM, Turn__PID.Target - DifSpeed,
//		       Angle_Result, GyroRate_Result, AveSpeed,
//		       Turn__PID.Kp, Turn__PID.Ki, Turn__PID.Kd);
//#else
//		/* 默认输出角度环数据 */
//		ai_tuning_printf("%lu,%3.2f,%3.2f,%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\r\n", 
//		       timestamp, 
//		       Angle_PID.Target, Angle_Result, AvePWM, Angle_PID.Target - Angle_Result,
//		       Angle_Result, GyroRate_Result, AveSpeed,
//		       Angle_PID.Kp, Angle_PID.Ki, Angle_PID.Kd);
//#endif			
		
