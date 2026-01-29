/*********************************************************************************************************************
* MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MM32F327X-G8P 开源库的一部分
* 
* MM32F327X-G8P 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的保证。详见 GPL。
* 
* 您应该在收到本开源库时获得了 GPL 的副本。
* 如果没有，请参阅<http://www.gnu.org/licenses/>
* 
* 文件名称          zf_device_bluetooth_ch04
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.37
* 适用平台          MM32F327X_G8P
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2026-01-27        YUNYYING            CH-04 蓝牙模块驱动移植
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   RX                  查看 zf_device_bluetooth_ch04.h 中 CH04_RX_PINx 宏定义
*                   TX                  查看 zf_device_bluetooth_ch04.h 中 CH04_TX_PINx 宏定义
*                   VCC                 3.3V电源
*                   GND                 电源地
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_driver_gpio.h"
#include "zf_driver_uart.h"
#include "zf_driver_delay.h"
#include "zf_device_type.h"

#include "zf_device_bluetooth_ch04.h"

#include <stdarg.h>


static  fifo_struct     bluetooth_ch04_fifo;
static  uint8           bluetooth_ch04_buffer[BLUETOOTH_CH04_BUFFER_SIZE];      // 数据存放数组
static  uint8           bluetooth_ch04_data = 0;                                // 临时数据变量

// ========== 接收相关静态变量 ==========
static  uint8           bluetooth_ch04_rx_packet[BLUETOOTH_CH04_BUFFER_SIZE];   // 接收缓冲数组
static  uint8           bluetooth_ch04_rx_flag = 0;                             // 接收标志位
static  uint8           bluetooth_ch04_rx_state = 0;                            // 接收状态机
static  uint8           bluetooth_ch04_rx_index = 0;                            // 接收数据索引

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 发送单个字节
// 参数说明     data            8bit 数据
// 返回参数     uint32          1-成功 0-失败
// 使用示例     bluetooth_ch04_send_byte(0x5A);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_ch04_send_byte (const uint8 data)
{
    uart_write_byte(BLUETOOTH_CH04_INDEX, data);
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 发送数组
// 参数说明     buff            需要发送的数据地址
// 参数说明     len             发送长度
// 返回参数     uint32          返回剩余未发送的字节数
// 使用示例     bluetooth_ch04_send_buffer(buff, 16);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_ch04_send_buffer (const uint8 *buff, uint32 len)
{
    zf_assert(NULL != buff);
    uint32 i;
    
    for(i = 0; i < len; i++)
    {
        uart_write_byte(BLUETOOTH_CH04_INDEX, buff[i]);
    }
    
    return 0;  // 返回0表示全部发送成功
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 发送字符串
// 参数说明     *str            要发送的字符串地址
// 返回参数     uint32          返回剩余未发送的字节数
// 使用示例     bluetooth_ch04_send_string("Hello Bluetooth!");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_ch04_send_string (const char *str)
{
    zf_assert(NULL != str);
    uint32 i = 0;
    
    while(str[i] != '\0')
    {
        uart_write_byte(BLUETOOTH_CH04_INDEX, (uint8)str[i]);
        i++;
    }
    
    return 0;  // 返回0表示全部发送成功
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 读取接收的数据
// 参数说明     buff            存储的数据地址
// 参数说明     len             读取的长度
// 返回参数     uint32          实际读取字节数
// 使用示例     bluetooth_ch04_read_buffer(buff, 16);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_ch04_read_buffer (uint8 *buff, uint32 len)
{
    zf_assert(NULL != buff);
    
    if(bluetooth_ch04_rx_flag == 0)
    {
        return 0;  // 没有接收到完整数据包
    }
    
    uint32 data_len = (bluetooth_ch04_rx_index < len) ? bluetooth_ch04_rx_index : len;
    
    // 复制数据到缓冲区
    if(data_len > 0)
    {
        for(uint32 i = 0; i < data_len; i++)
        {
            buff[i] = bluetooth_ch04_rx_packet[i];
        }
    }
    
    // 清除标志位，准备接收下一个数据包
    bluetooth_ch04_rx_flag = 0;
    
    return data_len;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 获取接收标志位
// 参数说明     void
// 返回参数     uint8           1-接收到完整数据包 0-未接收到数据包
// 使用示例     if(bluetooth_ch04_get_rx_flag()) { ... }
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 bluetooth_ch04_get_rx_flag (void)
{
    return bluetooth_ch04_rx_flag;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 清除接收标志位
// 参数说明     void
// 返回参数     void
// 使用示例     bluetooth_ch04_clear_rx_flag();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_ch04_clear_rx_flag (void)
{
    bluetooth_ch04_rx_flag = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 串口中断回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     
// 备注信息     该函数在 ISR 文件的串口中断程序被调用
//              由串口中断服务函数调用 wireless_module_uart_handler() 函数
//              再由 wireless_module_uart_handler() 函数调用本函数
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_ch04_uart_callback (void)
{
    // 读取一个字节数据
    if(uart_query_byte(BLUETOOTH_CH04_INDEX, &bluetooth_ch04_data) == 1)
    {
        // 状态机处理接收数据
        switch(bluetooth_ch04_rx_state)
        {
            case 0:  // 等待包头
                if(bluetooth_ch04_data == BLUETOOTH_CH04_PACKET_HEADER && bluetooth_ch04_rx_flag == 0)
                {
                    bluetooth_ch04_rx_state = 1;
                    bluetooth_ch04_rx_index = 0;
                }
                break;
                
            case 1:  // 接收数据
                if(bluetooth_ch04_data == BLUETOOTH_CH04_PACKET_TAIL)
                {
                    // 接收到包尾，标记数据包完成
                    bluetooth_ch04_rx_state = 0;
                    bluetooth_ch04_rx_packet[bluetooth_ch04_rx_index] = '\0';  // 字符串结束符
                    bluetooth_ch04_rx_flag = 1;  // 标记接收完成
                }
                else
                {
                    // 继续接收数据，防止缓冲区溢出
                    if(bluetooth_ch04_rx_index < (BLUETOOTH_CH04_BUFFER_SIZE - 1))
                    {
                        bluetooth_ch04_rx_packet[bluetooth_ch04_rx_index] = bluetooth_ch04_data;
                        bluetooth_ch04_rx_index++;
                    }
                    else
                    {
                        // 缓冲区溢出，重置状态
                        bluetooth_ch04_rx_state = 0;
                        bluetooth_ch04_rx_index = 0;
                    }
                }
                break;
                
            default:
                bluetooth_ch04_rx_state = 0;
                break;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 初始化
// 参数说明     void
// 返回参数     uint8           初始化状态 0-成功 1-失败
// 使用示例     bluetooth_ch04_init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 bluetooth_ch04_init (void)
{
    uint8 return_state = 0;
    
    // 初始化接收状态机变量
    bluetooth_ch04_rx_state = 0;
    bluetooth_ch04_rx_index = 0;
    bluetooth_ch04_rx_flag = 0;
    
    // 注册串口中断回调函数
    set_wireless_type(BLUETOOTH_CH04, bluetooth_ch04_uart_callback);
    
    // 初始化 FIFO（虽然在这个版本中没有使用，但为了兼容性保留）
    fifo_init(&bluetooth_ch04_fifo, FIFO_DATA_8BIT, bluetooth_ch04_buffer, BLUETOOTH_CH04_BUFFER_SIZE);
    
    // 初始化 UART
    // 注意：CH-04模块默认波特率为9600，与CH9141的115200不同
    uart_init(BLUETOOTH_CH04_INDEX, BLUETOOTH_CH04_BUAD_RATE, BLUETOOTH_CH04_RX_PIN, BLUETOOTH_CH04_TX_PIN);
    
    // 开启串口接收中断
    uart_rx_interrupt(BLUETOOTH_CH04_INDEX, 1);
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙极简版格式化输出（仅保留核心功能）
// 参数说明     format          格式化字符串（同printf）
// 参数说明     ...             可变参数（数字、字符串等）
// 返回参数     uint32          发送的字节数（失败返回0）
// 使用示例     bluetooth_ch04_printf("温度：%.2f，计数：%d\n", 25.5f, 123);
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_ch04_printf(const char *format, ...)
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
    
    // 5. 通过蓝牙发送格式化后的字符串
    uint32 i = 0;
    while(temp_buf[i] != '\0')
    {
        uart_write_byte(BLUETOOTH_CH04_INDEX, (uint8)temp_buf[i]);
        i++;
    }
    
    return (uint32)len;
}
