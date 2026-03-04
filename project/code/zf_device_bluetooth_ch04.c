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

// 代码实际上是Claude整合的，DMA转运在测试时并没有生效，但是仍然保留代码接口

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

// ========== 发送FIFO（非阻塞发送）==========
static  fifo_struct     bluetooth_ch04_tx_fifo;
static  uint8           bluetooth_ch04_tx_buffer[BLUETOOTH_CH04_TX_BUFFER_SIZE]; // 发送FIFO缓冲区
static  uint8           bluetooth_ch04_tx_busy = 0;                              // TX是否正在发送（1-发送中 0-空闲）

// ========== 接收相关静态变量 ==========
static  uint8           bluetooth_ch04_rx_packet[BLUETOOTH_CH04_BUFFER_SIZE];   // 接收缓冲数组
static  uint8           bluetooth_ch04_rx_flag = 0;                             // 接收标志位
static  uint8           bluetooth_ch04_rx_state = 0;                            // 接收状态机
static  uint8           bluetooth_ch04_rx_index = 0;                            // 接收数据索引

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 发送单个字节（非阻塞）
// 参数说明     data            8bit 数据
// 返回参数     uint32          1-成功入队 0-发送FIFO已满
// 使用示例     bluetooth_ch04_send_byte(0x5A);
// 备注信息     非阻塞：数据写入发送FIFO后立即返回，由TX中断完成实际发送
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_ch04_send_byte (const uint8 data)
{
    fifo_state_enum result = fifo_write_element(&bluetooth_ch04_tx_fifo, data);
    if(result == FIFO_SUCCESS)
        uart_tx_interrupt(BLUETOOTH_CH04_INDEX, 1);                             // 使能TX中断触发发送
    return (result == FIFO_SUCCESS) ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 发送数组（非阻塞）
// 参数说明     buff            需要发送的数据地址
// 参数说明     len             发送长度
// 返回参数     uint32          返回未能入队的字节数（0表示全部入队成功）
// 使用示例     bluetooth_ch04_send_buffer(buff, 16);
// 备注信息     非阻塞：数据写入发送FIFO后立即返回，由TX中断完成实际发送
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_ch04_send_buffer (const uint8 *buff, uint32 len)
{
    zf_assert(NULL != buff);
    uint32 i;
    for(i = 0; i < len; i++)
    {
        if(fifo_write_element(&bluetooth_ch04_tx_fifo, buff[i]) != FIFO_SUCCESS)
            break;                                                              // FIFO已满，停止入队
    }
    if(i > 0)
    {
        if(!bluetooth_ch04_tx_busy)                                             // TX空闲时，直接发送首字节启动传输
        {
            bluetooth_ch04_tx_busy = 1;
            uint8 first_byte;
            if(fifo_read_element(&bluetooth_ch04_tx_fifo, &first_byte, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS)
            {
                uart_write_byte(BLUETOOTH_CH04_INDEX, first_byte);
            }
            else
            {
                bluetooth_ch04_tx_busy = 0;                                     // 读取失败，恢复空闲状态
            }
        }
        uart_tx_interrupt(BLUETOOTH_CH04_INDEX, 1);                             // 使能TX中断触发发送
    }
    return len - i;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 发送字符串（非阻塞）
// 参数说明     *str            要发送的字符串地址
// 返回参数     uint32          返回未能入队的字节数（0表示全部入队成功）
// 使用示例     bluetooth_ch04_send_string("Hello Bluetooth!");
// 备注信息     非阻塞：数据写入发送FIFO后立即返回，由TX中断完成实际发送
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_ch04_send_string (const char *str)
{
    zf_assert(NULL != str);
    uint32 i = 0;
    while(str[i] != '\0')
    {
        if(fifo_write_element(&bluetooth_ch04_tx_fifo, (uint8)str[i]) != FIFO_SUCCESS)
            break;                                                              // FIFO已满，停止入队
        i++;
    }
    if(i > 0)
    {
        if(!bluetooth_ch04_tx_busy)                                             // TX空闲时，直接发送首字节启动传输
        {
            bluetooth_ch04_tx_busy = 1;
            uint8 first_byte;
            if(fifo_read_element(&bluetooth_ch04_tx_fifo, &first_byte, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS)
            {
                uart_write_byte(BLUETOOTH_CH04_INDEX, first_byte);
            }
            else
            {
                bluetooth_ch04_tx_busy = 0;                                     // 读取失败，恢复空闲状态
            }
        }
        uart_tx_interrupt(BLUETOOTH_CH04_INDEX, 1);                             // 使能TX中断触发发送
    }
    return (str[i] != '\0') ? 1 : 0;                                           // 返回0表示全部入队，1表示FIFO溢出
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
// 函数简介     蓝牙转串口模块 从发送FIFO取出一个字节（供TX中断使用）
// 参数说明     data            取出字节的存储地址
// 返回参数     uint8           1-取出成功 0-FIFO为空
// 使用示例     在 UART6_IRQHandler 的TX中断中调用
// 备注信息     该函数仅应在TX中断服务函数中调用，实现非阻塞发送
//-------------------------------------------------------------------------------------------------------------------
uint8 bluetooth_ch04_tx_dequeue (uint8 *data)
{
    if(fifo_read_element(&bluetooth_ch04_tx_fifo, data, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS)
        return 1;
    bluetooth_ch04_tx_busy = 0;                                                 // FIFO已空，标记TX为空闲
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 清空发送缓冲区并停止发送
// 参数说明     void
// 返回参数     void
// 使用示例     bluetooth_ch04_tx_flush();
// 备注信息     退出调试模式时调用，防止退出后继续发送
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_ch04_tx_flush (void)
{
    uart_tx_interrupt(BLUETOOTH_CH04_INDEX, 0);                                 // 关闭TX中断
    fifo_clear(&bluetooth_ch04_tx_fifo);                                        // 清空发送FIFO
    bluetooth_ch04_tx_busy = 0;                                                 // 标记TX为空闲
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
    bluetooth_ch04_tx_busy = 0;
    
    // 注册串口中断回调函数
    set_wireless_type(BLUETOOTH_CH04, bluetooth_ch04_uart_callback);
    
    // 初始化接收 FIFO（兼容性保留）
    fifo_init(&bluetooth_ch04_fifo, FIFO_DATA_8BIT, bluetooth_ch04_buffer, BLUETOOTH_CH04_BUFFER_SIZE);
    
    // 初始化发送 FIFO（非阻塞发送使用）
    fifo_init(&bluetooth_ch04_tx_fifo, FIFO_DATA_8BIT, bluetooth_ch04_tx_buffer, BLUETOOTH_CH04_TX_BUFFER_SIZE);
    
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
    
    // 5. 通过蓝牙发送格式化后的字符串（非阻塞，写入发送FIFO）
    uint32 i = 0;
    while(temp_buf[i] != '\0')
    {
        if(fifo_write_element(&bluetooth_ch04_tx_fifo, (uint8)temp_buf[i]) != FIFO_SUCCESS)
            break;                                                              // FIFO已满，停止入队
        i++;
    }
    if(i > 0)
    {
        if(!bluetooth_ch04_tx_busy)                                             // TX空闲时，直接发送首字节启动传输（解决首字符丢失问题）
        {
            bluetooth_ch04_tx_busy = 1;
            uint8 first_byte;
            if(fifo_read_element(&bluetooth_ch04_tx_fifo, &first_byte, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS)
            {
                uart_write_byte(BLUETOOTH_CH04_INDEX, first_byte);              // 阻塞发送首字节，确保可靠启动
            }
            else
            {
                bluetooth_ch04_tx_busy = 0;                                     // 读取失败，恢复空闲状态
            }
        }
        uart_tx_interrupt(BLUETOOTH_CH04_INDEX, 1);                             // 使能TX中断发送FIFO中剩余字节
    }
    
    return (uint32)len;
}
