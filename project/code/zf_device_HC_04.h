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
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          zf_device_HC_04
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.37
* 适用平台          MM32F327X_G8P
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2024-05-13        AI Assistant        first version for HC-04
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   RX                  查看 zf_device_HC_04.h 中 HC_04_RX_PINx 宏定义
*                   TX                  查看 zf_device_HC_04.h 中 HC_04_TX_PINx 宏定义
*                   VCC                 3.3V或5V电源（根据模块版本）
*                   GND                 电源地
*                   EN                  接高电平或悬空（进入AT模式时需接低电平）
*                   STATE               状态指示引脚（可选）
*                   ------------------------------------
* 注意：HC-04 模块没有硬件流控(RTS/CTS)，因此发送数据时不需要等待流控信号
*       HC-04 默认波特率为9600，可通过AT命令修改
********************************************************************************************************************/
#ifndef _ZF_DEVICE_HC_04_H_
#define _ZF_DEVICE_HC_04_H_

#include "zf_common_typedef.h"

#define HC_04_INDEX                  ( UART_6 )                      // HC-04 模块建议使用的串口号（可根据需要修改）
#define HC_04_BAUD_RATE              ( 9600 )                      // HC-04 模块波特率（9600/38400/115200等，需与模块设置一致）
#define HC_04_TX_PIN                 ( UART6_TX_C6 )                 // 单片机TX引脚，连接模块RX
#define HC_04_RX_PIN                 ( UART6_RX_C7 )                 // 单片机RX引脚，连接模块TX
#define HC_04_STATE_PIN              ( C13 )                         // HC-04 模块的状态引脚（可选，用于检测连接状态）

#define HC_04_BUFFER_SIZE            ( 128 )                         // HC-04 模块使用的接收缓冲区大小
#define HC_04_SEND_TIMEOUT           ( 100 )                         // 发送超时时间（毫秒）
#define HC_04_RECEIVE_TIMEOUT        ( 100 )                         // 接收超时时间（毫秒）

// 状态定义
#define HC_04_STATE_DISCONNECTED     ( 0 )                           // 未连接状态
#define HC_04_STATE_CONNECTED        ( 1 )                           // 已连接状态

// HC-04 模块函数声明
uint32      hc_04_send_byte          (const uint8 data);
uint32      hc_04_send_buffer        (const uint8 *buff, uint32 len);
uint32      hc_04_send_string        (const char *str);
void        hc_04_send_image         (const uint8 *image_addr, uint32 image_size);

uint32      hc_04_read_buffer        (uint8 *buff, uint32 len);
uint8       hc_04_read_byte          (void);
uint32      hc_04_get_rx_length      (void);
void        hc_04_clear_rx_buffer    (void);

uint8       hc_04_get_state          (void);
uint8       hc_04_check_connection   (void);

void        hc_04_uart_callback      (void);

uint8       hc_04_init               (uint32 baud_rate);
uint8       hc_04_deinit             (void);

// AT 命令相关函数（HC-04 支持 AT 命令模式）
uint8       hc_04_enter_at_mode      (void);
uint8       hc_04_exit_at_mode       (void);
uint8       hc_04_send_at_command    (const char *at_cmd, char *response, uint32 timeout);
uint8       hc_04_set_name           (const char *name);
uint8       hc_04_set_pin            (const char *pin);
uint8       hc_04_set_baud_rate      (uint32 baud_rate);

#endif