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

#ifndef _AI_TUNING_H_
#define _AI_TUNING_H_

#include "zf_common_typedef.h"

// ========== UART配置 ==========
#define AI_TUNING_INDEX                   ( UART_1 )                      // 使用 Debug UART
#define AI_TUNING_BUAD_RATE               ( 115200 )                     // 波特率 115200
#define AI_TUNING_TX_PIN                  ( UART1_TX_A9  )                // TX引脚
#define AI_TUNING_RX_PIN                  ( UART1_RX_A10 )                // RX引脚

// ========== 缓冲区配置 ==========
#define AI_TUNING_BUFFER_SIZE             ( 100 )                         // 接收缓冲区大小
#define AI_TUNING_TX_BUFFER_SIZE          ( 512 )                         // 发送FIFO缓冲区大小（非阻塞发送）

// ========== 协议格式定义 ==========
#define AI_TUNING_PACKET_HEADER           ( '[' )                         // 数据包头
#define AI_TUNING_PACKET_TAIL             ( ']' )                         // 数据包尾
#define AI_TUNING_PACKET_MAX_DATA_LEN     ( 96 )                          // 最大数据长度

// ========== PID 环选择配置 ==========
// 选择要调参的 PID 环（修改下面的宏来选择）
// 1 = 角速度环 (Rate PID)
// 2 = 角度环 (Angle PID) - 默认
// 3 = 速度环 (Speed PID)
// 4 = 转向环 (Turn PID)
#define AI_TUNING_TARGET_PID_LOOP         ( 2 )

// ========== 函数声明 ==========
uint32  ai_tuning_send_byte           (const uint8 data);
uint32  ai_tuning_send_buffer         (const uint8 *buff, uint32 len);
uint32  ai_tuning_send_string         (const char *str);
uint32  ai_tuning_read_buffer         (uint8 *buff, uint32 len);
uint8   ai_tuning_get_rx_flag         (void);
void    ai_tuning_clear_rx_flag       (void);
void    ai_tuning_uart_callback       (void);
void    ai_tuning_handle_receive      (void);
void    ai_tuning_process_data        (uint8 *data_packet, uint32 length);
uint8   ai_tuning_init                (void);
uint32  ai_tuning_printf              (const char *format, ...);
void    ai_tuning_send_flash_params   (void);
uint8   ai_tuning_tx_dequeue          (uint8 *data);                  // 从TX FIFO取出一字节（供TX中断使用）

#endif
