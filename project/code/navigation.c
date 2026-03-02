/********************************************************************************************************************
* 惯性导航系统，移植自https://gitee.com/Emma321/navigation
* 移植涉及文件:
* "nag_flash.c"		"nag_flash.h"
* "navigation.c"	"navigation.h"
* 
* 本文件功能说明：
* 惯性导航上层调用
********************************************************************************************************************/

// 原有的开源库声明
/*
 * navigation.c
 *
 *  Created on: 2024年10月16日
 *      Author: Monst
 */

#include "zf_common_headfile.h"
#include "navigation.h"
#include "navi_flash.h"
#include "BuzzerAndLED.h"
#include "math.h"

// 全局变量初始化
Nag N;
static uint8 current_read_page = 0xFF; // 静态变量存储当前读取的Flash页（替代N.current_read_page）

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取偏航角的线程函数
// 参数说明     通过切换N.End_f切换状态：0=记录中，1=写入最后一页，2=结束记录
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Read(void)
{
    switch(N.End_f)
    {
        case 0:
            Run_Nag_Save();  		// 默认执行记录函数
            break;
        case 1:
            flash_Navi_Write();  	// 写入最后一页，保证数据完整
            N.End_f++;
            break;
        case 2:
            N.End_f++;  			// 结束记录
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     生成偏航角偏差值
// 参数说明     N.Final_Out为最终偏差（当前yaw - 目标yaw），供PID使用
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Run(void)
{
    Run_Nag_GPS();  // 偏航角读取复现

    // 完成复现后清零输出，防止旋转
    if(N.Nag_Stop_f)
    {
        N.Final_Out = 0;
        return;
    }

    // 计算误差值（当前yaw - 目标yaw）
    N.Final_Out = Yaw_Result - N.Angle_Run;

    // 偏航角误差-180°~180°范围修正，处理350°-10°=340°→-20°
    if (N.Final_Out > 180.0f) N.Final_Out -= 360.0f;
    else if (N.Final_Out < -180.0f) N.Final_Out += 360.0f;

    // 数据异常保护
    if(!isfinite(N.Final_Out))
    {
        N.Nag_Stop_f = 1;
        N.Final_Out = 0;
        return;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏航角存入Flash
// 参数说明     按5cm里程间隔存储YAW角，浮点数计算保留精度
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_Save(void)
{
    // 累计里程（左右编码器均值，浮点数保留精度）
    N.Mileage_All += (R_Mileage + L_Mileage) / 2.0f;
    
    // 缓冲区满则写入Flash
    if(N.size > MaxSize)
    {
        flash_Navi_Write();
        N.size = 0;   				// 缓冲区索引重置
        N.Flash_page_index--;   	// Flash页号递减
        
        // 严格校验Flash页号在80~120号扇区范围内
        zf_assert(
            N.Flash_page_index >= Nag_Start_Page && 
            LOGIC_PAGE_TO_SECTOR(N.Flash_page_index) >= NAG_MIN_SECTOR && 
            LOGIC_PAGE_TO_SECTOR(N.Flash_page_index) <= NAG_MAX_SECTOR
        );
    }

    // 达到里程阈值则存储偏航角和速度方向
    if(fabs(N.Mileage_All) >= Nag_Set_mileage)
    {
        // 计算相对于起始点的yaw角（减去Yaw_Dif）
        float relative_yaw = Nag_Yaw - N.Yaw_Dif;
		
#ifdef USE_UNWRAPPED_YAW
	// 无边界模式：直接使用原始相对角度
#else
        // 处理-180~180范围
        if (relative_yaw > 180.0f) relative_yaw -= 360.0f;
        else if (relative_yaw < -180.0f) relative_yaw += 360.0f;
#endif
		
        // 计算速度方向（1=前进，-1=后退）
        int8 speed_dir = ((R_Mileage + L_Mileage) / 2.0f) >= 0 ? 1 : -1;
        
        // 偏航角放大100倍转int32，速度方向存储在低8位
        int32 Save = ((int32)(relative_yaw * 100.0f) << 8) | (speed_dir & 0xFF);
        flash_union_buffer[N.size++].int32_type = Save;

        N.Save_index++; // 存储总条数+1

        // 重置里程计（支持正/倒车）
        if(N.Mileage_All > 0) N.Mileage_All -= Nag_Set_mileage;
        else N.Mileage_All += Nag_Set_mileage;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏航角复现（按需读取Flash）
// 参数说明     仅读取当前里程对应的Flash页，降低RAM占用
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_GPS(void)
{
    // 统一为浮点数计算，删除int强制转换，保留里程精度
    N.Mileage_All += (R_Mileage + L_Mileage) / 2.0f;
    uint16 prospect = 0;

    if(fabs(N.Mileage_All) >= Nag_Set_mileage)
    {
        // 无有效录制数据，停止复现
        if(N.Save_index < 2)
        {
            N.Nag_Stop_f = 1;
            return;
        }

        // 复现完成，停止纠偏
        if(N.Run_index > N.Save_index - 2)
        {
            N.Nag_Stop_f++;
            return;
        }
        
        N.Run_index++; // 复现索引+1
        prospect = N.Run_index; // 前瞻索引
        
        // 越界保护
        if(prospect > N.Save_index - 2) prospect = N.Save_index - 2;

        // 计算当前索引对应的Flash逻辑页
        uint16 page_offset = prospect / MaxSize; // 已存储的页数
        uint8 target_page = Nag_End_Page - page_offset; // 目标逻辑页
        
        // 校验目标页是否在80~120号扇区范围内
        if (LOGIC_PAGE_TO_SECTOR(target_page) < NAG_MIN_SECTOR || LOGIC_PAGE_TO_SECTOR(target_page) > NAG_MAX_SECTOR)
        {
            N.Nag_Stop_f = 1; // 越界停止
            return;
        }

        // 仅当目标页≠当前页时，才读取Flash（按需读取）
        if (target_page != current_read_page)
        {
            N.Flash_page_index = target_page;
            flash_Navi_Read(); // 读取当前页数据
            current_read_page = target_page; // 更新当前页
        }

        // 从当前页缓冲区读取目标偏航角和速度方向
        int32 data = flash_union_buffer[prospect % MaxSize].int32_type;
        float relative_angle = (data >> 8) / 100.0f;
        int8 speed_dir = data & 0xFF;
        if (speed_dir > 127) speed_dir -= 256; // 符号扩展
        
        N.Angle_Run = relative_angle + N.Yaw_Dif;
        N.Speed_Dir = speed_dir;
		
#ifdef USE_UNWRAPPED_YAW
	// 无边界模式：直接使用原始相对角度
#else
        // 处理-180~180范围
        if (N.Angle_Run > 180.0f) N.Angle_Run -= 360.0f;
        else if (N.Angle_Run < -180.0f) N.Angle_Run += 360.0f;
#endif
		
        // 重置里程计（支持正/倒车）
        if(N.Mileage_All > 0) N.Mileage_All -= Nag_Set_mileage;
        else N.Mileage_All += Nag_Set_mileage;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯导参数初始化
// 返回参数     void
// 使用示例     放入程序初始化阶段
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Init_Nag(void)
{
    memset(&N, 0, sizeof(N));
    N.Flash_page_index = Nag_End_Page;
    N.Index_R_f = 0;
    current_read_page = 0xFF;
    flash_buffer_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     回放启动预加载（读取录制长度并装载首个目标角）
// 返回参数     uint8 1=无有效数据，0=启动正常
//-------------------------------------------------------------------------------------------------------------------
uint8 Nag_Replay_Start(void)
{
    // 回放开始前清理状态
    N.Nag_Stop_f = 0;
    N.Run_index = 0;
    N.Mileage_All = 0;
    current_read_page = 0xFF;

    // 先读取结束页获取总记录长度，再读取起始逻辑页数据
    N.Flash_page_index = Nag_End_Page;
    N.Index_R_f = 0;
    flash_Navi_Read();

    if(N.Save_index < 2)
    {
        N.Nag_Stop_f = 1;
        N.Angle_Run = Yaw_Result;
        return 1;
    }

    // 预加载首个目标角，避免起步阶段使用未初始化目标角
    N.Flash_page_index = Nag_End_Page;
    flash_Navi_Read();
    {
        float relative_angle = flash_union_buffer[0].int32_type / 100.0f;
        N.Angle_Run = relative_angle + N.Yaw_Dif;
		
#ifdef USE_UNWRAPPED_YAW
		// 无边界模式：直接使用原始相对角度
#else
        if (N.Angle_Run > 180.0f) N.Angle_Run -= 360.0f;
        else if (N.Angle_Run < -180.0f) N.Angle_Run += 360.0f;
#endif
		
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯性导航执行函数（中断调用）
// 参数说明     N.Nag_SystemRun_Index：0=停止，1=记录路径，3=复现路径
// 返回参数     void
// 使用示例     放入定时器中断/主循环
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_System(void)
{
    // 保护逻辑：停止状态/纠偏停止时，不执行
    if(!N.Nag_SystemRun_Index || N.Nag_Stop_f) return;

    switch(N.Nag_SystemRun_Index)
    {
        case 1 : Nag_Read();    // 1=记录路径
            break;
        case 3: Nag_Run();      // 3=复现路径
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     废弃：一次性读取Flash（已替换为按需读取）
// 参数说明     无
// 返回参数     void
// 使用示例     请勿调用
// 备注信息     保留函数定义避免编译错误，内部逻辑清空
//-------------------------------------------------------------------------------------------------------------------
void NagFlashRead(void)
{
    if(N.Save_state) return;
    N.Save_state = 1;
    N.Nag_SystemRun_Index++;
}
