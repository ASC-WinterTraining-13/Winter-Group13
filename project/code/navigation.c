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
            //Buzzer_check(500);   	// 蜂鸣器确认执行
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
    
    // 先判断误差超限，再修正范围（避免冗余）
    if (fabs(N.Final_Out) > 180.0f)
    {
        N.Nag_Stop_f = 1; // 误差超限，停止纠偏
        N.Final_Out = 0;
        return;
    }
    
    // 偏航角误差-180°~180°范围修正，处理350°-10°=340°→-20°
    if (N.Final_Out > 180.0f) N.Final_Out -= 360.0f;
    else if (N.Final_Out < -180.0f) N.Final_Out += 360.0f;
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
            N.Flash_page_index >= Nag_End_Page && 
            LOGIC_PAGE_TO_SECTOR(N.Flash_page_index) >= NAG_MIN_SECTOR && 
            LOGIC_PAGE_TO_SECTOR(N.Flash_page_index) <= NAG_MAX_SECTOR
        );
    }

    // 达到里程阈值则存储偏航角
    if(N.Mileage_All >= Nag_Set_mileage)
    {
        // 偏航角放大100倍转int32，避免浮点存储误差
        int32 Save = (int32)(Nag_Yaw * 100.0f);
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

    if(N.Mileage_All >= Nag_Set_mileage)
    {
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
        uint8 target_page = Nag_Start_Page - page_offset; // 目标逻辑页
        
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

        // 从当前页缓冲区读取目标偏航角（还原为float）
        N.Angle_Run = flash_union_buffer[prospect % MaxSize].int32_type / 100.0f;

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
    N.Flash_page_index = Nag_Start_Page;
    current_read_page = 0xFF; // 初始化当前读取页为无效值
    flash_buffer_clear();
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
