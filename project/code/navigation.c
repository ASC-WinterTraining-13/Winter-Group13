/********************************************************************************************************************
* 惯性导航系统，移植自https://gitee.com/Emma321/navigation
* 移植涉及文件:
* "nag_flash.c"		"nag_flash.h"
* "navigation.c"	"navigation.h"
* 
* 本文件功能说明：
* 惯性导航上层调用 + 适配你的MPU6050姿态解算（无新建文件）
********************************************************************************************************************/


#include "zf_common_headfile.h"
#include "navigation.h"
#include "mpu6050_Analysis.h"  // 你的MPU6050姿态解算头文件
#include "param_config.h"      // 你的全局变量头文件
#include "nag_flash.h"  

// 新增：适配你的变量（核心映射）
static float last_yaw = 0.0f;  // 上一帧偏航角
static int8 dir_change = 0;    // 偏航角跨0度计数
// 编码器里程系数（根据你的硬件修改！）
// 公式：每速度单位对应实际里程(mm) = (π×轮子直径mm) / (编码器线数×减速比)
#define ENCODER_MILEAGE_COEFF 0.204f  

int32 Nav_read[Read_MaxSize];
Nag N;
// 开源库依赖的全局变量（必须定义）
float angle_Z = 0.0f;          

// 新增：适配你的速度→里程转换函数
static float Get_Left_Mileage(void)
{
    // 速度×5ms×系数 = 5ms内左轮行驶的里程(mm)
    return LeftSpeed * 5.0f * ENCODER_MILEAGE_COEFF;
}

static float Get_Right_Mileage(void)
{
    return RightSpeed * 5.0f * ENCODER_MILEAGE_COEFF;
}

// 新增：惯导适配初始化（替换原gyroOffset_init）
void Nav_Adapter_Init(void)
{
    // 1. 你的MPU6050零飘校准
    MPU6050_Calibration_Start();
    
    // 2. 开源库原有初始化
    Init_Nag();
    
    // 3. 初始化偏航角变量
    last_yaw = 0.0f;
    dir_change = 0;
    angle_Z = 0.0f;
}

// 新增：惯导适配核心函数（主循环调用，处理中断标志位）
void Nav_Adapter_Main(void)
{
    // 1. 检查你的MPU6050解算标志位
    if(mpu6050_analysis_enable)
    {
        // 2. 执行你的姿态解算（更新Yaw_Result）
        MPU6050_Analysis();
        mpu6050_analysis_enable = 0;  // 清除标志位
        
        // 3. 映射你的偏航角到开源库的angle_Z（解决360°循环）
        float current_yaw = Yaw_Result;
        if ((current_yaw - last_yaw) < -350.0f) dir_change++;
        else if ((current_yaw - last_yaw) > 350.0f) dir_change--;
        angle_Z = 360.0f * dir_change + current_yaw;
        last_yaw = current_yaw;
    }
    
    // 4. 调用开源库核心逻辑
    Nag_System();
}

// 惯导读取线程（记录/复现切换）
void Nag_Read()
{
    switch(N.End_f)
    {
        case 0: Run_Nag_Save();  // 记录模式
            break;
        case 1: 
            nag_flash_write_data();   // 调用三次包装接口：写入惯导数据
            N.End_f++;
            break;
        case 2: 
            N.End_f++;          // 结束线程
            break;
    }
}

// 偏航角偏差计算
void Nag_Run()
{
    Run_Nag_GPS();  // 读取目标偏航角
    
    // 停止保护
    if(N.Nag_Stop_f)
    {
        N.Final_Out=0;
        return;
    }
    
    // 计算当前偏航角与目标偏航角的偏差
    N.Final_Out=angle_Z-N.Angle_Run;
}

// 偏航角记录（调用三次包装Flash接口）
void Run_Nag_Save()
{
    // 修改：适配你的速度里程，删除int强制转换
    N.Mileage_All += (Get_Right_Mileage() + Get_Left_Mileage()) / 2.0f;
    
    // 每页存满后写入Flash
    if(N.size > MaxSize)
    {
        nag_flash_write_data();  // 调用三次包装接口：写入数据
        N.size=0;
        N.Flash_sector_index--;
        zf_assert(N.Flash_sector_index >= Nag_Start_Sector && N.Flash_sector_index <= Nag_End_Sector); // 越界保护
    }

    // 达到里程阈值，记录偏航角
    if(N.Mileage_All >= Nag_Set_mileage)
    {
        // 偏航角放大100倍（避免浮点存储）
        int32 Save=(int32)(Nag_Yaw*100);
        flash_union_buffer[N.size++].int32_type = Save;
        N.Save_index++;

        // 重置累计里程
        if(N.Mileage_All > 0) 
            N.Mileage_All -= Nag_Set_mileage;
        else 
            N.Mileage_All += Nag_Set_mileage;
    }
}

// 偏航角复现（读取Flash）
void Run_Nag_GPS()
{
    // 修改：适配你的速度里程，删除int强制转换
    N.Mileage_All += (Get_Right_Mileage() + Get_Left_Mileage()) / 2.0f;
    uint16 prospect=0;
    
    // 达到里程阈值，读取目标偏航角
    if(N.Mileage_All >= Nag_Set_mileage)
    {
        // 越界保护
        if(N.Run_index> N.Save_index-2)
        {
            N.Nag_Stop_f++;
            return;
        }
        
        N.Run_index++;
        prospect=N.Run_index;
        
        // 前瞻越界保护
        if(prospect >N.Save_index-2)  
            prospect=N.Save_index-2;
        
        // 读取目标偏航角（缩小100倍）
        N.Angle_Run = (Nav_read[prospect]/100.0f);  
        
        // 重置累计里程
        if(N.Mileage_All > 0) 
            N.Mileage_All -= Nag_Set_mileage;
        else 
            N.Mileage_All += Nag_Set_mileage;
    }
}

// 惯导初始化（保留原函数）
void Init_Nag()
{
    memset(&N, 0, sizeof(N));
    N.Flash_sector_index = Nag_Start_Sector; // 从40扇区开始
    N.Flash_page_index = Nag_Start_Page;     // 扇区内从3页开始
    flash_buffer_clear();
}

// 惯导核心执行函数（放入中断）
void Nag_System()
{
    // 停止保护
    if(!N.Nag_SystemRun_Index || N.Nag_Stop_f )  
        return;

    switch(N.Nag_SystemRun_Index)
    {
        case 1 : Nag_Read();    // 记录/复现切换
            break;
        case 3: Nag_Run();      // 偏差计算
            break;
    }
}

// 一次性读取Flash数据到数组
void NagFlashRead()
{
    if(N.Save_state) return;
    
    nag_flash_read_data();  // 调用三次包装接口：读取惯导数据
    uint8 page_trun=0;
    
    for(int index=0;index <= N.Save_index;index++)
    {
        if(index >= N.Save_index)
        {
            N.Save_state=1;
            break;
        }
        
        int temp_index=index-(MaxSize*page_trun);
        
        // 页切换
        if(temp_index >MaxSize)
        {
            N.Flash_sector_index--;
            page_trun++;
            nag_flash_read_data(); // 调用三次包装接口：读取新页数据
        }
        
        // 读取数据到数组
        Nav_read[index]= flash_union_buffer[index-(MaxSize*page_trun)].int32_type;
    }
    
    N.Nag_SystemRun_Index++;
}