#include "zf_common_headfile.h"
#include "navigation.h"
#include "kalman.h"  
#include "nag_flash.h"  

int32 Nav_read[Read_MaxSize];
Nag N;

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
    // 累计里程（左右编码器平均值）
    N.Mileage_All+=(R_Mileage+L_Mileage)/2.0;
    
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
    // 累计里程
    N.Mileage_All+=(int)((R_Mileage+L_Mileage)/2.0);
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

// 惯导初始化
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
