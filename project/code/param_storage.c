#include "param_storage.h"
#include "param_config.h"

// 默认参数值（首次使用或恢复出厂设置时使用）
static const float DEFAULT_PARAMS[15] = {
    // Angle_PID(索引 0-2)
    453.0f, 0.0f, 340.0f,
    
    // Speed_PID (索引 3-5)
    -78.1f, -0.95f, 0.0f,
    
    // Turn_pid (索引 6-8)
    100.0f, 0.0f, 0.0f,
    
    // 4_pid (索引 9-11)
    0.0f, 0.0f, 0.0f,
    
    // 5_pid (索引 12-14)
    0.0f, 0.0f, 0.0f
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     加载默认参数到缓冲区
// 使用示例     load_default();  // 内部函数，用户不直接调用
//-------------------------------------------------------------------------------------------------------------------

static void load_default(void)
{
    for(uint8 i = 0; i < 15; i++)
    {
        flash_union_buffer[i].float_type = DEFAULT_PARAMS[i];
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化参数系统
// 使用示例     Param_Init();  // 在 main() 函数开始时调用一次
// 备注信息     首次使用时自动写入默认值，后续启动自动从Flash加载
//-------------------------------------------------------------------------------------------------------------------

void Param_Init(void)
{
    if(flash_check(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE))  // 返回1 = 有数据
    {
        // 从Flash读取到缓冲区		
        flash_read_page_to_buffer(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE);
    }
    else  // 返回0 = 没有数据（首次使用）
    {
        // 加载默认值到缓冲区
        load_default();
        
        // 写入Flash
        flash_write_page_from_buffer(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE);		
    }
		// 角度环pid
		Angle_PID.Kp = ANGLE_KP;
		Angle_PID.Ki = ANGLE_KI;
		Angle_PID.Kd = ANGLE_KD;
		
		// 速度环pid
		Speed_PID.Kp = SPEED_KP;
        Speed_PID.Ki = SPEED_KI;
        Speed_PID.Kd = SPEED_KD;
		
		// 转向环pid
		Turn_PID.Kp = TURN_KP;
        Turn_PID.Ki = TURN_KI;
        Turn_PID.Kd = TURN_KD;
		
		// 循迹环pid
		Track_PID.Kp = TRACK_KP;
        Track_PID.Ki = TRACK_KI;
        Track_PID.Kd = TRACK_KD;
		
		//5
		TEMP_888_FUNC_5_PID.Kp = TEMP_888_FUNC_5_KP;
        TEMP_888_FUNC_5_PID.Ki = TEMP_888_FUNC_5_KI;
        TEMP_888_FUNC_5_PID.Kd = TEMP_888_FUNC_5_KD;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存参数到Flash
// 使用示例     Param_Save();  // 在退出参数修改时调用
// 备注信息     将当前缓冲区的数据写入Flash
//-------------------------------------------------------------------------------------------------------------------

void Param_Save(void)
{
    flash_write_page_from_buffer(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     擦除Flash存储区域
// 使用示例     Param_Erase();  // 擦除后下次启动会重新写入默认值（默认值在本文件可以找到）
// 备注信息     慎用！会清空所有保存的参数
//-------------------------------------------------------------------------------------------------------------------

void Param_Erase(void)
{
    flash_erase_page(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     同步pid参数至缓存区（仍然需要调用Param_Save();）
// 使用示例     Param_SyncToPID();
// 备注信息     一般是配合蓝牙调参使用的（针对需要将关键pid参数存入flash的要求）
//-------------------------------------------------------------------------------------------------------------------

void Param_SyncToPID(void)
{
    // 角度环pid
    Angle_PID.Kp = ANGLE_KP;
    Angle_PID.Ki = ANGLE_KI;
    Angle_PID.Kd = ANGLE_KD;
    
    // 速度环pid
    Speed_PID.Kp = SPEED_KP;
    Speed_PID.Ki = SPEED_KI;
    Speed_PID.Kd = SPEED_KD;
    
    // 转向环pid
    Turn_PID.Kp = TURN_KP;
    Turn_PID.Ki = TURN_KI;
    Turn_PID.Kd = TURN_KD;
    
    // 循迹环pid
    Track_PID.Kp = TRACK_KP;
    Track_PID.Ki = TRACK_KI;
    Track_PID.Kd = TRACK_KD;
//    
//    //5
//    TEMP_888_FUNC_5_PID.Kp = TEMP_888_FUNC_5_KP;
//    TEMP_888_FUNC_5_PID.Ki = TEMP_888_FUNC_5_KI;
//    TEMP_888_FUNC_5_PID.Kd = TEMP_888_FUNC_5_KD;
}
