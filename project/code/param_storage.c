#include "param_storage.h"
#include "param_config.h"
#include "math.h"

float param_cache[21] = {0};

// 默认参数值（首次使用或恢复出厂设置时使用）

static const float DEFAULT_PARAMS[21] = {
    // Rate__PID (索引 0-2)
    29.58f, 0.0f, 0.0f,
    
    // Angle_PID (索引 3-5)
    25.9f, 1.49f, 25.0f,
    
    // Speed_PID (索引 6-8)
    0.17f, 0.0f, 0.0f,
    
    // Turn__PID (索引 9-11)
    101.0f, 0.0f, 0.0f,
    
    // Track_PID (索引 12-14)
    0.0f, 0.0f, 0.0f,
	
	// Head__PID (索引 15-17)
	4.0f, 0.0f, 0.0f,
	
	// Posi__PID (索引 18-20)
	1.0f, 0.0f, 0.0f
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     加载默认参数到缓冲区
// 使用示例     load_default();  // 内部函数，用户不直接调用
//-------------------------------------------------------------------------------------------------------------------

static void load_default(void)
{
    for(uint8 i = 0; i < 21; i++)
    {
        param_cache[i] = DEFAULT_PARAMS[i];
    }
}

static void copy_cache_to_flash_buffer(void)
{
    for(uint8 i = 0; i < 21; i++)
    {
        flash_union_buffer[i].float_type = param_cache[i];
    }
}

static void copy_flash_buffer_to_cache(void)
{
    for(uint8 i = 0; i < 21; i++)
    {
        param_cache[i] = flash_union_buffer[i].float_type;
    }
}

static uint8 param_cache_is_valid(void)
{
    uint8 non_zero_kp_count = 0;
    uint8 kp_index[7] = {FLASH_RATE__KP, FLASH_ANGLE_KP, FLASH_SPEED_KP, FLASH_TURN__KP, FLASH_TRACK_KP, FLASH_HEAD__KP, FLASH_POSI__KP};

    for(uint8 i = 0; i < 21; i++)
    {
        if(!isfinite(param_cache[i]))
        {
            return 0;
        }
        if(fabsf(param_cache[i]) > 10000.0f)
        {
            return 0;
        }
    }

    for(uint8 i = 0; i < 7; i++)
    {
        if(fabsf(param_cache[kp_index[i]]) > 0.0001f)
        {
            non_zero_kp_count++;
        }
    }

    // 关键Kp几乎全为0，判定为无效参数页
    if(non_zero_kp_count == 0)
    {
        return 0;
    }

    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     同步pid参数至缓存区（仍然需要调用Param_Save();）
// 使用示例     Param_SyncToPID();
// 备注信息     一般是配合蓝牙调参使用的（针对需要将关键pid参数存入flash的要求）
//-------------------------------------------------------------------------------------------------------------------

void Param_SyncToPID(void)
{
    // 角速度环PID
    Rate__PID.Kp = RATE__KP;
    Rate__PID.Ki = RATE__KI;
    Rate__PID.Kd = RATE__KD;
    
    // 角度环PID
    Angle_PID.Kp = ANGLE_KP;
    Angle_PID.Ki = ANGLE_KI;
    Angle_PID.Kd = ANGLE_KD;
    
    // 速度环PID
    Speed_PID.Kp = SPEED_KP;
    Speed_PID.Ki = SPEED_KI;
    Speed_PID.Kd = SPEED_KD;
    
    // 转向环PID
    Turn__PID.Kp = TURN__KP;
    Turn__PID.Ki = TURN__KI;
    Turn__PID.Kd = TURN__KD;

	// 循迹环PID
    Track_PID.Kp = TRACK_KP;
    Track_PID.Ki = TRACK_KI;
    Track_PID.Kd = TRACK_KD;
	
	// 航向角环PID
	Head__PID.Kp = HEAD__KP;
	Head__PID.Ki = HEAD__KI;
	Head__PID.Kd = HEAD__KD;
	
	// 位置环PID
	Posi__PID.Kp = POSI__KP;
	Posi__PID.Ki = POSI__KI;
	Posi__PID.Kd = POSI__KD;
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
        copy_flash_buffer_to_cache();

        // 读取到异常/全零参数时，自动回退默认值并修复Flash页
        if(!param_cache_is_valid())
        {
            load_default();
            copy_cache_to_flash_buffer();
            flash_write_page_from_buffer(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE);
        }
    }
    else  // 返回0 = 没有数据（首次使用）
    {
        // 加载默认值到缓冲区
        load_default();
        
        // 写入Flash
        copy_cache_to_flash_buffer();
        flash_write_page_from_buffer(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE);		
    }
		// 同步pid参数至缓存区
		Param_SyncToPID();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存参数到Flash
// 使用示例     Param_Save();  // 在退出参数修改时调用
// 备注信息     将当前缓冲区的数据写入Flash
//-------------------------------------------------------------------------------------------------------------------

void Param_Save(void)
{
    copy_cache_to_flash_buffer();
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
