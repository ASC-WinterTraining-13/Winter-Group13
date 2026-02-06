/********************************************************************************************************************
* 江协科技OLED-V2.0/UFT-8/7针脚SPI接口+逐飞科技spi底层 整合项目
* 即基于逐飞科技spi底层迁移江协科技OLED上层功能
* 利用GPT-5进行整合，可以显示中文，使用格式化打印，和大部分线条绘制功能
* 整合涉及文件：
* "OLED.c"			"OLED.h"
* "OLED_Data.c"		"OLED_Data.h"
* 字符集添加操作与江协科技OLED驱动基本相同
********************************************************************************************************************/


#include "zf_driver_spi.h"
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "OLED.h"
#include "OLED_Data.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/**
 * @brief 硬件连接定义 (请根据你的 MM32 核心板接线修改)
 */
#define OLED_SPI             SPI_1
#define OLED_SPI_SPEED       (40 * 1000 * 1000)
#define OLED_D0_PIN          SPI1_SCK_A5        // D0 (CLK)
#define OLED_D1_PIN          SPI1_MOSI_A7       // D1 (MOSI)
#define OLED_RES_PIN         A6                 // RES (Reset)
#define OLED_DC_PIN          D0                 // DC (Data/Command)
#define OLED_CS_PIN          A4                 // CS (Chip Select)

/* 逐飞库引脚操作宏适配 */
#define OLED_W_RES(x)        ((x) ? gpio_high(OLED_RES_PIN) : gpio_low(OLED_RES_PIN))
#define OLED_W_DC(x)         ((x) ? gpio_high(OLED_DC_PIN)  : gpio_low(OLED_DC_PIN))
#define OLED_W_CS(x)         ((x) ? gpio_high(OLED_CS_PIN)  : gpio_low(OLED_CS_PIN))

/* 屏幕参数 */
#define OLED_WIDTH           128
#define OLED_HEIGHT          64

/* 显存数组 */
static uint8_t OLED_DisplayBuf[8][128];

/**************** 底层通信函数 ****************/

static void OLED_SPI_SendByte(uint8_t Byte) {
    spi_write_8bit(OLED_SPI, Byte);
}

void OLED_WriteCommand(uint8_t Command) {
    OLED_W_CS(0);
    OLED_W_DC(0);
    OLED_SPI_SendByte(Command);
    OLED_W_CS(1);
}

void OLED_WriteData(uint8_t *Data, uint16_t Count) {
    OLED_W_CS(0);
    OLED_W_DC(1);
    while (Count--) {
        OLED_SPI_SendByte(*Data++);
    }
    OLED_W_CS(1);
}

static void OLED_SetCursor(uint8_t Page, uint8_t X) {
    OLED_WriteCommand(0xB0 | Page);					//设置页地址
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//设置列地址高4位
    OLED_WriteCommand(0x00 | (X & 0x0F));			//设置列地址低4位
}

/**************** 显存与更新 ****************/

void OLED_Update(void) {
    for (uint8_t i = 0; i < 8; i++) {
        OLED_SetCursor(i, 0);
        OLED_WriteData(OLED_DisplayBuf[i], 128);
    }
}

void OLED_UpdateArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height) {
    if (X < 0) Width += X; X = X < 0 ? 0 : X;
    if (Y < 0) Height += Y; Y = Y < 0 ? 0 : Y;
    if (X + Width > 128) Width = 128 - X;
    if (Y + Height > 64) Height = 64 - Y;
    
    for (uint8_t i = Y / 8; i < (Y + Height - 1) / 8 + 1; i++) {
        OLED_SetCursor(i, X);
        OLED_WriteData(&OLED_DisplayBuf[i][X], Width);
    }
}

void OLED_Clear(void) {
    memset(OLED_DisplayBuf, 0x00, sizeof(OLED_DisplayBuf));
    OLED_Update();
}

void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height) {
    for (int16_t j = Y; j < Y + Height; j++) {
        for (int16_t i = X; i < X + Width; i++) {
            if (i >= 0 && i < 128 && j >= 0 && j < 64) {
                OLED_DisplayBuf[j / 8][i] &= ~(0x01 << (j % 8));
            }
        }
    }
    OLED_UpdateArea(X, Y, Width, Height);
}

void OLED_Reverse(void) {
    for (uint8_t i = 0; i < 8; i++) {
        for (uint8_t j = 0; j < 128; j++) {
            OLED_DisplayBuf[i][j] ^= 0xFF;
        }
    }
    OLED_Update();
}

void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height) {
    for (int16_t j = Y; j < Y + Height; j++) {
        for (int16_t i = X; i < X + Width; i++) {
            if (i >= 0 && i < 128 && j >= 0 && j < 64) {
                OLED_DisplayBuf[j / 8][i] ^= (0x01 << (j % 8));
            }
        }
    }
    OLED_UpdateArea(X, Y, Width, Height);
}

/**************** 基础绘图 ****************/

void OLED_DrawPoint(int16_t X, int16_t Y) {
    if (X >= 0 && X < 128 && Y >= 0 && Y < 64) {
        OLED_DisplayBuf[Y / 8][X] |= (0x01 << (Y % 8));
    }
}

uint8_t OLED_GetPoint(int16_t X, int16_t Y) {
    if (X >= 0 && X < 128 && Y >= 0 && Y < 64) {
        return (OLED_DisplayBuf[Y / 8][X] >> (Y % 8)) & 0x01;
    }
    return 0;
}

void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1) {
    int16_t dx = abs(X1 - X0), dy = abs(Y1 - Y0);
    int16_t sx = (X0 < X1) ? 1 : -1, sy = (Y0 < Y1) ? 1 : -1;
    int16_t err = dx - dy, e2;
    while (1) {
        OLED_DrawPoint(X0, Y0);
        if (X0 == X1 && Y0 == Y1) break;
        e2 = 2 * err;
        if (e2 > -dy) { err -= dy; X0 += sx; }
        if (e2 < dx) { err += dx; Y0 += sy; }
    }
}

/**************** 几何形状 ****************/

void OLED_DrawRectangle(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled) {
    if (!IsFilled) {
        OLED_DrawLine(X, Y, X + Width - 1, Y);
        OLED_DrawLine(X, Y, X, Y + Height - 1);
        OLED_DrawLine(X + Width - 1, Y, X + Width - 1, Y + Height - 1);
        OLED_DrawLine(X, Y + Height - 1, X + Width - 1, Y + Height - 1);
    } else {
        for (int16_t i = X; i < X + Width; i++) {
            for (int16_t j = Y; j < Y + Height; j++) {
                OLED_DrawPoint(i, j);
            }
        }
    }
}

/* 内部辅助：多边形填充判定 */
static uint8_t OLED_pnpoly(uint8_t nvert, int16_t *vertx, int16_t *verty, int16_t testx, int16_t testy) {
    uint8_t i, j, c = 0;
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((verty[i] > testy) != (verty[j] > testy)) &&
            (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
            c = !c;
    }
    return c;
}

void OLED_DrawTriangle(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint8_t IsFilled) {
    if (!IsFilled) {
        OLED_DrawLine(X0, Y0, X1, Y1);
        OLED_DrawLine(X1, Y1, X2, Y2);
        OLED_DrawLine(X2, Y2, X0, Y0);
    } else {
        int16_t minx = X0, maxx = X0, miny = Y0, maxy = Y0;
        if (X1 < minx) minx = X1; if (X1 > maxx) maxx = X1;
        if (X2 < minx) minx = X2; if (X2 > maxx) maxx = X2;
        if (Y1 < miny) miny = Y1; if (Y1 > maxy) maxy = Y1;
        if (Y2 < miny) miny = Y2; if (Y2 > maxy) maxy = Y2;
        int16_t vx[] = {X0, X1, X2}, vy[] = {Y0, Y1, Y2};
        for (int16_t i = minx; i <= maxx; i++) {
            for (int16_t j = miny; j <= maxy; j++) {
                if (OLED_pnpoly(3, vx, vy, i, j)) OLED_DrawPoint(i, j);
            }
        }
    }
}

void OLED_DrawCircle(int16_t X, int16_t Y, uint8_t Radius, uint8_t IsFilled) {
    int16_t x = 0, y = Radius, d = 3 - 2 * Radius;
    while (x <= y) {
        if (!IsFilled) {
            OLED_DrawPoint(X + x, Y + y); OLED_DrawPoint(X - x, Y + y);
            OLED_DrawPoint(X + x, Y - y); OLED_DrawPoint(X - x, Y - y);
            OLED_DrawPoint(X + y, Y + x); OLED_DrawPoint(X - y, Y + x);
            OLED_DrawPoint(X + y, Y - x); OLED_DrawPoint(X - y, Y - x);
        } else {
            for (int16_t j = Y - y; j <= Y + y; j++) { OLED_DrawPoint(X + x, j); OLED_DrawPoint(X - x, j); }
            for (int16_t j = Y - x; j <= Y + x; j++) { OLED_DrawPoint(X + y, j); OLED_DrawPoint(X - y, j); }
        }
        if (d < 0) d += 4 * x + 6;
        else { d += 4 * (x - y) + 10; y--; }
        x++;
    }
}

void OLED_DrawEllipse(int16_t X, int16_t Y, uint8_t A, uint8_t B, uint8_t IsFilled) {
    int16_t a = A, b = B;
    int16_t x = 0, y = b;
    long a2 = a * a, b2 = b * b;
    long d = b2 + a2 * (0.25 - b);
    while (b2 * x <= a2 * y) {
        if (!IsFilled) {
            OLED_DrawPoint(X + x, Y + y); OLED_DrawPoint(X - x, Y + y);
            OLED_DrawPoint(X + x, Y - y); OLED_DrawPoint(X - x, Y - y);
        } else {
            for (int16_t j = Y - y; j <= Y + y; j++) { OLED_DrawPoint(X + x, j); OLED_DrawPoint(X - x, j); }
        }
        if (d < 0) d += b2 * (2 * x + 3);
        else { d += b2 * (2 * x + 3) + a2 * (2 - 2 * y); y--; }
        x++;
    }
    d = b2 * (x + 0.5) * (x + 0.5) + a2 * (y - 1) * (y - 1) - a2 * b2;
    while (y >= 0) {
        if (!IsFilled) {
            OLED_DrawPoint(X + x, Y + y); OLED_DrawPoint(X - x, Y + y);
            OLED_DrawPoint(X + x, Y - y); OLED_DrawPoint(X - x, Y - y);
        } else {
            for (int16_t j = Y - y; j <= Y + y; j++) { OLED_DrawPoint(X + x, j); OLED_DrawPoint(X - x, j); }
        }
        if (d > 0) d += a2 * (3 - 2 * y);
        else { d += a2 * (3 - 2 * y) + b2 * (2 * x + 2); x++; }
        y--;
    }
}

static uint8_t OLED_IsInAngle(int16_t X, int16_t Y, int16_t StartAngle, int16_t EndAngle) {
    double angle = atan2(-Y, X) * 180 / 3.14159265;
    if (angle < 0) angle += 360;
    if (StartAngle <= EndAngle) return (angle >= StartAngle && angle <= EndAngle);
    else return (angle >= StartAngle || angle <= EndAngle);
}

void OLED_DrawArc(int16_t X, int16_t Y, uint8_t Radius, int16_t StartAngle, int16_t EndAngle, uint8_t IsFilled) {
    for (int16_t j = Y - Radius; j <= Y + Radius; j++) {
        for (int16_t i = X - Radius; i <= X + Radius; i++) {
            if (pow(i - X, 2) + pow(j - Y, 2) <= pow(Radius, 2)) {
                if (OLED_IsInAngle(i - X, j - Y, StartAngle, EndAngle)) {
                    if (IsFilled || pow(i - X, 2) + pow(j - Y, 2) > pow(Radius - 1, 2)) OLED_DrawPoint(i, j);
                }
            }
        }
    }
}

/**************** 字符与文本 ****************/

void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize) {
    uint8_t c = Char - ' ';
    if (FontSize == OLED_8X16) {
        for (uint8_t i = 0; i < 8; i++) OLED_DisplayBuf[Y / 8][X + i] = OLED_F8x16[c][i];
        for (uint8_t i = 0; i < 8; i++) OLED_DisplayBuf[Y / 8 + 1][X + i] = OLED_F8x16[c][i + 8];
    } else {
        for (uint8_t i = 0; i < 6; i++) OLED_DisplayBuf[Y / 8][X + i] = OLED_F6x8[c][i];
    }
}

void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)		//遍历字符串
	{
		/* 判断当前字符是否为 ASCII 字符 */
		if ((uint8_t)String[i] < 128)
		{
			/* 是 ASCII 字符，正常显示 */
			OLED_ShowChar(X, Y, String[i], FontSize);
			X += FontSize;					//根据字体大小移动 X 坐标
		}
		else
		{
			/* 不是 ASCII 字符，视为中文字符 */
			uint8_t pIndex;
			uint8_t ChineseCount = 0;
			char SingleChinese[5] = {0};	// 临时存放一个汉字的编码
			
#ifdef OLED_CHARSET_UTF8
			/* UTF-8 编码下，汉字通常占 3 个字节 */
			SingleChinese[0] = String[i];
			SingleChinese[1] = String[i + 1];
			SingleChinese[2] = String[i + 2];
			ChineseCount = 3;
#endif

#ifdef OLED_CHARSET_GB2312
			/* GB2312 编码下，汉字通常占 2 个字节 */
			SingleChinese[0] = String[i];
			SingleChinese[1] = String[i + 1];
			ChineseCount = 2;
#endif

			/* 在汉字库中检索该编码 */
			for (pIndex = 0; strcmp(OLED_CF16x16[pIndex].Index, "") != 0; pIndex++)
			{
				if (strcmp(OLED_CF16x16[pIndex].Index, SingleChinese) == 0)
				{
					break;		// 找到匹配的汉字
				}
			}

			/* 显示查找到的汉字字模 */
			/* 注意：江协汉字默认是 16x16 大小 */
			OLED_ShowImage(X, Y, 16, 16, OLED_CF16x16[pIndex].Data);
			
			X += 16;			// 汉字占用 16 像素宽度
			i += (ChineseCount - 1); // 字符串指针跳过剩余的汉字字节
		}
	}
}

void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...) {
    char str[64];
    va_list arg;
    va_start(arg, format);
    vsnprintf(str, 64, format, arg);
    va_end(arg);
    OLED_ShowString(X, Y, str, FontSize);
}

// 其余 ShowNum, ShowFloatNum 等可使用 snprintf 在 OLED_Printf 基础上快速实现
void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize) {
    OLED_Printf(X, Y, FontSize, "%0*lu", Length, Number);
}

void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize) {
    OLED_Printf(X, Y, FontSize, "%+0*ld", Length, Number);
}

void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize) {
    OLED_Printf(X, Y, FontSize, "%0*lX", Length, Number);
}

void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize) {
    char str[33];
    for (uint8_t i = 0; i < Length; i++) {
        str[i] = (Number >> (Length - i - 1)) & 0x01 ? '1' : '0';
    }
    str[Length] = '\0';
    OLED_ShowString(X, Y, str, FontSize);
}

void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize) {
    OLED_Printf(X, Y, FontSize, "%0*.*f", IntLength + FraLength + 1, FraLength, Number);
}

void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image) {
    OLED_ClearArea(X, Y, Width, Height);
    for (uint8_t j = 0; j < (Height - 1) / 8 + 1; j++) {
        for (uint8_t i = 0; i < Width; i++) {
            if (X + i >= 0 && X + i < 128 && Y / 8 + j >= 0 && Y / 8 + j < 8) {
                OLED_DisplayBuf[Y / 8 + j][X + i] |= (Image[j * Width + i] << (Y % 8));
                if (Y % 8 != 0 && Y / 8 + j + 1 < 8) {
                    OLED_DisplayBuf[Y / 8 + j + 1][X + i] |= (Image[j * Width + i] >> (8 - Y % 8));
                }
            }
        }
    }
}

/**************** 初始化 ****************/

void OLED_Init(void) {
    /* 逐飞库初始化 */
    spi_init(OLED_SPI, SPI_MODE0, OLED_SPI_SPEED, OLED_D0_PIN, OLED_D1_PIN, SPI_MISO_NULL, SPI_CS_NULL);
    gpio_init(OLED_RES_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(OLED_DC_PIN,  GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(OLED_CS_PIN,  GPO, GPIO_HIGH, GPO_PUSH_PULL);
    
    OLED_W_RES(0);
    system_delay_ms(100);
    OLED_W_RES(1);
    
    /* 厂商初始化命令序列 (SSD1306) */
    OLED_WriteCommand(0xAE); OLED_WriteCommand(0x00); OLED_WriteCommand(0x10);
    OLED_WriteCommand(0x40); OLED_WriteCommand(0x81); OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xA1); OLED_WriteCommand(0xC8); OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0xA8); OLED_WriteCommand(0x3f); OLED_WriteCommand(0xD3);
    OLED_WriteCommand(0x00); OLED_WriteCommand(0xD5); OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xD9); OLED_WriteCommand(0xF1); OLED_WriteCommand(0xDA);
    OLED_WriteCommand(0x12); OLED_WriteCommand(0xDB); OLED_WriteCommand(0x40);
    OLED_WriteCommand(0x20); OLED_WriteCommand(0x02); OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x14); OLED_WriteCommand(0xA4); OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0xAF);
    
    OLED_Clear();
}
