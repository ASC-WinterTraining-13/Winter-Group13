#include "zf_driver_spi.h"
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "OLED.h"
#include "OLED_Data.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// ================ 逐飞库引脚和SPI配置（请按实际情况修改） ================
#define OLED_SPI             SPI_1
#define OLED_SPI_SPEED       (40 * 1000 * 1000)
#define OLED_D0_PIN          SPI1_SCK_A5        // SPI SCK (D0)
#define OLED_D1_PIN          SPI1_MOSI_A7       // SPI MOSI (D1)
#define OLED_RES_PIN         A6                 // OLED RES
#define OLED_DC_PIN          D0                 // OLED DC
#define OLED_CS_PIN          A4                 // OLED CS

#define OLED_RES(x)  ((x) ? gpio_high(OLED_RES_PIN) : gpio_low(OLED_RES_PIN))
#define OLED_DC(x)   ((x) ? gpio_high(OLED_DC_PIN)  : gpio_low(OLED_DC_PIN))
#define OLED_CS(x)   ((x) ? gpio_high(OLED_CS_PIN)  : gpio_low(OLED_CS_PIN))

// ------------------- 显存与常量 -------------------
#define OLED_X_MAX           128
#define OLED_Y_MAX           64
static uint8_t OLED_DisplayBuf[8][128];

// ================ 底层硬件适配接口 ================
static void OLED_SPI_SendByte(uint8_t Byte)
{
    spi_write_8bit(OLED_SPI, Byte);
}
static void OLED_WriteCommand(uint8_t Command)
{
    OLED_CS(0); OLED_DC(0);
    OLED_SPI_SendByte(Command);
    OLED_CS(1);
}
static void OLED_WriteDataStream(const uint8_t *Data, uint16_t Count)
{
    OLED_CS(0); OLED_DC(1);
    while (Count--)
        OLED_SPI_SendByte(*Data++);
    OLED_CS(1);
}

static void OLED_GPIO_Init(void)
{
    spi_init(OLED_SPI, SPI_MODE0, OLED_SPI_SPEED, OLED_D0_PIN, OLED_D1_PIN, SPI_MISO_NULL, SPI_CS_NULL);
    gpio_init(OLED_RES_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(OLED_DC_PIN,  GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(OLED_CS_PIN,  GPO, GPIO_HIGH, GPO_PUSH_PULL);
    OLED_RES(1); OLED_DC(1); OLED_CS(1);
}

// ================ 核心时序 ================
static void OLED_SetCursor(uint8_t Page, uint8_t X)
{
    OLED_WriteCommand(0xB0 | (Page & 0x07));
    OLED_WriteCommand(0x10 | ((X >> 4) & 0x0F));
    OLED_WriteCommand(0x00 | (X & 0x0F));
}

// ================ 初始化 ===================
void OLED_Init(void)
{
    OLED_GPIO_Init();
    OLED_RES(0);
    system_delay_ms(50);
    OLED_RES(1);

    OLED_WriteCommand(0xAE);      // Display OFF
    OLED_WriteCommand(0x00);      // Set lower column address
    OLED_WriteCommand(0x10);      // Set higher column address
    OLED_WriteCommand(0x40);      // Set display start line
    OLED_WriteCommand(0xB0);      // Set page address
    OLED_WriteCommand(0x81);      // Contrast
    OLED_WriteCommand(0x7F);      // Contrast value
    OLED_WriteCommand(0xA1);      // Segment remap
    OLED_WriteCommand(0xA6);      // Normal display
    OLED_WriteCommand(0xA8);      // Multiplex ratio
    OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xC8);      // COM scan direction
    OLED_WriteCommand(0xD3);      // Display offset
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0xD5);      // Oscillator division
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xD9);      // Pre-charge period
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDA);      // COM pins
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0xDB);      // VCOMH
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0x20);      // Page mode
    OLED_WriteCommand(0x02);
    OLED_WriteCommand(0x8D);      // Charge pump
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xA4);      // Resume RAM display
    OLED_WriteCommand(0xA6);      // Normal display
    OLED_WriteCommand(0xAF);      // Display ON

    OLED_Clear();
}

// ================ 显存同步更新 ================
void OLED_Update(void)
{
    for(uint8_t page=0; page<8; page++)
    {
        OLED_SetCursor(page, 0);
        OLED_WriteDataStream(OLED_DisplayBuf[page], OLED_X_MAX);
    }
}
void OLED_UpdateArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    uint8_t page_start = Y / 8, page_end = (Y + Height - 1) / 8;
    for(uint8_t p = page_start; p <= page_end; ++p) {
        OLED_SetCursor(p, X);
        OLED_WriteDataStream(&OLED_DisplayBuf[p][X], Width);
    }
}

// ================ 清除、反色、区域操作 ================
void OLED_Clear(void)
{
    memset(OLED_DisplayBuf, 0x00, sizeof(OLED_DisplayBuf));
    OLED_Update();
}
void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    uint8_t page_start = Y / 8, page_end = (Y + Height - 1) / 8;
    for(uint8_t p = page_start; p <= page_end; ++p)
        memset(&OLED_DisplayBuf[p][X], 0x00, Width);
    OLED_UpdateArea(X, Y, Width, Height);
}
void OLED_Reverse(void)
{
    for(uint8_t p=0; p<8; ++p)
        for(uint8_t x=0; x<OLED_X_MAX; ++x)
            OLED_DisplayBuf[p][x] = ~(OLED_DisplayBuf[p][x]);
    OLED_Update();
}
void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    uint8_t page_start = Y / 8, page_end = (Y + Height - 1) / 8;
    for(uint8_t p = page_start; p <= page_end; ++p)
        for(uint8_t x = X; x < X + Width; ++x)
            OLED_DisplayBuf[p][x] = ~(OLED_DisplayBuf[p][x]);
    OLED_UpdateArea(X, Y, Width, Height);
}

// ================ 像素点操作 ================
void OLED_DrawPoint(int16_t X, int16_t Y)
{
    if(X<0 || X>=OLED_X_MAX || Y<0 || Y>=OLED_Y_MAX) return;
    OLED_DisplayBuf[Y/8][X] |= (1<<(Y%8));
}
uint8_t OLED_GetPoint(int16_t X, int16_t Y)
{
    if(X<0 || X>=OLED_X_MAX || Y<0 || Y>=OLED_Y_MAX) return 0;
    return ((OLED_DisplayBuf[Y/8][X]>>(Y%8)) & 0x01);
}

// ================ 字符/字符串/数字显示 ================
void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize)
{
    uint8_t c = Char - 32;
    if(FontSize == OLED_8X16)
    {
        for(uint8_t i=0;i<8;++i)
            OLED_DisplayBuf[Y/8][X+i] = OLED_F8x16[c][i];
        for(uint8_t i=0;i<8;++i)
            OLED_DisplayBuf[Y/8+1][X+i] = OLED_F8x16[c][i+8];
    }
    else if(FontSize == OLED_6X8)
    {
        for(uint8_t i=0;i<6;++i)
            OLED_DisplayBuf[Y/8][X+i] = OLED_F6x8[c][i];
    }
    OLED_UpdateArea(X,Y,FontSize,FontSize==OLED_8X16?16:8);
}
void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize)
{
    while(*String)
    {
        OLED_ShowChar(X, Y, *String, FontSize);
        X += FontSize;
        ++String;
    }
}
void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%0*lu", Length, (unsigned long)Number);
    OLED_ShowString(X, Y, buf, FontSize);
}
void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize)
{
    char buf[14];
    snprintf(buf, sizeof(buf), "%+0*ld", Length, (long)Number);
    OLED_ShowString(X, Y, buf, FontSize);
}
void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%0*lX", Length, (unsigned long)Number);
    OLED_ShowString(X, Y, buf, FontSize);
}
void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    char buf[48] = {0};
    for(int i=Length-1;i>=0;--i)
        buf[Length-1-i] = (Number&(1<<i)) ? '1':'0';
    OLED_ShowString(X, Y, buf, FontSize);
}
void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize)
{
    char fmt[10], buf[32];
    snprintf(fmt, sizeof(fmt), "%%%d.%df", IntLength+FraLength+1, FraLength);
    snprintf(buf, sizeof(buf), fmt, Number);
    OLED_ShowString(X, Y, buf, FontSize);
}
void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...)
{
    char buf[64];
    va_list args; va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    OLED_ShowString(X, Y, buf, FontSize);
}
void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image)
{
    for(uint8_t page=0;page<Height/8;page++)
        for(uint8_t i=0;i<Width;i++)
            OLED_DisplayBuf[(Y/8)+page][X+i] = *(Image + i + page*Width);
    OLED_UpdateArea(X, Y, Width, Height);
}

// ================ 基础图形 =======================
void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1)
{
    int dx=abs(X1-X0), dy=abs(Y1-Y0), sx=X0<X1?1:-1, sy=Y0<Y1?1:-1, err=dx-dy, e2;
    while(1)
    {
        OLED_DrawPoint(X0, Y0);
        if(X0==X1 && Y0==Y1) break;
        e2=2*err;
        if(e2>-dy){err-=dy;X0+=sx;}
        if(e2<dx){err+=dx;Y0+=sy;}
    }
    OLED_Update();
}
void OLED_DrawRectangle(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled)
{
    if(IsFilled)
        for(int16_t i=X;i<X+Width;i++)
            for(int16_t j=Y;j<Y+Height;j++)
                OLED_DrawPoint(i,j);
    else
    {
        for(int16_t i=X;i<X+Width;i++)
            OLED_DrawPoint(i,Y), OLED_DrawPoint(i,Y+Height-1);
        for(int16_t j=Y;j<Y+Height;j++)
            OLED_DrawPoint(X,j), OLED_DrawPoint(X+Width-1,j);
    }
    OLED_Update();
}

void OLED_DrawCircle(int16_t X,int16_t Y,uint8_t R,uint8_t IsFilled)
{
    int x = 0, y = R; int d = 3 - 2*R;
    while(x<=y)
    {
        if(IsFilled)
        {
            for(int j=Y-y;j<=Y+y;j++)
                OLED_DrawPoint(X+x,j),OLED_DrawPoint(X-x,j);
            for(int j=Y-x;j<=Y+x;j++)
                OLED_DrawPoint(X+y,j),OLED_DrawPoint(X-y,j);
        }
        else
        {
            OLED_DrawPoint(X+x,Y+y);OLED_DrawPoint(X-x,Y+y);
            OLED_DrawPoint(X+x,Y-y);OLED_DrawPoint(X-x,Y-y);
            OLED_DrawPoint(X+y,Y+x);OLED_DrawPoint(X-y,Y+x);
            OLED_DrawPoint(X+y,Y-x);OLED_DrawPoint(X-y,Y-x);
        }
        if(d<0)d+=4*x+6;
        else{d+=4*(x-y)+10; y--;}
        x++;
    }
    OLED_Update();
}
