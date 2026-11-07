#include "USART.h"
#include "OLED.h"
#include "SI2C.h"

/* SSD1306 常见写地址：7 位地址 0x3C 左移一位 */
#define OLED_ADDR  0x78

/* 置位后，普通显示写入会停止，避免持续访问故障总线 */
static volatile uint8_t oled_io_error = 0;

/* 0=ACK，1=NACK。每发送一个字节都检查应答。 */
static uint8_t OLED_SendCheckedByte(uint8_t data)
{
    if (sendbyte(data) != 0U)
    {
        oled_io_error = 1U;
        return 0U;
    }
    return 1U;
}

/* 向 OLED 写一条命令；失败返回 0 */
static uint8_t OLED_WriteCommand(uint8_t cmd)
{
    uint8_t ok = 1U;

    if (oled_io_error)
        return 0U;

    sendstart();
    if (!OLED_SendCheckedByte(OLED_ADDR)) ok = 0U;
    if (ok && !OLED_SendCheckedByte(0x00)) ok = 0U;
    if (ok && !OLED_SendCheckedByte(cmd))  ok = 0U;
    sendstop();

    return ok;
}

/* 向 OLED 写一个显示数据字节；失败返回 0 */
static uint8_t OLED_WriteData(uint8_t data)
{
    uint8_t ok = 1U;

    if (oled_io_error)
        return 0U;

    sendstart();
    if (!OLED_SendCheckedByte(OLED_ADDR)) ok = 0U;
    if (ok && !OLED_SendCheckedByte(0x40)) ok = 0U;
    if (ok && !OLED_SendCheckedByte(data)) ok = 0U;
    sendstop();

    return ok;
}

uint8_t OLED_HasError(void)
{
    return oled_io_error;
}

/* 检查总线空闲电平及 OLED 地址 ACK。不会清除已记录的写错误。 */
uint8_t OLED_CheckConnection(void)
{
    uint8_t ack;

    /* I2C 空闲时两根线都应被释放为高电平 */
    scl_write(1);
    sda_write(1);
    delay_us(10);

    if (sda_read() == 0U)
        return 0U;

    sendstart();
    ack = sendbyte(OLED_ADDR);
    sendstop();

    /* 释放总线后 SDA 应回到高电平 */
    sda_write(1);
    scl_write(1);
    delay_us(2);

    if (ack != 0U || sda_read() == 0U)
        return 0U;

    return 1U;
}

/* ==================== OLED 地址设置 ==================== */

void OLED_SetPos(uint8_t page, uint8_t column)
{
    if (page > 7U || column > 127U || oled_io_error)
        return;

    if (!OLED_WriteCommand((uint8_t)(0xB0U | page))) return;
    if (!OLED_WriteCommand((uint8_t)(0x10U | ((column >> 4) & 0x0FU)))) return;
    OLED_WriteCommand((uint8_t)(column & 0x0FU));
}

/* ==================== 清屏函数 ==================== */

void OLED_Clear(void)
{
    uint8_t page;
    uint16_t column;

    if (oled_io_error)
        return;

    for (page = 0; page < 8U; page++)
    {
        OLED_SetPos(page, 0);
        if (oled_io_error) return;

        for (column = 0; column < 128U; column++)
        {
            if (!OLED_WriteData(0x00))
                return;
        }
    }
}

void OLED_ClearLine(uint8_t page)
{
    uint16_t column;

    if (page > 7U || oled_io_error)
        return;

    OLED_SetPos(page, 0);
    if (oled_io_error) return;

    for (column = 0; column < 128U; column++)
    {
        if (!OLED_WriteData(0x00))
            return;
    }
}

/* ==================== OLED 初始化 ==================== */

uint8_t OLED_Init(void)
{
    MY_SI2C_INIT();
    delay_us(1000);

    /* 每次初始化尝试前清除上一次的错误标志 */
    oled_io_error = 0U;

    if (!OLED_CheckConnection())
    {
        oled_io_error = 1U;
        return 0U;
    }

    if (!OLED_WriteCommand(0xAE)) return 0U;
    if (!OLED_WriteCommand(0xD5)) return 0U;
    if (!OLED_WriteCommand(0x80)) return 0U;
    if (!OLED_WriteCommand(0xA8)) return 0U;
    if (!OLED_WriteCommand(0x3F)) return 0U;
    if (!OLED_WriteCommand(0xD3)) return 0U;
    if (!OLED_WriteCommand(0x00)) return 0U;
    if (!OLED_WriteCommand(0x40)) return 0U;
    if (!OLED_WriteCommand(0x8D)) return 0U;
    if (!OLED_WriteCommand(0x14)) return 0U;
    if (!OLED_WriteCommand(0x20)) return 0U;
    if (!OLED_WriteCommand(0x02)) return 0U;
    if (!OLED_WriteCommand(0xA1)) return 0U;
    if (!OLED_WriteCommand(0xC8)) return 0U;
    if (!OLED_WriteCommand(0xDA)) return 0U;
    if (!OLED_WriteCommand(0x12)) return 0U;
    if (!OLED_WriteCommand(0x81)) return 0U;
    if (!OLED_WriteCommand(0x7F)) return 0U;
    if (!OLED_WriteCommand(0xD9)) return 0U;
    if (!OLED_WriteCommand(0xF1)) return 0U;
    if (!OLED_WriteCommand(0xDB)) return 0U;
    if (!OLED_WriteCommand(0x30)) return 0U;
    if (!OLED_WriteCommand(0xA4)) return 0U;
    if (!OLED_WriteCommand(0xA6)) return 0U;
    if (!OLED_WriteCommand(0xAF)) return 0U;

    OLED_Clear();

    if (oled_io_error)
        return 0U;

    return 1U;
}

/* ==================== 6x8 字体 ==================== */

/*
 * 每个字符占 6 列：
 * 前 5 列为字模，第 6 列为空白间隔。
 * 支持数字 0~9 和大写字母 A~Z。
 */
static const uint8_t OLED_F6x8[][5] =
{
    /* 数字 0~9 */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9 */

    /* 字母 A~Z */
    {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x09,0x01}, /* F */
    {0x3E,0x41,0x49,0x49,0x7A}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */
    {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x3F,0x40,0x38,0x40,0x3F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x07,0x08,0x70,0x08,0x07}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}  /* Z */
};

/* 根据字符获取字模 */
static const uint8_t *OLED_GetFont(char ch)
{
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};

    if (ch >= '0' && ch <= '9')
    {
        return OLED_F6x8[ch - '0'];
    }

    if (ch >= 'A' && ch <= 'Z')
    {
        return OLED_F6x8[10 + ch - 'A'];
    }

    if (ch >= 'a' && ch <= 'z')
    {
        return OLED_F6x8[10 + ch - 'a'];
    }

    /* 空格和暂不支持的字符显示为空白 */
    return blank;
}

/* ==================== 字符显示 ==================== */

void OLED_ShowChar(uint8_t page, uint8_t column, char ch)
{
    uint8_t i;
    const uint8_t *font;

    if (page > 7 || column > 122)
    {
        return;
    }

    font = OLED_GetFont(ch);

    OLED_SetPos(page, column);

    for (i = 0; i < 5; i++)
    {
        OLED_WriteData(font[i]);
    }

    /* 字符间隔 */
    OLED_WriteData(0x00);
}

/* ==================== 字符串显示 ==================== */

void OLED_ShowString(uint8_t page, uint8_t column, const char *str)
{
    if (str == 0 || page > 7 || column > 127)
    {
        return;
    }

    while (*str != '\0' && column <= 122)
    {
        OLED_ShowChar(page, column, *str);

        column += 6;
        str++;
    }
}

/* ==================== 数字显示 ==================== */

/* 显示一位数字，输入范围为 0~9 */
void OLED_ShowNum(uint8_t page, uint8_t column, uint8_t num)
{
    if (num > 9)
    {
        return;
    }

    OLED_ShowChar(page, column, (char)('0' + num));
}
