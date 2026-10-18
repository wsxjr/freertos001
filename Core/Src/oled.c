/**
 * @file oled.c
 * @brief SSD1306 OLED 驱动
 */
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "oled.h"
#include "oled_font.h"

#include <string.h>

#define OLED_ADDR_LOW           0x78U
#define OLED_ADDR_HIGH          0x7AU
#define OLED_WIDTH              128U
#define OLED_PAGES              8U
#define OLED_DEGREE_CHAR        0x7FU

static uint8_t s_oled_addr = OLED_ADDR_LOW;

static uint8_t s_oled_buf[OLED_PAGES][OLED_WIDTH];

static uint8_t s_oled_tx[1U + OLED_WIDTH];

static const uint8_t s_degree_glyph[16] =
{
    0x00, 0x00, 0x00, 0x0C, 0x12, 0x12, 0x0C, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* 写命令 */
static void OLED_WriteCmd(uint8_t cmd)
{
    uint8_t buf[2];

    buf[0] = 0x00U;
    buf[1] = cmd;
    (void)HAL_I2C_Master_Transmit(&hi2c2, s_oled_addr, buf, 2U, 100U);
}

/* 整页刷新 */
static void OLED_Flush(void)
{
    uint8_t page;
    uint8_t i;

    for (page = 0U; page < OLED_PAGES; page++)
    {
        OLED_WriteCmd((uint8_t)(0xB0U | page));
        OLED_WriteCmd(0x00U);
        OLED_WriteCmd(0x10U);

        s_oled_tx[0] = 0x40U;
        for (i = 0U; i < OLED_WIDTH; i++)
        {
            s_oled_tx[1U + i] = s_oled_buf[page][i];
        }

        (void)HAL_I2C_Master_Transmit(&hi2c2, s_oled_addr, s_oled_tx,
                                      (uint16_t)(1U + OLED_WIDTH), 200U);
    }
}

static /* 清屏 */
void OLED_ClearBuffer(void)
{
    memset(s_oled_buf, 0, sizeof(s_oled_buf));
}

/* 清屏 */
void OLED_Clear(void)
{
    OLED_ClearBuffer();
    OLED_Flush();
}

/* 画一个 8x16 字符 */
static void OLED_DrawChar(uint8_t line, uint8_t col, char ch)
{
    uint8_t idx;
    uint8_t i;
    uint8_t page;
    uint8_t x;
    const uint8_t *glyph;

    if ((line < 1U) || (line > 4U) || (col < 1U) || (col > 16U))
    {
        return;
    }

    page = (uint8_t)((line - 1U) * 2U);
    x = (uint8_t)((col - 1U) * 8U);

    if (ch == (char)OLED_DEGREE_CHAR)
    {
        glyph = s_degree_glyph;
    }
    else
    {
        if ((ch < ' ') || (ch > '~'))
        {
            ch = '?';
        }
        idx = (uint8_t)(ch - ' ');
        glyph = OLED_F8x16[idx];
    }

    for (i = 0U; i < 8U; i++)
    {
        s_oled_buf[page][x + i]      = glyph[i];
        s_oled_buf[page + 1U][x + i] = glyph[8U + i];
    }
}

/* 画字符串 */
static void OLED_DrawString(uint8_t line, uint8_t col, const char *str)
{
    while ((*str != '\0') && (col <= 16U))
    {
        OLED_DrawChar(line, col, *str);
        str++;
        col++;
    }
}

/* 角度保留两位小数 */
static void format_angle(char *out, float value)
{
    int32_t scaled;
    uint32_t abs_scaled;
    uint32_t integer_part;
    uint32_t fraction;
    uint8_t n = 0U;
    uint8_t i = 0U;
    char tmp[4];

    if (value > 999.99f)
    {
        value = 999.99f;
    }
    if (value < -999.99f)
    {
        value = -999.99f;
    }

    scaled = (int32_t)((value * 100.0f)
                       + ((value >= 0.0f) ? 0.5f : -0.5f));

    if (scaled < 0)
    {
        out[n++] = '-';
        abs_scaled = (uint32_t)(-scaled);
    }
    else
    {
        abs_scaled = (uint32_t)scaled;
    }

    integer_part = abs_scaled / 100U;
    fraction = abs_scaled % 100U;

    if (integer_part == 0U)
    {
        tmp[i++] = '0';
    }
    else
    {
        while (integer_part > 0U)
        {
            tmp[i++] = (char)('0' + (integer_part % 10U));
            integer_part /= 10U;
        }
    }

    while (i > 0U)
    {
        out[n++] = tmp[--i];
    }

    out[n++] = '.';
    out[n++] = (char)('0' + (fraction / 10U));
    out[n++] = (char)('0' + (fraction % 10U));
    out[n] = '\0';
}

/* OLED 初始化 */
void OLED_Init(void)
{
    static const uint8_t init_cmds[] =
    {
        0xAEU,             /* display off */
        0xD5U, 0x80U,      /* clock divide */
        0xA8U, 0x3FU,      /* multiplex 64 */
        0xD3U, 0x00U,      /* display offset */
        0x40U,             /* start line 0 */
        0xA1U,             /* segment remap */
        0xC8U,             /* COM scan direction */
        0xDAU, 0x12U,      /* COM pins */
        0x81U, 0xCFU,      /* contrast */
        0xD9U, 0xF1U,      /* pre-charge */
        0xDBU, 0x30U,      /* VCOMH */
        0xA4U,             /* RAM content */
        0xA6U,             /* normal display */
        0x8DU, 0x14U,      /* charge pump */
        0xAFU              /* display on */
    };
    uint8_t i;

    osDelay(100U);

    if (HAL_I2C_IsDeviceReady(&hi2c2, OLED_ADDR_LOW, 3U, 100U) == HAL_OK)
    {
        s_oled_addr = OLED_ADDR_LOW;
    }
    else if (HAL_I2C_IsDeviceReady(&hi2c2, OLED_ADDR_HIGH, 3U, 100U) == HAL_OK)
    {
        s_oled_addr = OLED_ADDR_HIGH;
    }
    else
    {
        s_oled_addr = OLED_ADDR_LOW;
    }

    for (i = 0U; i < (uint8_t)sizeof(init_cmds); i++)
    {
        OLED_WriteCmd(init_cmds[i]);
    }

    OLED_Clear();
}

/* 显示 Roll/Pitch/Yaw */
void OLED_ShowAngles(const Angles *angles)
{
    char text[8];
    uint8_t len;

    if (angles == NULL)
    {
        return;
    }

    OLED_ClearBuffer();

    OLED_DrawString(1U, 1U, "IMU Attitude");

    OLED_DrawString(2U, 1U, "Roll:");
    format_angle(text, angles->roll);
    OLED_DrawString(2U, 7U, text);
    len = (uint8_t)strlen(text);
    OLED_DrawChar(2U, (uint8_t)(7U + len), (char)OLED_DEGREE_CHAR);

    OLED_DrawString(3U, 1U, "Pitch:");
    format_angle(text, angles->pitch);
    OLED_DrawString(3U, 8U, text);
    len = (uint8_t)strlen(text);
    OLED_DrawChar(3U, (uint8_t)(8U + len), (char)OLED_DEGREE_CHAR);

    OLED_DrawString(4U, 1U, "Yaw:");
    format_angle(text, angles->yaw);
    OLED_DrawString(4U, 6U, text);
    len = (uint8_t)strlen(text);
    OLED_DrawChar(4U, (uint8_t)(6U + len), (char)OLED_DEGREE_CHAR);

    OLED_Flush();
}

/* 显示 IMU 状态 */
void OLED_ShowStatus(void)
{
    OLED_ClearBuffer();
    OLED_DrawString(1U, 1U, "IMU Attitude");

    if (g_imu_status == IMU_STATUS_CALIB)
    {
        OLED_DrawString(2U, 1U, "Calibrating...");
        OLED_DrawString(3U, 1U, "Keep still");
    }
    else if (g_imu_status == IMU_STATUS_ERROR)
    {
        OLED_DrawString(2U, 1U, "ICM20602 ERR");
        OLED_DrawString(3U, 1U, "Check SPI1");
        OLED_DrawString(4U, 1U, "PA4-PA7");
    }
    else if (g_imu_status == IMU_STATUS_OK)
    {
        OLED_DrawString(2U, 1U, "Waiting IMU");
    }
    else
    {
        OLED_DrawString(2U, 1U, "Starting...");
    }

    OLED_Flush();
}

