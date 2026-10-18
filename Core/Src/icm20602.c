/**
 * @file icm20602.c
 * @brief ICM20602 SPI 驱动
 */
#include "icm20602.h"
#include "cmsis_os.h"

#define ICM_CS_LOW()    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET)
#define ICM_CS_HIGH()   HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET)

#define ICM_SPI_TIMEOUT 10U      /* 中断里调用，超时要短 */

static SPI_HandleTypeDef *s_hspi = NULL;
static bool  s_ready = false;
static float s_accel_lsb_per_g  = 8192.0f;
static float s_gyro_lsb_per_dps = 65.5f;

/* ---- 诊断用全局量（OLED 上显示） ---- */
static uint8_t s_last_id  = 0x00U;    /* 读 0x75（InvenSense 系列的 ID 寄存器）*/
static uint8_t s_reg00    = 0x00U;    /* 读 0x00（Bosch/QMI 系列的 ID 寄存器）*/
static uint8_t s_reg0F    = 0x00U;    /* 读 0x0F（ST LSM6DSx 系列的 ID 寄存器）*/
static uint8_t s_diag_w55 = 0x00U;    /* 写 0x55 -> 0x19 读回 */
static uint8_t s_diag_waa = 0x00U;    /* 写 0xAA -> 0x19 读回 */
static uint8_t s_ok_cfg   = 0xFFU;
static float s_bias_gx = 0.0f;      /* 陀螺零偏 [dps] */
static float s_bias_gy = 0.0f;
static float s_bias_gz = 0.0f;
static bool    s_id_ok    = false;   /* WHO_AM_I 是否为 0x12 */    /* 成功的配置序号；0xFF = 全部失败 */

/*---------------------------------------------------------------------------*/
/* 底层传输                                                                    */
/*---------------------------------------------------------------------------*/
/* SPI 收发，自动控制 CS */
static void icm_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    ICM_CS_LOW();
    HAL_SPI_TransmitReceive(s_hspi, (uint8_t *)tx, rx, len, ICM_SPI_TIMEOUT);
    ICM_CS_HIGH();
}

/* 写寄存器 */
static void icm_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = (uint8_t)(reg & 0x7FU);
    tx[1] = val;
    icm_transfer(tx, rx, 2U);
}

/* 连续读寄存器 */
static void icm_read_regs(uint8_t reg, uint8_t *dst, uint16_t len)
{
    uint8_t tx[16];
    uint8_t rx[16];
    uint16_t i;

    if (len > 15U) { len = 15U; }
    tx[0] = (uint8_t)(reg | 0x80U);
    for (i = 1U; i <= len; i++) { tx[i] = 0x00U; }

    icm_transfer(tx, rx, (uint16_t)(len + 1U));

    for (i = 0U; i < len; i++) { dst[i] = rx[i + 1U]; }
}

static uint8_t icm_read_reg(uint8_t reg)
{
    uint8_t v = 0U;
    icm_read_regs(reg, &v, 1U);
    return v;
}

/*---------------------------------------------------------------------------*/
/* 公开接口                                                                    */
/*---------------------------------------------------------------------------*/
/* 读器件 ID */
uint8_t ICM20602_WhoAmI(void)
{
    if (s_hspi == NULL) { return 0x00U; }
    return icm_read_reg(ICM20602_REG_WHO_AM_I);
}

uint8_t ICM20602_LastId(void)   { return s_last_id;  }
uint8_t ICM20602_DiagReg00(void){ return s_reg00;    }
uint8_t ICM20602_DiagReg0F(void){ return s_reg0F;    }
uint8_t ICM20602_DiagW55(void)  { return s_diag_w55; }
uint8_t ICM20602_DiagWAA(void)  { return s_diag_waa; }
uint8_t ICM20602_OkCfg(void)    { return s_ok_cfg;   }
bool    ICM20602_IdOk(void)     { return s_id_ok;    }
bool    ICM20602_IsReady(void)  { return s_ready;    }
float   ICM20602_GyroBiasX(void) { return s_bias_gx; }
float   ICM20602_GyroBiasY(void) { return s_bias_gy; }
float   ICM20602_GyroBiasZ(void) { return s_bias_gz; }

/* 依次尝试：Mode0@9MHz -> Mode3@9MHz -> Mode0@1.1MHz -> Mode3@1.1MHz */
typedef struct {
    uint32_t polarity;
    uint32_t phase;
    uint32_t prescaler;
} icm_spi_cfg_t;

/* 低速优先：杜邦线环境下 1.1MHz 更可靠，先试；
   9MHz 作为兜底（信号完整性差时容易读出坏数据，导致姿态发散）。 */
static const icm_spi_cfg_t s_cfg_list[4] = {
    { SPI_POLARITY_LOW,  SPI_PHASE_1EDGE, SPI_BAUDRATEPRESCALER_64 },
    { SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_64 },
    { SPI_POLARITY_LOW,  SPI_PHASE_1EDGE, SPI_BAUDRATEPRESCALER_16 },
    { SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_16 },
};

static bool icm_apply_cfg(SPI_HandleTypeDef *hspi, uint8_t idx)
{
    hspi->Init.CLKPolarity       = s_cfg_list[idx].polarity;
    hspi->Init.CLKPhase          = s_cfg_list[idx].phase;
    hspi->Init.BaudRatePrescaler = s_cfg_list[idx].prescaler;
    return (HAL_SPI_Init(hspi) == HAL_OK);
}

/* 初始化，找不到芯片就返回 false */
bool ICM20602_Init(SPI_HandleTypeDef *hspi, icm20602_afs_t afs, icm20602_gfs_t gfs)
{
    static const float accel_div[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
    static const float gyro_div[4]  = {   131.0f,   65.5f,   32.8f,   16.4f };
    uint8_t i;

    if (hspi == NULL) { return false; }

    s_hspi  = hspi;
    s_ready = false;
    s_ok_cfg = 0xFFU;

    /* 片选保持低（= 选中 SPI 接口），等上电稳定 */
    ICM_CS_LOW();
    osDelay(10);                  /* 手册：上电后 2ms 才允许访问寄存器 */

    /* 依次尝试各种 SPI 时序组合，找到能读出 0x12 的那一种 */
    for (i = 0U; i < 4U; i++)
    {
        if (!icm_apply_cfg(hspi, i)) { continue; }
        osDelay(1);

        /* 手册要求：I2C_IF_DIS 必须是 SPI 模式下的第一个操作 */
        icm_write_reg(ICM20602_REG_I2C_IF, 0x40U);
        osDelay(1);

        s_last_id = ICM20602_WhoAmI();
        if (s_last_id == ICM20602_WHO_AM_I_VALUE)
        {
            s_ok_cfg = i;           /* 记住这个可用配置 */
            s_id_ok  = true;
            break;
        }
    }

    if (s_ok_cfg == 0xFFU)
    {
        /* ---------- 全部失败：采集诊断数据 ---------- */
        s_last_id = icm_read_reg(ICM20602_REG_WHO_AM_I);   /* 0x75 */
        s_reg00   = icm_read_reg(0x00U);                   /* Bosch / QMI 的 ID 寄存器 */
        s_reg0F   = icm_read_reg(0x0FU);                   /* ST LSM6DSx 的 ID 寄存器  */

        icm_write_reg(ICM20602_REG_SMPLRT_DIV, 0x55U);
        osDelay(1);
        s_diag_w55 = icm_read_reg(ICM20602_REG_SMPLRT_DIV);

        icm_write_reg(ICM20602_REG_SMPLRT_DIV, 0xAAU);
        osDelay(1);
        s_diag_waa = icm_read_reg(ICM20602_REG_SMPLRT_DIV);


        if ((s_diag_w55 == 0x55U) && (s_diag_waa == 0xAAU))
        {
            /* 写-读回环通过 = SPI 总线完全正常，只是 ID 对不上。
               可能是兼容型号/克隆芯片（寄存器映射基本一致）。
               继续初始化，用数据是否合理来验证兼容性：
               平放时 az≈1g、温度≈室温 → 说明芯片兼容。 */
            s_ok_cfg = 3U;
            s_id_ok  = false;
        }
        else
        {
            return false;     /* 真正的通信故障（接线/供电/时序）*/
        }
    }

    /* ---------- 找到可用配置，继续正常初始化 ---------- */

    icm_write_reg(ICM20602_REG_PWR_MGMT_1, 0x80U);      /* 软复位 */
    osDelay(100);

    icm_write_reg(ICM20602_REG_I2C_IF, 0x40U);          /* 再锁一次 SPI */

    icm_write_reg(ICM20602_REG_PWR_MGMT_1, 0x01U);      /* 清 sleep，选 PLL */
    osDelay(50);

    icm_write_reg(ICM20602_REG_PWR_MGMT_2, 0x00U);      /* 全部轴使能 */

    icm_write_reg(ICM20602_REG_CONFIG,        0x03U);
    icm_write_reg(ICM20602_REG_GYRO_CONFIG,   (uint8_t)((uint8_t)gfs << 3));
    icm_write_reg(ICM20602_REG_ACCEL_CONFIG,  (uint8_t)((uint8_t)afs << 3));
    icm_write_reg(ICM20602_REG_ACCEL_CONFIG2, 0x03U);
    icm_write_reg(ICM20602_REG_SMPLRT_DIV,    9U);

    s_accel_lsb_per_g  = accel_div[(uint8_t)afs & 0x03U];
    s_gyro_lsb_per_dps = gyro_div[(uint8_t)gfs & 0x03U];
    s_ready = true;
    return true;
}

/* 读原始数据 */
bool ICM20602_ReadRaw(icm20602_raw_t *raw)
{
    uint8_t b[14];

    if ((!s_ready) || (raw == NULL)) { return false; }

    icm_read_regs(ICM20602_REG_ACCEL_XOUT_H, b, 14U);

    raw->ax   = (int16_t)(((uint16_t)b[0]  << 8) | b[1]);
    raw->ay   = (int16_t)(((uint16_t)b[2]  << 8) | b[3]);
    raw->az   = (int16_t)(((uint16_t)b[4]  << 8) | b[5]);
    raw->temp = (int16_t)(((uint16_t)b[6]  << 8) | b[7]);
    raw->gx   = (int16_t)(((uint16_t)b[8]  << 8) | b[9]);
    raw->gy   = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
    raw->gz   = (int16_t)(((uint16_t)b[12] << 8) | b[13]);
    return true;
}

/* 读数据并换算成 g 和 °/s */
bool ICM20602_Read(icm20602_data_t *out)
{
    icm20602_raw_t r;

    if (out == NULL) { return false; }
    if (!ICM20602_ReadRaw(&r)) { return false; }

    out->ax_g = (float)r.ax / s_accel_lsb_per_g;
    out->ay_g = (float)r.ay / s_accel_lsb_per_g;
    out->az_g = (float)r.az / s_accel_lsb_per_g;
    out->gx_dps = ((float)r.gx / s_gyro_lsb_per_dps) - s_bias_gx;
    out->gy_dps = ((float)r.gy / s_gyro_lsb_per_dps) - s_bias_gy;
    out->gz_dps = ((float)r.gz / s_gyro_lsb_per_dps) - s_bias_gz;
    out->temp_c = ((float)r.temp / 326.8f) + 25.0f;
    return true;
}
/*---------------------------------------------------------------------------*/
/* 陀螺零偏校准：静止采样求平均。调用期间板子必须保持静止！                    */
/*---------------------------------------------------------------------------*/
/* 陀螺零偏校准，调用时板子要静止 */
bool ICM20602_CalibrateGyro(uint16_t samples)
{
    uint16_t i;
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    icm20602_raw_t r;

    if ((!s_ready) || (samples == 0U)) { return false; }

    /* 先丢弃几帧，等内部滤波器稳定 */
    for (i = 0U; i < 20U; i++)
    {
        (void)ICM20602_ReadRaw(&r);
        osDelay(2);
    }

    for (i = 0U; i < samples; i++)
    {
        if (!ICM20602_ReadRaw(&r)) { return false; }
        sum_x += (float)r.gx;
        sum_y += (float)r.gy;
        sum_z += (float)r.gz;
        osDelay(5);
    }

    s_bias_gx = (sum_x / (float)samples) / s_gyro_lsb_per_dps;
    s_bias_gy = (sum_y / (float)samples) / s_gyro_lsb_per_dps;
    s_bias_gz = (sum_z / (float)samples) / s_gyro_lsb_per_dps;
    return true;
}
/*---------------------------------------------------------------------------*/
/* 运行时零偏跟踪：gx/gy/gz 是"已减过零偏"的残余值。                          */
/* 静止时残余值理论上应为 0；不为 0 说明零偏变了（温度漂移），慢慢并进去。      */
/*---------------------------------------------------------------------------*/
/* 静止时慢慢修正零偏 */
void ICM20602_TrackGyroBias(float gx_res, float gy_res, float gz_res, float gain)
{
    if (gain <= 0.0f) { return; }
    s_bias_gx += gain * gx_res;
    s_bias_gy += gain * gy_res;
    s_bias_gz += gain * gz_res;
}





