/**
 * @file icm20602.h
 * @brief ICM20602 六轴 IMU SPI 驱动
 *
 * PA4 = CS
 * PA5 = SCK
 * PA6 = MISO
 * PA7 = MOSI
 */
#ifndef __ICM20602_H
#define __ICM20602_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* 寄存器地址 */
#define ICM20602_REG_SMPLRT_DIV     0x19U
#define ICM20602_REG_CONFIG         0x1AU
#define ICM20602_REG_GYRO_CONFIG    0x1BU
#define ICM20602_REG_ACCEL_CONFIG   0x1CU
#define ICM20602_REG_ACCEL_CONFIG2  0x1DU
#define ICM20602_REG_FIFO_EN        0x23U
#define ICM20602_REG_INT_PIN_CFG    0x37U
#define ICM20602_REG_INT_ENABLE     0x38U
#define ICM20602_REG_INT_STATUS     0x3AU
#define ICM20602_REG_ACCEL_XOUT_H   0x3BU
#define ICM20602_REG_TEMP_OUT_H     0x41U
#define ICM20602_REG_GYRO_XOUT_H    0x43U
#define ICM20602_REG_USER_CTRL      0x6AU
#define ICM20602_REG_PWR_MGMT_1     0x6BU
#define ICM20602_REG_PWR_MGMT_2     0x6CU
#define ICM20602_REG_I2C_IF         0x70U
#define ICM20602_REG_WHO_AM_I       0x75U

#define ICM20602_WHO_AM_I_VALUE     0x12U

/* 加速度量程 */
typedef enum {
    ICM20602_AFS_2G  = 0,
    ICM20602_AFS_4G  = 1,
    ICM20602_AFS_8G  = 2,
    ICM20602_AFS_16G = 3
} icm20602_afs_t;

/* 陀螺量程 */
typedef enum {
    ICM20602_GFS_250DPS  = 0,
    ICM20602_GFS_500DPS  = 1,
    ICM20602_GFS_1000DPS = 2,
    ICM20602_GFS_2000DPS = 3
} icm20602_gfs_t;

/* 原始数据 */
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
} icm20602_raw_t;

/* 转换后的数据 */
typedef struct {
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;
    float temp_c;
} icm20602_data_t;

bool ICM20602_Init(SPI_HandleTypeDef *hspi, icm20602_afs_t afs, icm20602_gfs_t gfs);
bool ICM20602_IsReady(void);
uint8_t ICM20602_WhoAmI(void);

/* 校准期间板子必须静止 */
bool ICM20602_CalibrateGyro(uint16_t samples);

float ICM20602_GyroBiasX(void);
float ICM20602_GyroBiasY(void);
float ICM20602_GyroBiasZ(void);

/* 静止时慢慢修正零偏 */
void ICM20602_TrackGyroBias(float gx_res, float gy_res, float gz_res, float gain);

/* 诊断接口 */
uint8_t ICM20602_LastId(void);
uint8_t ICM20602_DiagReg00(void);
uint8_t ICM20602_DiagReg0F(void);
uint8_t ICM20602_DiagW55(void);
uint8_t ICM20602_DiagWAA(void);
uint8_t ICM20602_OkCfg(void);
bool ICM20602_IdOk(void);

bool ICM20602_ReadRaw(icm20602_raw_t *raw);
bool ICM20602_Read(icm20602_data_t *out);

#endif /* __ICM20602_H */
