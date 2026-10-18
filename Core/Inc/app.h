/**
 * @file app.h
 * @brief 共用结构体和 IMU 状态
 */
#ifndef __APP_H
#define __APP_H

#include <stdint.h>

/* 姿态角，单位：度 */
typedef struct
{
    float roll;
    float pitch;
    float yaw;
    uint32_t tick_ms;
} Angles;

/* IMU 状态 */
#define IMU_STATUS_INIT    0U
#define IMU_STATUS_OK      1U
#define IMU_STATUS_ERROR   2U
#define IMU_STATUS_CALIB   3U

extern volatile uint8_t g_imu_status;

#endif /* __APP_H */
