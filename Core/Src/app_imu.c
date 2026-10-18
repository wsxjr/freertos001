/**
 * @file app_imu.c
 * @brief IMU 采样和姿态解算
 */
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "spi.h"
#include "app.h"
#include "app_imu.h"
#include "icm20602.h"
#include "attitude.h"

extern osMessageQueueId_t AngleQHandle;

/* IMU 状态，OLED 任务会读 */
volatile uint8_t g_imu_status = IMU_STATUS_INIT;

/* IMU 任务：初始化、校准、100Hz 读取和解算 */
void StartIMUTask(void *argument)
{
    icm20602_data_t imu;
    Angles angles;
    uint32_t last_tick;
    uint8_t fail_count = 0U;
    uint8_t icm_ok = 0U;

    (void)argument;

    last_tick = osKernelGetTickCount();

    for (;;)
    {
        if (icm_ok == 0U)
        {
            g_imu_status = IMU_STATUS_INIT;

            if (ICM20602_Init(&hspi1, ICM20602_AFS_4G, ICM20602_GFS_500DPS))
            {
                g_imu_status = IMU_STATUS_CALIB;

                if (ICM20602_CalibrateGyro(200U))
                {
                    Attitude_Reset();
                    icm_ok = 1U;
                    g_imu_status = IMU_STATUS_OK;
                    last_tick = osKernelGetTickCount();
                    fail_count = 0U;
                }
                else
                {
                    icm_ok = 0U;
                    g_imu_status = IMU_STATUS_ERROR;
                }
            }
            else
            {
                g_imu_status = IMU_STATUS_ERROR;
            }

            if (icm_ok == 0U)
            {
                osDelay(500U);
                continue;
            }
        }

        if (ICM20602_Read(&imu))
        {
            uint32_t now = osKernelGetTickCount();
            float dt = (float)(now - last_tick) / 1000.0f;

            last_tick = now;

            if (dt < 0.001f)
            {
                dt = 0.001f;
            }
            if (dt > 0.050f)
            {
                dt = 0.050f;
            }

            Attitude_Update(&imu, dt);

            angles.roll = Attitude_Roll();
            angles.pitch = Attitude_Pitch();
            angles.yaw = Attitude_Yaw();
            angles.tick_ms = now;

            if (AngleQHandle != NULL)
            {
                (void)osMessageQueuePut(AngleQHandle, &angles, 0U, 0U);
            }

            fail_count = 0U;
        }
        else
        {
            if (fail_count < 255U)
            {
                fail_count++;
            }
            if (fail_count >= 10U)
            {
                icm_ok = 0U;
                g_imu_status = IMU_STATUS_ERROR;
            }
        }

        osDelay(10U);
    }
}

