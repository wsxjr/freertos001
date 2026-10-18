/**
 * @file app_oled.c
 * @brief OLED 显示任务
 */
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "app_oled.h"
#include "oled.h"

extern osMessageQueueId_t AngleQHandle;

/* 从 AngleQ 取角度并显示 */
void StartOLEDTask(void *argument)
{
    Angles angles;
    Angles last_angles;
    uint8_t have_angles = 0U;

    (void)argument;

    OLED_Init();

    for (;;)
    {
        if ((g_imu_status == IMU_STATUS_OK) && (AngleQHandle != NULL))
        {
            if (osMessageQueueGet(AngleQHandle, &angles, NULL,
                                  pdMS_TO_TICKS(200U)) == osOK)
            {
                last_angles = angles;
                have_angles = 1U;
            }
        }

        if ((g_imu_status == IMU_STATUS_OK) && (have_angles != 0U))
        {
            OLED_ShowAngles(&last_angles);
        }
        else
        {
            OLED_ShowStatus();
        }

        osDelay(100U);
    }
}

