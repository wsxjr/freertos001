/**
 * @file attitude.c
 * @brief 互补滤波姿态解算
 */
#include "attitude.h"
#include <math.h>

#define RAD_TO_DEG       57.2957795f
#define FILTER_ALPHA     0.98f      /* 陀螺权重：0.98 陀螺 + 0.02 加速度计 */
#define YAW_DEADBAND_DPS 0.35f      /* YAW 积分死区：更小角速度不积分      */

/* ---------- 静止检测（用于抑制 YAW 静止漂移）---------- */
#define STILL_GYRO_DPS   1.5f       /* 三轴角速度都小于此值 -> 认为静止     */
#define STILL_ACC_MIN    0.90f      /* 同时要求 |a| 接近 1g                 */
#define STILL_ACC_MAX    1.10f
#define STILL_CONFIRM    20U        /* 连续 20 次(100Hz 下 0.2s)确认静止     */
#define BIAS_TRACK_GAIN  0.002f     /* 零偏跟踪速度（时间常数约 5 秒）      */

/* ---------- 安装方向配置 ----------
   如果实际转动方向和显示方向相反，把对应轴改成 1。
   注意：翻转某个轴时，加速度计与陀螺**必须同时翻转**，否则互补滤波会正反馈发散。 */
#define INVERT_PITCH_AXIS  0
#define INVERT_ROLL_AXIS   0
#define INVERT_YAW_AXIS    0

static volatile float s_pitch = 0.0f;
static volatile float s_roll  = 0.0f;
static volatile float s_yaw   = 0.0f;
static volatile uint8_t s_started = 0U;
static icm20602_data_t s_last_imu;      /* 供显示/调试读取 */
static uint16_t s_still_cnt = 0U;       /* 连续静止计数 */

/* 复位角度 */
void Attitude_Reset(void)
{
    s_pitch   = 0.0f;
    s_roll    = 0.0f;
    s_yaw     = 0.0f;
    s_started = 0U;
}

/* 用一帧 IMU 数据更新姿态 */
void Attitude_Update(const icm20602_data_t *imu, float dt_sec)
{
    float ax, ay, az;
    float gx, gy, gz;
    float acc_pitch, acc_roll;

    if (imu == NULL) { return; }

    s_last_imu = *imu;

    ax = imu->ax_g;
    ay = imu->ay_g;
    az = imu->az_g;
    gx = imu->gx_dps;
    gy = imu->gy_dps;
    gz = imu->gz_dps;

#if INVERT_PITCH_AXIS
    ax = -ax; gy = -gy;
#endif
#if INVERT_ROLL_AXIS
    ay = -ay; gx = -gx;
#endif
#if INVERT_YAW_AXIS
    gz = -gz;
#endif

    /* 0) 静止检测：三轴角速度都很小 且 加速度模长接近 1g
          （确认静止后：冻结 YAW 积分 + 缓慢跟踪残余零偏）*/
    {
        float acc_norm = sqrtf((ax * ax) + (ay * ay) + (az * az));

        if ((fabsf(gx) < STILL_GYRO_DPS) && (fabsf(gy) < STILL_GYRO_DPS) &&
            (fabsf(gz) < STILL_GYRO_DPS) &&
            (acc_norm > STILL_ACC_MIN) && (acc_norm < STILL_ACC_MAX))
        {
            if (s_still_cnt < 0xFFFFU) { s_still_cnt++; }
        }
        else
        {
            s_still_cnt = 0U;
        }

        if (s_still_cnt > STILL_CONFIRM)
        {
            /* 确认静止：把残余角速度并进零偏（补偿温度漂移）*/
            ICM20602_TrackGyroBias(gx, gy, gz, BIAS_TRACK_GAIN);
        }
    }

    /* 1) 加速度计算倾角（绝对基准，长期不漂）*/
    acc_pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * RAD_TO_DEG;
    acc_roll  = atan2f( ay, az) * RAD_TO_DEG;

    /* 2) 首次：直接用加速度计初始化 */
    if (s_started == 0U)
    {
        s_started = 1U;
        s_pitch   = acc_pitch;
        s_roll    = acc_roll;
        s_yaw     = 0.0f;
        return;
    }

    if (dt_sec <= 0.0f) { return; }

    /* 3) 互补滤波：陀螺积分(响应快) + 加速度计修正(消除漂移) */
    s_pitch = (FILTER_ALPHA * (s_pitch + (gy * dt_sec)))
            + ((1.0f - FILTER_ALPHA) * acc_pitch);

    s_roll  = (FILTER_ALPHA * (s_roll + (gx * dt_sec)))
            + ((1.0f - FILTER_ALPHA) * acc_roll);

    /* 4) 偏航角：只有陀螺 Z 积分（会漂移）。加死区抑制静止漂移。 */
    if (s_still_cnt <= STILL_CONFIRM)          /* 非静止才积分 */
    {
        if ((gz > YAW_DEADBAND_DPS) || (gz < -YAW_DEADBAND_DPS))
        {
            s_yaw += gz * dt_sec;
            if (s_yaw >  180.0f) { s_yaw -= 360.0f; }
            if (s_yaw < -180.0f) { s_yaw += 360.0f; }
        }
    }
    /* 静止时 s_yaw 保持不变 -> 放桌上不再漂 */

    /* 限幅（PITCH 物理范围就是 ±90）*/
    if (s_pitch >  90.0f) { s_pitch =  90.0f; }
    if (s_pitch < -90.0f) { s_pitch = -90.0f; }
}

/* 当前 Pitch */
float Attitude_Pitch(void) { return s_pitch; }
/* 当前 Roll */
float Attitude_Roll(void)  { return s_roll;  }
/* 当前 Yaw */
float Attitude_Yaw(void)   { return s_yaw;   }
/* 最近一帧 IMU 数据 */
const icm20602_data_t *Attitude_LastImu(void) { return &s_last_imu; }



