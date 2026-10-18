/**
 * @file attitude.h
 * @brief 姿态解算接口
 */
#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "icm20602.h"

void Attitude_Update(const icm20602_data_t *imu, float dt_sec);
void Attitude_Reset(void);

float Attitude_Pitch(void);
float Attitude_Roll(void);
float Attitude_Yaw(void);

const icm20602_data_t *Attitude_LastImu(void);

#endif /* __ATTITUDE_H */
