#pragma once
#ifndef IMU_h_
#define IMU_h_

#include "config.h"

void initIMU();
void readIMU();

float imu_angle();
float imu_rate();

#endif
