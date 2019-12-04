#pragma once
#ifndef IMU_h_
#define IMU_h_

#include "config.h"
#include "MedianFilter.h"

#ifdef MPU6050_68
  #define MPU_ADDR 0x68
#endif

#ifdef MPU6050_69
  #define MPU_ADDR 0x69
#endif

int imu_rate();
float imu_angle();
void initIMU();
void readIMU();
void processGyro();
void processAcc();
void imuCombine();


#endif

