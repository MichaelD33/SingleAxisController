#pragma once
#ifndef PID_h
#define PID_h

#include "imu.h"
#include "RX.h"
#include "config.h"

byte_pwmOut motorPwmOut();

void initPids();
void resetPids();
void computePids();


#endif
