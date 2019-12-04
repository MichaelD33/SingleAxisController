#pragma once
#ifndef PID_h
#define PID_h

#include "imu.h"
#include "RX.h"
#include "config.h"

/*  DEFINE PROPORTIONAL CONSTANTS */
#define Kp 1

#ifdef LOOP_SAMPLING

  #define Ki 0.000000 * PID_SAMPLETIME
  #define Kd 0 * PID_SAMPLETIME

#else

/*  DEFINE INTEGRAL AND DERIVATIVE CONSTANTS ACCORDING TO PID SAMPLE TIME */    
  #define Ki 0.000000
  #define Kd 0
  
#endif


byte_pwmOut motorPwmOut();

void initPids();
void resetPids();
void computePids();

#endif
