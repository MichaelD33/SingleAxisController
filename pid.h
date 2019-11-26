#pragma once
#ifndef PID_h
#define PID_h

#include "imu.h"
#include "RX.h"
#include "config.h"

/*  DEFINE PROPORTIONAL CONSTANTS */
#define Kp 0.0

#ifndef LOOP_SAMPLING

  #define Ki 0.000000
  #define Kd 0  //10

#else

/*  DEFINE INTEGRAL AND DERIVATIVE CONSTANTS ACCORDING TO PID SAMPLE TIME */    
  #define Ki 0.000000
  #define Kd 0
  
#endif


int_pwmOut motorPwmOut();

void initPids();
void resetPids();
void computePids();

#endif
