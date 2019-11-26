#pragma once
#ifndef PID_h
#define PID_h

#include "imu.h"
#include "RX.h"
#include "config.h"

/*  DEFINE PROPORTIONAL CONSTANTS */
#define KpX 0.0
#define KpY 0.0
#define KpZ 0.00

#ifndef LOOP_SAMPLING

  #define KiX 0.000000
  #define KiY 0.000000
  #define KiZ 0.0

  #define KdX 0  //10
  #define KdY 0 //10
  #define KdZ 0  //7

#else

/*  DEFINE INTEGRAL AND DERIVATIVE CONSTANTS ACCORDING TO PID SAMPLE TIME */    
  #define KiX 0.000000 * PID_SAMPLETIME  //0000005
  #define KiY 0.000000 * PID_SAMPLETIME  //0000005
  #define KiZ 0.0 * PID_SAMPLETIME  //0000005

  #define KdX 3 / PID_SAMPLETIME  //10
  #define KdY 3 / PID_SAMPLETIME  //10
  #define KdZ 10 / PID_SAMPLETIME  //7
  
#endif


int_pwmOut motorPwmOut();

void initPids();
void resetPids();
void computePids();

#endif
