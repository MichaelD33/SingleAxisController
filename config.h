#pragma once
#ifndef CONFIG_h
#define CONFIG_h
#include "stdint.h"

/* Define Parameters */

/* ——————————————————————————————————————————————————————DEBUGGING————————————————————————————————————————————————————————— */

  #define PRINT_SERIALDATA     // calls the printSerial() function in loop()
  #define LOOP_SAMPLING      // enables loop sampling for fixed PID and IMU sampling rates as well as loop time profiling

/* —————————————————————————————————————————————————AIRCRAFT CONFIGURATION——————————————————————————————————————————————————— */

/*  STABILIZATION MODE  */
                  
    /*    VERSION 0.4.1    7x20mm Motors   — Black PCB in Red Frame      */

    #define ACCEL_X_OFFSET (933)
    #define ACCEL_Y_OFFSET (-776)
    #define ACCEL_Z_OFFSET (1160)

    #define GYRO_X_OFFSET (108)
    #define GYRO_Y_OFFSET (13)
    #define GYRO_Z_OFFSET (-4)

    #define MPU6050_68
    
    
/* ———————————————————————————————————————————————————REMOTE CONTROL CONFIGURATION—————————————————————————————————————————————————————— */

//  TRANSMITTER GIMBAL/SWITCH OUTPUT VALUES
    #define MINTHROTTLE 172 //  minimum throttle output
    #define MAXTHROTTLE 1811 // maximum throttle output


//  SWITCH OUTPUTS
    #define ARM 1                       //  TBD
    #define MODE 2                      //  TBD
    #define BEEP 3                      //  TBD
    #define FAILSAFE 4                  //  TBD

//  SET QUADCOPTER ROATATIONAL RATE
    #define RC_RATES 180 // Maximum rotation speed: 180 degrees per second

/* ———————————————————————————————————————————————PID CONTROLLER CONFIGURATION———————————————————————————————————————————————— */

    #ifdef LOOP_SAMPLING
      #define LOOP_SAMPLETIME 3000 //define loop sample time at a frequency of 3000µs
      #define PID_SAMPLETIME_S 0.003
      #define PID_SAMPLETIME 3000 
      #define IMU_SAMPLETIME 3000
    #endif
    
    #define MAX_INTEGRAL 230  //  integral clamping to avoid writing values outside the range of pwm output
  
/* ————————————————————————————————————————————————MOTOR OUTPUT CONFIGURATION———————————————————————————————————————————————— */

//  SPEED CONTROLLER TYPE
    #define BRUSHED

// SPEED CONTROLLER CONFIG
    #define ESC_TOLERANCE 0.9        // THROTTLE MAX = (ESC_MAX * ESC_TOLERANCE)
    #define ESC_MAX 255              // 255 for BRUSHED MOTORS
    #define ESC_MIN 0                // 0 for BRUSHED MOTORS

/* ———————————————————————————————————————————INERTIAL MEASURMENT UNIT CONFIGURATION—————————————————————————————————————————— */


    #define ACC_PART (1.0 - GYRO_PART)
    #define GYRO_PART 0.985

    #define FILTER_COMPARISONS 15 //number of sample comparisons for median filter
    
    
//  IMU COMMUNICATION SETTINGS
//   #define I2C_STANDARD     
     #define I2C_FASTMODE

     #define DIGITAL_LOW_PASS_FILTER //comment this line out to deactivate the MPU6050 digital low pass filter 
//     #define DLPF_BANDWIDTH 3  // filtration bandwith configuration (coming soon)


     #define ACC_SENSITIVITY_2G
//     #define ACC_SENSITIVITY_4G
//     #define ACC_SENSITIVITY_8G
//     #define ACC_SENSITIVITY_16G

//      #define GYRO_SENSITIVITY_250
        #ifdef GYRO_SENSITIVITY_250
          #define GYRO_SENS 131
        #endif
        
      #define GYRO_SENSITIVITY_500
        #ifdef GYRO_SENSITIVITY_500
          #define GYRO_SENS 65.6
        #endif
        
//      #define GYRO_SENSITIVITY_1000
        #ifdef GYRO_SENSITIVITY_1000
          #define GYRO_SENS 32.8
        #endif
        
//      #define GYRO_SENSITIVITY_2000
        #ifdef GYRO_SENSITIVITY_2000
         #define GYRO_SENS 16.4
       #endif     



/* ———————————————————————————————————————————————CUSTOM VARIABLE STRUCTURE CONFIGURATION————————————————————————————————————————————————— */

typedef struct {
  int16_t x, y, z;
} axis_int16_t;

typedef struct {
  int16_t x, y, z;
} axis_int32_t;

typedef struct {
  float x, y, z;
} axis_float_t;

typedef struct {
  uint8_t one, two, three, four;
} int_pwmOut;

void writeMotor(int, float);
int armingState();
int lastArmingState();

#endif  //end #ifndef
