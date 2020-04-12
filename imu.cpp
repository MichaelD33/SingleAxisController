/*
 * Inertial Measurement Unit Data Acquisition and Processsing
 * 
 * 
 * Reference:
 * 
 * B00000000 - 0
 * B00001000 - 1
 * B00010000 - 2
 * B00011000 - 3
 * 
*/

#include <Arduino.h>
#include <Math.h>
#include <Wire.h>
#include "imu.h"
#include "RX.h"

float angle = 0;
float accel, TmpRaw, gyroRate, gamma;
int AcXRaw,AcYRaw,AcZRaw,GyXRaw,GyYRaw,GyZRaw;

axis_float_t gyroOutput;

median_filter_t accel_x_filter = median_filter_new(FILTER_COMPARISONS,0); //declare median filter for x axis 
median_filter_t accel_y_filter = median_filter_new(FILTER_COMPARISONS,0); //declare median filter for y axis
median_filter_t accel_z_filter = median_filter_new(FILTER_COMPARISONS,0); //declare median filter for z axis

median_filter_t gyro_x_filter = median_filter_new(FILTER_COMPARISONS,0); //declare median filter for x axis 
median_filter_t gyro_y_filter = median_filter_new(FILTER_COMPARISONS,0); //declare median filter for y axis
median_filter_t gyro_z_filter = median_filter_new(FILTER_COMPARISONS,0); //declare median filter for z axis


 void initIMU(){ 
   
   Wire.begin();

   #ifdef I2C_FASTMODE
     Wire.setClock(400000); // 400kHz I2C clock. Comment this line if microcontroller does not support 400kHz
   #endif
   
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x6B);  // PWR_MGMT_1 register
   Wire.write(0);     // set to zero (wakes up the MPU-6050)
   Wire.endTransmission(true);  
   
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x1B);  // Access register 1B - gyroscope config
   #ifdef GYRO_SENSITIVITY_250
     Wire.write(B00000000); // Setting the gyro to full scale +/- 250 deg/sec
   #elif defined GYRO_SENSITIVITY_500
     Wire.write(B00001000); // Setting the gyro to full scale +/- 500 deg/sec
   #elif defined GYRO_SENSITIVITY_1000
     Wire.write(B00010000); // Setting the gyro to full scale +/- 1000 deg/sec
   #elif defined GYRO_SENSITIVITY_2000
     Wire.write(B00011000); // Setting the gyro to full scale +/- 2000 deg/sec
   #endif
   Wire.endTransmission(true);
   
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x1C);  // Access register 1C - accelerometer config

   #ifdef ACC_SENSITIVITY_2G
     Wire.write(B00000000); // Setting the accelerometer to +/- 2g
     #define ACCEL_SENS 16384
   #elif defined ACC_SENSITIVITY_4G
     Wire.write(B00001000); // Setting the accelerometer to +/- 4g
     #define ACCEL_SENS 8192
   #elif defined ACC_SENSITIVITY_8G
     Wire.write(B00010000); // Setting the accelerometer to +/- 8g
     #define ACCEL_SENS 4096
   #elif defined ACC_SENSITIVITY_16G
     Wire.write(B00011000); // Setting the accelerometer to +/- 16g
     #define ACCEL_SENS 2048   
   #endif
  
   Wire.endTransmission(true);
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x1A);  // digital low pass filter register 0x1A

   #ifdef DIGITAL_LOW_PASS_FILTER
     Wire.write(B00000100); // ENABLING LOW PASS FILTRATION
   #else
     Wire.write(B00000000);
   #endif     
     
   Wire.endTransmission(true);
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
   Wire.endTransmission(false);

}

void readIMU(){   
   
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
   Wire.endTransmission(false);
   Wire.requestFrom(MPU_ADDR,14);  // request a total of 14 registers
   AcXRaw=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
   AcYRaw=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
   AcZRaw=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
   TmpRaw=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
   GyYRaw=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
   GyXRaw=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
   GyZRaw=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
   
   processGyro();  
   
}

void processGyro(){  

  axis_float_t gyroFiltered;

  median_filter_in(gyro_x_filter, GyXRaw);
  median_filter_in(gyro_y_filter, GyYRaw);
  median_filter_in(gyro_z_filter, GyZRaw);

  gyroFiltered.x = median_filter_out(gyro_x_filter);
  gyroFiltered.y = median_filter_out(gyro_y_filter);
  gyroFiltered.z = median_filter_out(gyro_z_filter);

  gyroOutput.x = ((gyroFiltered.x - GYRO_X_OFFSET) / GYRO_SENS);
  gyroOutput.y = ((gyroFiltered.y - GYRO_Y_OFFSET) / GYRO_SENS);
  gyroOutput.z = ((gyroFiltered.z - GYRO_Z_OFFSET) / GYRO_SENS);

  //gamma = sqrt(sq(gyroOutput.x) + sq(gyroOutput.y) + sq(gyroOutput.z));

  gyroRate = (  gyroOutput.x / sqrt(2) + gyroOutput.y / sqrt(2) );
  
  processAcc();
  imuCombine();

}

void processAcc(){
    //filtering accelerometer noise using a median filter
    axis_float_t accel_filtered; // filtered accelerometer raw values
   
    median_filter_in(accel_x_filter, AcXRaw);
    median_filter_in(accel_y_filter, AcYRaw);
    median_filter_in(accel_z_filter, AcZRaw);

    accel_filtered.x = (median_filter_out(accel_x_filter));
    accel_filtered.y = (median_filter_out(accel_y_filter));
    accel_filtered.z = (median_filter_out(accel_z_filter));

//    roll = atan2(accel_filtered.x, accel_filtered.z) * 180 / M_PI;
//    pitch = atan2(accel_filtered.y, accel_filtered.z) * 180 / M_PI;

//    float beta = sqrt(sq(accel_filtered.x) + sq(accel_filtered.y) + sq(accel_filtered.z));
//    accel = (acos(accel_filtered.z / beta) * 180 / M_PI);
     
     float beta = (  accel_filtered.x / sqrt(2) + accel_filtered.y / sqrt(2) );
     accel = atan2(beta, accel_filtered.z) * 180 / M_PI;
}

void imuCombine(){

  angle = GYRO_PART * (angle + (gyroRate * SAMPLETIME_S)) + (1-GYRO_PART) * accel;

  #ifdef PRINT_SERIALDATA
    if(chAux2() == 2){
     Serial.print("Arming: ");
     Serial.print(chAux1());
     Serial.print(", pot 1: ");
     Serial.print(chAuxPot1());
     Serial.print(", pot 2: ");
     Serial.print(chAuxPot2(), 3);
     Serial.print(", angle: ");
     Serial.print(angle);
     Serial.print(", acc: ");
     Serial.print(accel);
     Serial.print(", gyro: ");
     Serial.print(gyroRate);
    }
  #endif
   
}


int imu_rate() {
  return gyroRate;
}

float imu_angle() {
  return angle;
}
