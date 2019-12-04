/*
 *  The AIO Flight Controller - An integrated quadcopter flight controller program for the Arduino platform.
 *  
 *  Copyright © 2018-2019 Michael Delaney. All rights reserved.
 * 
 *  This program takes orientation data from an inertial measurement unit in addition to input from an external remote 
 *  in order to adjust its position by varying the speed of its motors according to calculations made by the control loop.
 * 
 *  Source Code: https://github.com/MichaelD33/AIO-Quadcopter-Flight-Controller
 *  Design Files: https://github.com/MichaelD33/AIO-Quadcopter-Design

    A simple quadcopter flight controller in ~1500 lines of code

    This program is part of the AIO Flight Controller.
    
    The AIO Flight Controller is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    The AIO Flight Controller is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program; if not, see <https://www.gnu.org/licenses/>.
  
 */

#include <Arduino.h>
#include "config.h"
#include "imu.h"
#include "RX.h"
#include "pid.h"

bool armState = false;
bool lastArmState = false;

#ifdef LOOP_SAMPLING
  long indexTime;
  long imuEndTime = 0;
  long pidEndTime = 0;
  long lastStart = 0;
#endif

//version 0.4.1 configuration - Black PCB
byte motorOutput[] = {10, 9, 5, 6};  

void setup() {

  #ifdef PRINT_SERIALDATA
    Serial.begin(115200);
  #endif
  
  DDRB = DDRB | B11110000; //sets pins D8, D9, D10, D11 as outputs
  DDRC = DDRC | B11000000; //sets pin D5 as output
  DDRD = DDRD | B10000000; //sets pin D6 as output
  DDRE = DDRE | B01000000; //sets pin D7 as output

  delay(2000); //give time for RX to connect to remote
  initSbus();  //connect to the remote reciever (rx.cpp)
  initIMU();   //activate the imu and set gyroscope and accelerometer sensitivity (imu.cpp)
  
  readRx();
  readIMU();

  indexTime = micros();
  imuEndTime = micros();
  pidEndTime = micros();

}


void loop() {

  #ifdef LOOP_SAMPLING
    /*     ** LOOP TIMING **      */         
    while((micros() - lastStart) < LOOP_SAMPLETIME){
      indexTime = micros();
    }
       
    indexTime = micros();
    long lastSample = indexTime - lastStart;

    #ifdef PRINT_SERIALDATA
      if(chAux2() == 1){
        Serial.print("Last Loop Duration: ");
        Serial.print(lastSample);  
      }
    #endif
    
    lastStart = indexTime;

    /*     ** IMU TIMING **       */   
    while((micros() - imuEndTime) < IMU_SAMPLETIME){
      indexTime = micros();
    }

    #ifdef PRINT_SERIALDATA
      if(chAux2() == 1){
        Serial.print(",\t IMU: ");
        Serial.print(indexTime - imuEndTime);
      }
    #endif

    imuEndTime = indexTime;  // record end time to use for sampling calculation  
      
  #endif

  if(chAux1() == 1){
    /*      ** PID TIMING **      */ 
    #ifdef LOOP_SAMPLING            
      while((micros() - pidEndTime) < PID_SAMPLETIME){
        indexTime = micros();
      }

      #ifdef PRINT_SERIALDATA
        if(chAux2() == 1){
          Serial.print(",\t PID: ");
          Serial.print(indexTime - pidEndTime);
        }
      #endif
      
      pidEndTime = indexTime;
      
    #endif

    /*      ** PROCESS INPUT DATA **      */ 
    initPids(); 

    /*      ** SET MOTOR SPEEDS **      */ 
    //writeMotor(0, motorPwmOut().one);   //PWM motor 1
    writeMotor(1, motorPwmOut().two);   //PWM motor 2
    //writeMotor(2, motorPwmOut().three); //PWM motor 3
    writeMotor(3, motorPwmOut().four);  //PWM motor 4
            
    armState = true;

  }else{
      /*  IF DEVICE DISARMS ——> DISABLE MOTORS */
      // writeMotor(0, 0);
      writeMotor(1, 0);
      // writeMotor(2, 0);
      writeMotor(3, 0);

  }

    

   /*     ** IMU DATA COLLECTION **       */
  readIMU(); //read the imu and calculate the quadcopters position relative to gravity (imu.cpp)
              // IMU supports up to 8kHz gyro update rate and 1kHz acc update rate --- when DLPF is activated this is diminished significantly (see MPU6050 register mapping datasheet)
  
   /*     ** RX DATA COLLECTION **       */
  readRx();  //read the remote and convert data to a rotational rate of ±180°/s (rx.cpp)



  lastArmState = armState;
  
  #ifdef PRINT_SERIALDATA
    printSerial(); // used for GUI application and debugging
    Serial.println("");
  #endif

}


void writeMotor(int index, int value){
  analogWrite(motorOutput[index], (uint8_t)(value));
}

int armingState(){
  return armState;
}

int lastArmingState(){
  return lastArmState;
}
  
void printSerial(){

/*

  switch(chAux2()){
    
    case 0: break;

    case 1: break;

    case 2: break;

    default: break;
    
  }
*/
    
}
