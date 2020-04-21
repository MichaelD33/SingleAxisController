/*
    The PID Controller makes adjustments to the motor speeds in order to adjust orientation in the desired angle/direction
*/
#include <Arduino.h>
#include "pid.h"
#include "config.h"


unsigned long lastTime, currentT;

float Kp = 0;
float Ki = 0;
float Kd = 0;

int desiredAngle;
float currentAngle, lastAngle;
float error, deltaError, errorSum, integral;
float output;

byte_pwmOut motorSpeed;
int_pwmOut motorSpeed_raw;

void initPids(){
  //time since last calculation
    
    computePids();
    resetPids();
    lastAngle = currentAngle;
    lastTime = currentT;

    Kp = (1 + chAuxPot1() + chAuxPot2());
    Ki = 0 * SAMPLETIME;
    Kd = 0 / SAMPLETIME;
} 
    

void computePids(){
    
    currentAngle = imu_angle(); //read angle from IMU and set it to the current angle
   
    // error = (-1 * chRoll())  - currentAngle ;
    error = 0 - currentAngle;
    
//      errorSum += error;
    deltaError = (currentAngle - lastAngle);
    
    //compute integral
//    float integral = Ki * errorSum;
    integral += Ki * error;
    
    //clamp the range of integral values
    if(integral > MAX_INTEGRAL){ 
      integral = MAX_INTEGRAL; 
    }else if (integral < (0 - MAX_INTEGRAL)){
      integral = (0 - MAX_INTEGRAL);
    }

    output = (Kp * error + integral - Kd * deltaError);
 
  
    //write outputs to corresponding motors at the corresponding speed
     //motorSpeed.one = abs(chThrottle() + outputX - outputY - outputZ); 
     motorSpeed_raw.two = chThrottle() - output; 
     //motorSpeed.three = abs(chThrottle() - outputX + outputY - outputZ);
     motorSpeed_raw.four = chThrottle() + output;
     
     //clamp the min and max output from the pid controller (to match the needed 0-255 for pwm)

//     if(motorSpeed.one > ESC_MAX){
//        motorSpeed.one = ESC_MAX;  
//       }else if (motorSpeed.one < ESC_MIN){
//        motorSpeed.one = ESC_MIN;
//       }else{ }  

     if(motorSpeed_raw.two > ESC_MAX){
        motorSpeed_raw.two = ESC_MAX;  
       }else if (motorSpeed_raw.two < ESC_MIN){
        motorSpeed_raw.two = ESC_MIN;
       }else{ } 

//     if(motorSpeed.three > ESC_MAX){
//        motorSpeed.three = ESC_MAX;  
//       }else if (motorSpeed.three < ESC_MIN){
//        motorSpeed.three = ESC_MIN;
//       }else{ } 

     if(motorSpeed_raw.four > ESC_MAX){
        motorSpeed_raw.four = ESC_MAX;  
       }else if (motorSpeed_raw.four < ESC_MIN){
        motorSpeed_raw.four = ESC_MIN;
       }else{ } 

    motorSpeed.two = motorSpeed_raw.two;
    motorSpeed.four = motorSpeed_raw.four;

    if(chAux2() == 0){
      Serial.print(chAux1());
      Serial.print(",");
      Serial.print(chAuxPot1());
      Serial.print(",");
      Serial.print(chAuxPot2(), 3);
      Serial.print(",");
      Serial.print(Kp);
      Serial.print(",");
      Serial.print(output);
      Serial.print(",");
      Serial.print(motorSpeed.two);
      Serial.print(",");
      Serial.print(motorSpeed.four);
    }
           
}

void resetPids(){

   if(chThrottle() < 10){    
      errorSum = 0;
      
    }else if(armingState() != lastArmingState()){
    //reset the integral term when the quadcopter is armed
      errorSum = 0;
    
  }
       
}

byte_pwmOut motorPwmOut(){
  return motorSpeed;
}
