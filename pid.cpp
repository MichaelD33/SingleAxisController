/*
    The PID Controller makes adjustments to the motor speeds in order to adjust orientation in the desired angle/direction
*/
#include <Arduino.h>
#include "pid.h"
#include "config.h"

// Rate PIDs
float KpR = 0;
float KiR = 0.0;
float KdR = 0;


// Stabilization PIDs
float Kp = 0; //1.5; // 1;
float Ki = 0.0;
float Kd = 0; //3.5?

int desiredAngle;
float currentAngle, lastAngle;
float error, deltaError, errorSum, integral;
float output;

float lastRate, currentRate, errorRate, deltaRate, rateIntegral;

byte_pwmOut motorSpeed;
int_pwmOut motorSpeed_raw;

void initPids(){
  //time since last calculation

 /* Call the computePID function to calculate the motor speeds for the quadcopter */
 
    computePids();
    resetPids();

/* Store the previous loop's measurements in order to calculate derivative for PID controller(s) */
    lastAngle = currentAngle;
    lastRate = currentRate;

    KpR = (0 + chAuxPot1() + chAuxPot2());
//    Ki = (0 + (2*chAuxPot1()) + (4*chAuxPot2())) * SAMPLETIME_S;
//    Kd = (0 + (chAuxPot1()/5) + (chAuxPot2()/4)) / SAMPLETIME_S;
} 
    

void computePids(){


/*  —————————————————————————————————— The Rate PID Controller  ——————————————————————————————————  */

    currentRate = imu_rate();

    errorRate = (-1 * chPitch()) - currentRate;   // P
    rateIntegral += KiR * errorRate;              // I
    deltaRate = currentRate - lastRate;           // D

    if(rateIntegral > MAX_INTEGRAL){ 
      rateIntegral = MAX_INTEGRAL; 
    }else if (rateIntegral < (0 - MAX_INTEGRAL)){
      rateIntegral = (0 - MAX_INTEGRAL);
    }

    output = (KpR * errorRate + rateIntegral - KdR * deltaRate);
    
    motorSpeed_raw.two = chThrottle() - output; 
    motorSpeed_raw.four = chThrottle() + output;

    if(motorSpeed_raw.two > ESC_MAX){
      motorSpeed_raw.two = ESC_MAX;  
    }else if (motorSpeed_raw.two < ESC_MIN){
      motorSpeed_raw.two = ESC_MIN;
    }else{ } 

    if(motorSpeed_raw.four > ESC_MAX){
      motorSpeed_raw.four = ESC_MAX;  
    }else if (motorSpeed_raw.four < ESC_MIN){
      motorSpeed_raw.four = ESC_MIN;
    }else{ } 

    motorSpeed.two = motorSpeed_raw.two;
    motorSpeed.four = motorSpeed_raw.four;
    
    if(chAux2() == 0){
      Serial.print("IMU: ");      
      Serial.print(imu_angle());
      Serial.print(", Kp: ");
      Serial.print(KpR);
      Serial.print(", Ki: ");
      Serial.print(KiR, 4);
      Serial.print(", Kd: ");
      Serial.println(KdR);
    }


/*  —————————————————————————————————— The Angle PID Controller  ——————————————————————————————————  
    // read angle from IMU and set it to the current angle
    currentAngle = imu_angle(); 

    // determine the difference between the position and setpoint
//    int setpoint = 0;
//    if(chAux3() == 1){
//      setpoint = 30;
//    }else{
//      setpoint = 0;
//    }
//    
//    error = (setpoint) - currentAngle;


    error = (-1 * chPitch()) - currentAngle;

    // determine the change in difference over time (derivative)
    deltaError = (currentAngle - lastAngle);
    
    // compute the accumulation of error over time (integral)
    integral += Ki * error;
    
    // clamp the range of integral values
    if(integral > MAX_INTEGRAL){ 
      integral = MAX_INTEGRAL; 
    }else if (integral < (0 - MAX_INTEGRAL)){
      integral = (0 - MAX_INTEGRAL);
    }

    output = (Kp * error + integral - Kd * deltaError);
 
  
    //write outputs to corresponding motors at the corresponding speed
// ——————————————————————————————————————————————————————————————————————— //    
     //motorSpeed.one = abs(chThrottle() + outputX - outputY - outputZ); 
     motorSpeed_raw.two = chThrottle() - output; 
     //motorSpeed.three = abs(chThrottle() - outputX + outputY - outputZ);
     motorSpeed_raw.four = chThrottle() + output;
     
     //clamp the min and max output from the pid controller (to match the needed 0-255 for pwm)
// ——————————————————————————————————————————————————————————————————————————————————————————————— //
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

*/
           
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
