/*
    The PID Controller makes adjustments to the motor speeds in order to adjust orientation in the desired angle/direction
*/
#include "pid.h"
#include "config.h"


unsigned long lastTime, currentT;

int desiredAngle;
float currentAngle, lastAngle;
float error, deltaError, errorSum;
float output;

int_pwmOut motorSpeed;

void initPids(){
  //time since last calculation
    
    computePids();
    resetPids();
    lastAngle = currentAngle;
    lastTime = currentT;
} 
    

void computePids(){

    #ifndef LOOP_SAMPLING
      currentT = micros();
    #endif
    
    #ifdef HORIZON
      currentAngle = (sqrt(imu_angles().y*imu_angles().y + imu_angles().x*imu_angles().x) * tan(imu_angles().y / imu_angles().x)); //read angle from IMU and set it to the current angle
    #endif

    #ifdef ACRO
      currentAngle = (sqrt(imu_rates().y*imu_rates().y + imu_rates().x*imu_rates().x) * tan(imu_rates().y / imu_rates().x)); //read gyro rates from IMU and set it to the current angle
    #endif

    Serial.print(currentAngle);
    
    // error = (-1 * chRoll())  - currentAngle ;
    error = 0 - currentAngle;
    
    #ifndef LOOP_SAMPLING
      long timeChange = (currentT - lastTime);
      
      errorSum += error * timeChange;
      deltaError = (currentAngle - lastAngle) / timeChange;       
    #else
      errorSum += error;
      deltaError = (currentAngle - lastAngle);
    #endif
    
    //compute integral
    float integral = Ki * errorSum;


    
    //clamp the range of integral values
    if(integral > MAX_INTEGRAL){ 
      integral = MAX_INTEGRAL; 
    }else if (integral < (MAX_INTEGRAL * -1)){
      integral = MAX_INTEGRAL * -1;
    }

    output = (Kp * error + integral - Kd * deltaError);

/*
    #ifdef PRINT_SERIALDATA
      if(chAux2() == 0){
        Serial.print(",");
        Serial.print(output);
      }
    #endif


    //removed integral clamping
    outputX = (Kp * error + Ki * errorSum - Kd * deltaError);

    //compute PID output as a function of the throttle
    outputX = (Kp * error + integral - Kd * deltaError) * (chThrottle() / (ESC_MAX * ESC_TOLERANCE));
*/
 
  
    //write outputs to corresponding motors at the corresponding speed
     //motorSpeed.one = abs(chThrottle() + outputX - outputY - outputZ); 
     motorSpeed.two = abs(chThrottle() - output); 
     //motorSpeed.three = abs(chThrottle() - outputX + outputY - outputZ);
     motorSpeed.four = abs(chThrottle() + output);
     
     //clamp the min and max output from the pid controller (to match the needed 0-255 for pwm)
     if(motorSpeed.one > ESC_MAX){
        motorSpeed.one = ESC_MAX;  
       }else if (motorSpeed.one < ESC_MIN){
        motorSpeed.one = ESC_MIN;
       }else{ }  

     if(motorSpeed.two > ESC_MAX){
        motorSpeed.two = ESC_MAX;  
       }else if (motorSpeed.two < ESC_MIN){
        motorSpeed.two = ESC_MIN;
       }else{ } 

     if(motorSpeed.three > ESC_MAX){
        motorSpeed.three = ESC_MAX;  
       }else if (motorSpeed.three < ESC_MIN){
        motorSpeed.three = ESC_MIN;
       }else{ } 

     if(motorSpeed.four > ESC_MAX){
        motorSpeed.four = ESC_MAX;  
       }else if (motorSpeed.four < ESC_MIN){
        motorSpeed.four = ESC_MIN;
       }else{ } 
           
}

void resetPids(){

   if(chThrottle() < 10){    
      errorSum = 0;
      
    }else if(armingState() != lastArmingState()){
    //reset the integral term when the quadcopter is armed
      errorSum = 0;
    
  }
       
}

int_pwmOut motorPwmOut(){
  return motorSpeed;
}
