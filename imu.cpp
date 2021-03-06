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

#include "src/I2Cdev.h"
#include "src/MPU6050_6Axis_MotionApps20.h"
#include "src/helper_3dmath.h"

MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float angle;
float virtualRate, virtualRate2, rate2Angle;
int rateArray[3];

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

 void initIMU(){ 
   
   Wire.begin();
   Wire.setClock(400000); // 400kHz I2C clock. Comment this line if microcontroller does not support 400kHz
   
   Serial.println(F("Initializing I2C devices..."));
   mpu.initialize();
   pinMode(INTERRUPT_PIN, INPUT);

   // verify connection
   Serial.println(F("Testing device connections..."));
   Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

   // wait for ready
   Serial.println(F("\nSend any character to begin DMP programming and demo: "));
   while (Serial.available() && Serial.read()); // empty buffer
   while (!Serial.available());                 // wait for data
   while (Serial.available() && Serial.read()); // empty buffer again

   // configure the DMP
   Serial.println(F("Initializing DMP..."));
   devStatus = mpu.dmpInitialize();

   // supply your own gyro offsets here, scaled for min sensitivity
   mpu.setXGyroOffset(GYRO_X_OFFSET);
   mpu.setYGyroOffset(GYRO_Y_OFFSET);
   mpu.setZGyroOffset(GYRO_Z_OFFSET);
   mpu.setZAccelOffset(ACCEL_Z_OFFSET);

   if (devStatus == 0) {
       // turn on the DMP, now that it's ready
       Serial.println(F("Enabling DMP..."));
       mpu.setDMPEnabled(true);

       // enable Arduino interrupt detection
       Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
       attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
       mpuIntStatus = mpu.getIntStatus();

       // set our DMP Ready flag so the main loop() function knows it's okay to use it
       Serial.println(F("DMP ready! Waiting for first interrupt..."));
       dmpReady = true;

        // get expected DMP packet size for later comparison
       packetSize = mpu.dmpGetFIFOPacketSize();
   } else {
       // ERROR!
       // 1 = initial memory load failed
       // 2 = DMP configuration updates failed
       // (if it's going to break, usually the code will be 1)
       Serial.print(F("DMP Initialization failed (code "));
       Serial.print(devStatus);
       Serial.println(F(")"));
    }

}

void readIMU(){   
   
  if (!dmpReady) return;
  
      // wait for MPU interrupt or extra packet(s) available
      while (!mpuInterrupt && fifoCount < packetSize) {
          
          // other program behavior stuff here

          // if you are really paranoid you can frequently test in between other
          // stuff to see if mpuInterrupt is true, and if so, "break;" from the
          // while() loop to immediately process the MPU data

      }
  
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();
  
      // get current FIFO count
      fifoCount = mpu.getFIFOCount();
  
      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
          // reset so we can continue cleanly
          mpu.resetFIFO();
          Serial.println(F("FIFO overflow!"));
  
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
          // wait for correct available data length, should be a VERY short wait
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
          // read a packet from FIFO
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount -= packetSize;


    /*  —————————————————————————————————— IMU ANGLE OUTPUT——————————————————————————————————  */
    
          mpu.dmpGetQuaternion(&q, fifoBuffer);

    /* APPLYING VIRTUAL ROTATION TO QUATERNION DATA */
          
          Quaternion p(sin(M_PI/8), 0, 0, cos(M_PI/8));

          // quaternion multiplication: q * p, stored back in p
          p = q.getProduct(p);
      
          // quaternion multiplication: p * conj(q), stored back in p
          p.getProduct(q.getConjugate());

     /* CONVERT QUATERNION TO YPR ANGLES */

          mpu.dmpGetGravity(&gravity, &p);
          mpu.dmpGetYawPitchRoll(ypr, &p, &gravity);
          


          angle = 0 - (ypr[2] * 180/M_PI);

      /*  —————————————————————————————————— IMU RATE OUTPUT  ——————————————————————————————————  */

          mpu.dmpGetGyro(rateArray, fifoBuffer);
          

          virtualRate = (rateArray[0]/sqrt(2) - rateArray[1]/sqrt(2));
         // rate2Angle += virtualRate * SAMPLETIME_S;
          virtualRate2 = (rateArray[0]/sqrt(2) + rateArray[1]/sqrt(2));

          #ifdef PRINT_SERIALDATA
            if(chAux2() == 2){ 
 //             Serial.print("a: ");
 //             Serial.print(ypr[2] * 180/M_PI);
              Serial.print("RaY:");
              Serial.print(rateArray[0]);
              Serial.print(",RaX:");
              Serial.print(rateArray[1]);
              Serial.print(",vR:");
              Serial.print(virtualRate);
              Serial.print(",vR2:");
              Serial.println(virtualRate2);
 //             Serial.print(", r2a: ");
 //             Serial.println(rate2Angle);
              
          }
          #endif

          
      }
   
}

float imu_rate(){
  return virtualRate;
}

float imu_angle(){
  return angle;
}
