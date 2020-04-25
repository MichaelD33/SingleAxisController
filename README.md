## Single Axis Testing Controller
---
This program is based on the AIO Flight Controller. This is a *testing* program, it is not a fully functional flight controller. This program is designed to work with the [AIO quadcopter version 0.4.1](https://github.com/MichaelD33/AIO-Quadcopter-Design/tree/master/circuitry/version%200.4.1), mounted in the [single axis testing jig](https://github.com/MichaelD33/AIO-Quadcopter-Design/tree/master/test%20jig/single%20axis%20test%20jig).

Single Axis Test Setup: 
![Image of the single axis test jig](https://i2.wp.com/delaney.nyc/wp-content/uploads/2020/04/IMG_4906-scaled.jpeg)


---

### Dependancies:
This program requires jrowberg's MPU6050 and i2CDev libraries from the [i2cdevlib](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino) repository in order to function properly.

Note that there are a few features to note if you intend to use a custom flight controller with this platform.
 - The IMU for the AIO quadcopter is *not* mounted along the axis of rotation for the test jig. For this reason, the IMU output is manipulated (rotated by 45° along the z axis) in order to report the attitude of the quadcopter along the axis of rotation. This allows for the PID controller to be simplified. Nevertheless, if you are using different hardware, you will have to remove this rotation from the program. *(currently there is no parameter for controlling this, and it will have to be removed manually from the imu.cpp file)*
 
- Second: given the design of the testing setup, motors 2 and 4 were removed from the program as they were unused. Ensure that your setup will utilize motors 1 and 3 for testing, or else modify the code to suit your purposes.
