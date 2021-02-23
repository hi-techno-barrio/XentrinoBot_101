#ifndef xentrino_config_h_
#define xentrino_config_h_

#include "Arduino.h"
// #include "xentrino_base_config.h"
// #include "xentrino.h"
// #ifndef XENTRINOBOT_CONFIG_H

// sign funciton
#define sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
// utility defines
#define _2_SQRT3 1.15470053838
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI_2 1.57079632679
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038


//uncomment the base you're building
#define XENTRINOBOT_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors
// #define XENTRINOBOT_BASE SKID_STEER      // 4WD robot
// #define XENTRINOBOT_BASE ACKERMANN       // Car-like steering robot w/ 2 motors
// #define XENTRINOBOT_BASE ACKERMANN1      // Car-like steering robot w/ 1 motor
// #define XENTRINOBOT_BASE MECANUM         // Mecanum drive robot

//uncomment the motor driver you're using
#define USE_L298_DRIVER
// #define USE_BTS7960_DRIVER
// #define USE_ESC

//uncomment the IMU you're using
//#define USE_GY85_IMU
#define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

#define DEBUG 1

#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant

//define your robot' specs here
#define MAX_RPM 330               // motor's maximum RPM
#define COUNTS_PER_REV 1550       // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.10       // wheel's diameter in meters
#define PWM_BITS 8                // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.235  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
#define MAX_STEERING_ANGLE 0.415  // max steering angle. This only applies to Ackermann steering

//=================BIGGER ROBOT SPEC (BTS7960)=============================
// #define K_P 0.05  // P constant
// #define K_I 0.9   // I constant
// #define K_D 0.1   // D constant

// define your robot' specs here
// #define MAX_RPM 45               // motor's maximum RPM
// #define COUNTS_PER_REV 4000      // wheel encoder's no of ticks per rev
// #define WHEEL_DIAMETER 0.15      // wheel's diameter in meters
// #define PWM_BITS 8               // PWM Resolution of the microcontroller
// #define LR_WHEELS_DISTANCE 0.32  // distance between left and right wheels
// #define FR_WHEELS_DISTANCE 0.38  // distance between front and back wheels. Ignore this if you're on 2WD/ACKERMANN
//================= END OF BIGGER ROBOT SPEC =============================

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

/// ENCODER PINS
#define MOTOR1_ENCODER_A 15
#define MOTOR1_ENCODER_B 14 

#define MOTOR2_ENCODER_A 11
#define MOTOR2_ENCODER_B 12 

#define MOTOR3_ENCODER_A 17
#define MOTOR3_ENCODER_B 16 

#define MOTOR4_ENCODER_A 9
#define MOTOR4_ENCODER_B 10

//MOTOR PINS
#ifdef USE_L298_DRIVER
  #define CONTROLLER L298N

  #define MOTOR1_PWM 21
  #define MOTOR1_IN_A 20
  #define MOTOR1_IN_B 1

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A 8
  #define MOTOR2_IN_B 6

  #define MOTOR3_PWM 22
  #define MOTOR3_IN_A 23
  #define MOTOR3_IN_B 0

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 2
  #define MOTOR4_IN_B 3

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

#endif  // header end
