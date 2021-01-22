/*
/*
Christopher Coballes
Hi-Techno Barrio,Philippines
*/

#define MOTOR_DRIVER L298N

// #define pwm_pin1 9  ;
// #define pwm_pin2 2  ;
// #define  pinM1_1 10 ;
// #define  pinM1_2 12 ;
// #define  pinM2_1 8  ;
// #define  pinM2_2 7  ;

#include <Motor.h>

Motor motor1(Motor::MOTOR_DRIVER,10,12,9);
Motor motor2(Motor::MOTOR_DRIVER,8,7,2 );

void setup() 
{
  
}

void loop() 
{
  motor1.forward();
  delay(5000);
  motor2.stop_motor();
  delay(2000);
  motor1.reverse();
  delay(5000);
  motor2.stop_motor();
  delay(2000);
}
