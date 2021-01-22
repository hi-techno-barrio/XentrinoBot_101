/*
Christopher Coballes
Hi-Techno Barrio,Philippines
*/

#define MOTOR_DRIVER L298N

int pwm_pin1 9 ;
int pwm_pin2 2 ;

int pinM1_1  10 ;
int pinM1_2  12 ;
int pinM2_1  8  ;
int pinM2_2  7  ;


#include <Motor.h>
Motor motor1(Motor::MOTOR_DRIVER, 9,10,1);
Motor motor2(Motor::MOTOR_DRIVER, 9,10,1);

void setup() 
{
  motor1.stop();
  motor2.stop();
}

void loop() 
{
  motor1.forward();
  delay(2000);
  }
