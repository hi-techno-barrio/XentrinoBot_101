/* DC motor library for Arduino
coded by  Christopher Coballes Philippines  
ver 1.0 - 1/1/2020
*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
#include <analogWrite.h>

class Motor
  { 
                 
    public:

    enum board  {L298N,MOTO,BTS7960 };
			Motor(board controller, int pin1,int pin2,int pwm_pin );
			void forward(void);
			void reverse(void);
			void set_speed(int speed);  
			void stop(void);
			void spin(int pwm) ;

     private:

            board var_controller_;
            int motor_pin1_; 
            int motor_pin2_;
            int pwm_pin_ ;
  };
  
#endif
