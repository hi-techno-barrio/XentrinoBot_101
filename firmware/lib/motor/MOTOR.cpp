/* DC motor library for Arduino
coded by  Christopher Coballes Philippines  
ver 1.0 - 1/1/2020
*/
#include "Arduino.h"
#include "Motor.h"

Motor::Motor( board controller, int pin1,int pin2 ,int pwm_pin):   // this is a constructor
     var_controller_(controller),
     motor_pin1_(pin1),
     motor_pin2_(pin2),
     pwm_pin_(pwm_pin)  
 
{
    switch (controller)
    {
        case L298N:
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pin1_, OUTPUT);
            pinMode(motor_pin2_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));

            break;

      case MOTO:
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pin1_, OUTPUT);
            pinMode(motor_pin2_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));
            break;

        case BTS7960:
            pinMode(motor_pin1_, OUTPUT);
            pinMode(motor_pin2_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
            analogWrite(motor_pin1_, 0);
            analogWrite(motor_pin2_, 0);
            break;


    }
}

/////////////////// this function will start rotating motor forward ///////////////////////////////////////////////////                             
 void Motor::forward()                        
  {
      digitalWrite(motor_pin1_,HIGH);
      digitalWrite(motor_pin2_,LOW);                                               
  }
/////////////////////////////// // this function will start rotating motor reverse ////////////////////////////////////  
void Motor::reverse()                        
  {
      digitalWrite(motor_pin1_,LOW);
      digitalWrite(motor_pin2_,HIGH);                                               
  }

////////////////////////////////// // this function will stop the motor //////////////////////////////////////////////  
void Motor::stop()                        
  {
      digitalWrite(motor_pin1_,LOW);
      digitalWrite(motor_pin2_,LOW);                                               
  }   

//////////////////////////////////// // this function will set motor to spin /////////////////  
void Motor::spin(int pwm)
{
    switch (var_controller_)
    {
        case L298N:
            if(pwm > 0)
            {
                digitalWrite(motor_pin1_, HIGH);
                digitalWrite(motor_pin2_, LOW);
            }
            else if(pwm < 0)
            {
                digitalWrite(motor_pin1_, LOW);
                digitalWrite(motor_pin2_, HIGH);
            }
            analogWrite(pwm_pin_, abs(pwm));

            break;

  case MOTO:
            if(pwm > 0)
            {
                digitalWrite(motor_pin1_, HIGH);
                digitalWrite(motor_pin2_, LOW);
            }
            else if(pwm < 0)
            {
                digitalWrite(motor_pin1_, LOW);
                digitalWrite(motor_pin2_, HIGH);
            }
            analogWrite(pwm_pin_, abs(pwm));

            break;

        case BTS7960:
            if (pwm > 0)
            {
                analogWrite(motor_pin1_, 0);
                analogWrite(motor_pin2_, abs(pwm));
            }
            else if (pwm < 0)
            {
                analogWrite(motor_pin1_, 0);
                analogWrite(motor_pin2_, abs(pwm));
            }
            else
            {
                analogWrite(motor_pin1_, 0);
                analogWrite(motor_pin2_, 0);
            }

            break;
        
     }
}        
