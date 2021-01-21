#ifndef DC_Motor_h
#define DC_Motor_h

#include "Arduino.h"

class DC_Motor
  { 
    private:
            int pin_1,pin_2,motor_speed,dir_flag;
    public:
    DC_Motor(int pin1,int pin2);
    DC_Motor(int pin1,int pin2,int speed_flag);
    void forward(void);
    void forward_with_set_speed(void);
    void reverse(void);
    void reverse_with_set_speed(void);
    void run_motor(int dir,int speed);
    void set_speed(int speed);  
    void start_motor(int dir);
    void jogg_full_speed(int dir);
    void stop_motor(void);
    void dc_break(void); 
    void motor_speed_zero(void); 
    void soft_start(int dir,int speed,int time_int_sec);
    void smooth_stop(int time_int_sec);
    void jogg_set_speed(int dir,int speed);
  };
  
#endif
