/*
  Copyright (c) 2016, Juan Jimeno
  
*/

#include "Arduino.h"
#include "Kinematics.h"

Kinematics::Kinematics(base robot_base, int motor_max_rpm, float wheel_diameter, 
float wheels_x_distance, float wheels_y_distance):
    base_platform(robot_base),
    max_rpm_(motor_max_rpm),
    wheels_x_distance_(base_platform == DIFFERENTIAL_DRIVE ? 0 : wheels_x_distance),
    wheels_y_distance_(wheels_y_distance),
    wheel_circumference_(PI * wheel_diameter)
{    
}


Kinematics::rpm Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
    float tangential_vel;
	int min_sec = 60;
    float x_rpm;
    float y_rpm;
    float tan_rpm;
	//int total_wheels_;
	
    Kinematics::rpm rpm;
   
    switch (base_platform)
	{

		case DIFFERENTIAL_DRIVE:
		    linear_y = 0.0;
			total_wheels_ = 2;
		break ;
		
		case SKID_STEER:
			linear_y = 0.0;
			total_wheels_ = 4;
		break;
		
		case  ACKERMANN:
			linear_y = 0.0;
			angular_z = 0.0;
			total_wheels_ = 2;
		break;
		
		case ACKERMANN1:
		    linear_y = 0.0;
			angular_z = 0.0;
			total_wheels_ = 2;
		break;
		
		case  MECANUM:
		     total_wheels_ = 4 ;
		break;
	}
	
	//rpm = calculateRPM(linear_x, linear_y, angular_z);	
	//convert m/s to m/min
    //convert rad/s to rad/min
	
    x_rpm = linear_x * min_sec/ wheel_circumference_;
    y_rpm = linear_y * min_sec/ wheel_circumference_;
    tan_rpm = angular_z * min_sec * ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2))/ wheel_circumference_;
    //calculate for the target motor RPM and direction
    //front-left motor
    rpm.motor1 = x_rpm - y_rpm - tan_rpm;
    rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);

    //front-right motor
    rpm.motor2 = x_rpm + y_rpm + tan_rpm;
    rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);

	return rpm ;
}


Kinematics::velocities Kinematics::getVelocities(float steering_angle, int rpm1, int rpm2)
{
    Kinematics::velocities vel;
    float average_rps_x;

    //convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(rpm1 + rpm2) / total_wheels_) / 60; // RPM
    vel.linear_x = average_rps_x * wheel_circumference_; // m/s

    vel.linear_y = 0.0;

    //http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots
    vel.angular_z =  (vel.linear_x * tan(steering_angle)) / wheels_x_distance_;

    return vel;
}

Kinematics::velocities Kinematics::getVelocities(int rpm1, int rpm2)
{
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    //convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(rpm1 + rpm2 ) / total_wheels_) / 60; // RPM
    vel.linear_x = average_rps_x * wheel_circumference_; // m/s

    //convert average revolutions per minute in y axis to revolutions per second
    average_rps_y = ((float)(-rpm1 + rpm2 ) / total_wheels_) / 60; // RPM
    if(base_platform == MECANUM)
        vel.linear_y = average_rps_y * wheel_circumference_; // m/s
    else
        vel.linear_y = 0;

    //convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(-rpm1 + rpm2 ) / total_wheels_) / 60;
    vel.angular_z =  (average_rps_a * wheel_circumference_) / ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2)); //  rad/s

    return vel;
}

