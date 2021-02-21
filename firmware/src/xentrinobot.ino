// Author: Christopher M Coballes
// Credits:
//   Linorobot
//   ROS
//   THe Construct
//   
//ROS headers
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <Wire.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "xentrino_config.h"
#include "Imu.h"
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"
#include "Imu.h"

#define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards
#include "Encoder.h"

#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5

Encoder encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV); 

Motor motor1(Motor::CONTROLLER MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2(Motor::CONTROLLER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); 

PID   motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID   motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float req_linear_vel_x = 0;
float req_linear_vel_y = 0;
float req_angular_vel_z = 0;

unsigned long callback_time = 0;

//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
sensor_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
geometry_msgs::Vector3 raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void setup()
{
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("XENTRINOBOT CONNECTED");
    delay(1);
}

void loop()
{
    static unsigned long control_time = 0;
    static unsigned long debug_time = 0;
    static bool imu_is_initialized;
 
    //this block stops the motor when no command is received
   if ((millis() - callback_time) >=(1000 / COMMAND_RATE))
    {
        stopBase();
    } 
//this block drives the robot based on defined rate	
  if ((millis() - control_time) >= (1000 / COMMAND_RATE))
	{
	 moveBase();	
	 //sanity check if the IMU is connected
        if (!imu_is_initialized)
        {
            imu_is_initialized = initIMU();

            if(imu_is_initialized)
                nh.loginfo("IMU Initialized");
            else
                nh.logfatal("IMU failed to initialize. Check your IMU connection.");
        }
        else
        {
            publishIMU();
        }
        control_time = millis();
	}
	
	
    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(DEBUG)
    {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
            printDebug();
            prev_debug_time = millis();
        }
    }
    //call all the callbacks waiting to be called
    nh.spinOnce();
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    req_linear_vel_x = cmd_msg.linear.x;
    req_linear_vel_y = cmd_msg.linear.y;
    req_angular_vel_z = cmd_msg.angular.z;

    callback_time = millis();
}

void moveBase()
{
    //get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(req_linear_vel_x, req_linear_vel_y, req_angular_vel_z);

    //get the current speed of each motor
    int current_rpm1 = encoder1.getRPM();
    int current_rpm2 = encoder2.getRPM();
  
    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
	//need to auto-tune via firmaware
    Kinematics::velocities current_vel;
	//for 2WD only!
    current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
  
    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase()
{
    req_linear_vel_x = 0;
    req_linear_vel_y = 0;
    req_angular_vel_z = 0;
}

void publishIMU()
{
    //pass accelerometer data to imu object
    raw_imu_msg.linear_acceleration = readAccelerometer();

    //pass gyroscope data to imu object
    raw_imu_msg.angular_velocity = readGyroscope();

    //pass accelerometer data to imu object
    raw_imu_msg.magnetic_field = readMagnetometer();

    //publish raw_imu_msg
    raw_imu_pub.publish(&raw_imu_msg);
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void printDebug()
{
    char buffer[50];

    sprintf (buffer, "Encoder FrontLeft  : %ld", motor1_encoder.read());
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder FrontRight : %ld", motor2_encoder.read());
    nh.loginfo(buffer);
}

