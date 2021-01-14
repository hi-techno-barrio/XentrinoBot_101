#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
 
//initializing all the variables
#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

double speed_cmd_left2 = 0;      

// #define encoder

const int PIN_SIDE_LIGHT_LED = 46;                  //Side light blinking led pin

unsigned long lastMilli = 0;

#define motor 


#define PID

  
ros::NodeHandle nh;

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  
  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

//__________________________________________________________________________

void setup() {

  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
 
   //initialize motor
   
  //setting motor speeds to zero
 
  //setting PID parameters
  // Define the rotary encoder for left motor
   //attachInterrupt(0, encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  //attachInterrupt(1, encoderRightMotor, RISING);
}

//_________________________________________________________________________

void loop() {
  nh.spinOnce();
 // algorithm 1
 if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loop
    lastMilli = millis();
    
    if (!nh.connected()){
    buzzer  
        }
     }
    
  else{
    // alarm
     }
    
  //algo 1  detect disturbance   
    if (abs(pos_left) < 5){                                                   //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
 
    else {
     // get motor speed 1
       speed_act_left=((pos_left/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate speed of left wheel
    }
    
 // algo 1 detect disturbance
    if (abs(pos_right) < 5){                                                  //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
    // get motor speed 2
    //speed_act_right=((pos_right/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;          // calculate speed of right wheel
    }
    
    pos_left = 0;
    pos_right = 0;
     //algo 3 invoke speed boundaries
     speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
     PID_leftMotor.Compute();                                                 
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); //
    
    // algo 2 check idle command  
   if ( (noCommLoops >= noCommLoopMax) || (speed_req_left == 0) )  {     //Stopping if too much time without command
                          //Stopping
      leftMotor->setSpeed(0);
      leftMotor->run(BRAKE);
    }
    else {
     motor.spin() // if actual pwm is positive move forward else move backward
    }
     /*if (PWM_leftMotor > 0){                          //Going forward
      leftMotor->setSpeed(abs(PWM_leftMotor));
      leftMotor->run(BACKWARD);
    }
    else {                                               //Going backward
      leftMotor->setSpeed(abs(PWM_leftMotor));
      leftMotor->run(FORWARD);
    }
    */
    
    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);    
    PID_rightMotor.Compute();                                                 
    // compute PWM value for right motor. Check constant definition comments for more information.
    PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255); // 

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      rightMotor->setSpeed(0);
      rightMotor->run(BRAKE);
    }
    else if (speed_req_right == 0){                       //Stopping
      rightMotor->setSpeed(0);
      rightMotor->run(BRAKE);
    }
    else if (PWM_rightMotor > 0){                         //Going forward
      rightMotor->setSpeed(abs(PWM_rightMotor));
      rightMotor->run(FORWARD);
    }
    else {                                                //Going backward
      rightMotor->setSpeed(abs(PWM_rightMotor));
      rightMotor->run(BACKWARD);
    }

    if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      Serial.println(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }
    
    publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
  }
 }

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}

//Left motor encoder counter
void encoderLeftMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left++;
  else pos_left--;
}

//Right motor encoder counter
void encoderRightMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_right--;
  else pos_right++;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
