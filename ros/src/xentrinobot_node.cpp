#include <ros/ros.h>
#include <geometry_msgs/Accel.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/Imu.h>
#include <math.h>

double vel_x_ = 0.0;
double vel_y_ = 0.0;
double vel_z_ = 0.0;

double vel_dt_ = 0.0;
double imu_dt_ = 0.0;
double imu_z_ = 0.0;
double rate = 10.0;
 
ros::Time last_vel_time(0.0);
ros::Time last_imu_time(0.0);

void velCallback( const geometry_msgs::Accel& vel) {
    //callback every time the robot's linear velocity is received
    ros::Time current_time = ros::Time::now();
	
    linear_vel_x_ = vel.linear_x;
    linear_vel_y_ = vel.linear_y;
    angular_vel_z_ = vel.angular_z;

    vel_dt_ = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;
}

void IMUCallback( const sensor_msgs::Imu& imu){
    //callback every time the robot's angular velocity is received
    ros::Time current_time = ros::Time::now();
    //this block is to filter out imu noise
    if(imu.angular_velocity.z > -0.03 && imu.angular_velocity.z < 0.03)
    {
        angular_imu_z = 0.00;
    }
    else
    {
        angular_imu_z = imu.angular_velocity.z;
    }
    imu_dt = (current_time - last_imu_time).toSec();
    last_imu_time = current_time;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "xentrinobot_base_node");
    ros::NodeHandle nh_;
   
    ros::Subscriber imu_subscriber_;
    imu_subscriber_	= nh_.subscribe("imu/data", 50, IMUCallback);
	
    ros::Publisher odom_publisher_;
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("raw_odom", 50);
	
    ros::Subscriber velocity_subscriber_;
    velocity_subscriber_ = nh_.subscribe("raw_vel", 50, velCallback);
	
    tf2::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;

   
    double x_pos = 0.0;
    double y_pos = 0.0;
    double turn_angle_ = 0.0;

    ros::Rate r(rate);
    while(n.ok()){
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();

        //linear velocity is the linear velocity published from the Teensy board in x axis
        double linear_velocity_x = vel_x_;

        //linear velocity is the linear velocity published from the Teensy board in y axis
        double linear_velocity_y = vel_y_;

        //angular velocity is the rotation in Z from imu_filter_madgwick's output
        //double angular_velocity = g_imu_z;
	double angular_velocity = vel_z_;

        //calculate angular displacement  θ = ω * t
        double turn_angle_theta = angular_velocity * vel_dt_; //radians
        double delta_x = (linear_velocity_x * cos(theta) - linear_velocity_y * sin(theta)) *  vel_dt_; //m
        double delta_y = (linear_velocity_x * sin(theta) + linear_velocity_y * cos(theta)) *  vel_dt_; //m

        //calculate current position of the robot
        x_pos += delta_x;
        y_pos += delta_y;
        turn_angle_ += turn_angle_theta;

        //calculate robot's heading in quarternion angle
        //ROS has a function to calculate yaw in quaternion angle
        //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        odom_quat.setRPY(0,0,turn_angle_);
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";
	    
	//robot's position in x,y, and z
	odom_trans.transform.translation.x = x_pos_;
	odom_trans.transform.translation.y = y_pos_;
	odom_trans.transform.translation.z = 0.0;
		
	//robot's heading in quaternion
	odom_trans.transform.rotation.x = odom_quat.x();
	odom_trans.transform.rotation.y = odom_quat.y();
	odom_trans.transform.rotation.z = odom_quat.z();
	odom_trans.transform.rotation.w = odom_quat.w();
	odom_trans.header.stamp = current_time;
	    
	//publish robot's tf using odom_trans object
	//odom_broadcaster_.sendTransform(odom_trans);
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_footprint";

	//robot's position in x,y, and z
	odom.pose.pose.position.x = x_pos_;
	odom.pose.pose.position.y = y_pos_;
	odom.pose.pose.position.z = 0.0;
	    
	//robot's heading in quaternion
	odom.pose.pose.orientation.x = odom_quat.x();
	odom.pose.pose.orientation.y = odom_quat.y();
	odom.pose.pose.orientation.z = odom_quat.z();
	odom.pose.pose.orientation.w = odom_quat.w();
	odom.pose.covariance[0] = 0.001;
	odom.pose.covariance[7] = 0.001;
	odom.pose.covariance[35] = 0.001;

	//linear speed from encoders
	odom.twist.twist.linear.x = linear_velocity_x_;
	odom.twist.twist.linear.y = linear_velocity_y_;
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	    
	//angular speed from encoders
	odom.twist.twist.angular.z = angular_velocity_z_;
	odom.twist.covariance[0] = 0.0001;
	odom.twist.covariance[7] = 0.0001;
	odom.twist.covariance[35] = 0.0001;

	odom_publisher_.publish(odom);

	r.sleep();
      }
}
