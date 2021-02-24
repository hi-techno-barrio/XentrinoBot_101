

#ifndef _IMU_H_
#define _IMU_H_
#include <Kalman.h>             // Source: https://github.com/TKJElectronics/KalmanFilter
#include "I2CESP32.h"

#include <Wire.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "MPU6050.h"
#include "fake_mag.h"

#define G_TO_ACCEL 9.81
#define MGAUSS_TO_UTESLA 0.1
#define UTESLA_TO_TESLA 0.000001
#define ACCEL_SCALE 1 / 16384    // LSB/g
#define GYRO_SCALE 1 / 131       // LSB/(deg/s)
#define MAG_SCALE 0.3            // uT/LSB
    
MPU6050 accelerometer;
MPU6050 gyroscope;    
FakeMag magnetometer;
MPU6050 orientation; 
	                              // 1688 factory default for my test chip

bool initIMU()
{
    Wire.begin();
    bool ret;

    accelerometer.initialize();
    ret = accelerometer.testConnection();
    if(!ret)
        return false;

    gyroscope.initialize();
    ret = gyroscope.testConnection();
    if(!ret)
        return false;
  
    magnetometer.initialize();
    ret = magnetometer.testConnection();
    if(!ret)
        return false;
    
    return true;
}

geometry_msgs::Vector3 readAccelerometer()
{
    geometry_msgs::Vector3 accel;
    int16_t ax, ay, az;
    
    accelerometer.getAcceleration(&ax, &ay, &az);

    accel.x = ax * (double) ACCEL_SCALE * G_TO_ACCEL;
    accel.y = ay * (double) ACCEL_SCALE * G_TO_ACCEL;
    accel.z = az * (double) ACCEL_SCALE * G_TO_ACCEL;

    return accel;
}

geometry_msgs::Vector3 readGyroscope()
{
    geometry_msgs::Vector3 gyro;
    int16_t gx, gy, gz;

    gyroscope.getRotation(&gx, &gy, &gz);

    gyro.x = gx * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.y = gy * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.z = gz * (double) GYRO_SCALE * DEG_TO_RAD;

    return gyro;
}

geometry_msgs::Vector3 readMagnetometer()
{
    geometry_msgs::Vector3 mag;
    int16_t mx, my, mz;

    magnetometer.getHeading(&mx, &my, &mz);

    mag.x = mx * (double) MAG_SCALE * UTESLA_TO_TESLA;
    mag.y = my * (double) MAG_SCALE * UTESLA_TO_TESLA;
    mag.z = mz * (double) MAG_SCALE * UTESLA_TO_TESLA;

    return mag;
}

geometry_msgs::Quaternion readOrientation()
{
  geometry_msgs::Quaternion  orient ;
  
  Kalman kalmanX; // Create the Kalman instances
  Kalman kalmanY;
  uint32_t timer;
  
   /* IMU Data */
   double accX, accY, accZ;
   double gyroX, gyroY, gyroZ;
   double gyroXangle, gyroYangle; // Angle calculate using the gyro only
   //double compAngleX, compAngleY; // Calculated angle using a complementary filter
   double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
   double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
   
   //pass accelerometer data to imu object
   accX = orientation.getAccelerationX();
   accY = orientation.getAccelerationY();
   accZ = orientation.getAccelerationX();
   gyroX = orientation.getRotationX();
   gyroY = orientation.getRotationY();
   gyroZ = orientation.getRotationZ();
 
   timer = micros();

   // restricted pitch for 90 degress else robot will be stumbled :)
   double roll  = atan2(accY, accZ) * RAD_TO_DEG;
   double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
   double gyroXrate = gyroX / 131.0; // Convert to deg/s
   double gyroYrate = gyroY / 131.0; // Convert to deg/s
   
// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    //compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
   kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
   
    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
 
  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
     gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
     gyroYangle = kalAngleY;
   
   kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
   kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  // orientation.dmpgetQuaternion(&ox,&oy,&oz,&ow);	
	orient.x = kalAngleY;
	orient.y = kalAngleY;
	orient.z = -1;
	orient.w = -1 ;
	return orient; 
}

#endif

/*

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
              msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
    ROS_INFO( "Mag. Field: %.3f,%.3f,%.3f [uT]",
              msg->magnetic_field.x*1e-6, msg->magnetic_field.y*1e-6, msg->magnetic_field.z*1e-6);
}

*/

//ADXL345 https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
//HMC8553L https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
//ITG320 https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf


//MPU9150 https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
//MPU9250 https://www.invensense.com/wp-content/uploads/2015/02/MPU-9150-Datasheet.pdf
//MPU6050 https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf

//http://www.sureshjoshi.com/embedded/invensense-imus-what-to-know/
//https://stackoverflow.com/questions/19161872/meaning-of-lsb-unit-and-unit-lsb