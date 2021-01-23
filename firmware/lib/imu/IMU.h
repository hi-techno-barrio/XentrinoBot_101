#ifndef _IMU2_H_
#define _IMU2_H_

#include "I2Cdev.h"
// #include "imu_config.h"
#include "imu.h"

#define G_TO_ACCEL 9.81
#define MGAUSS_TO_UTESLA 0.1
#define UTESLA_TO_TESLA 0.000001

#include <Wire.h>
#include "geometry_msgs/Vector3.h"

#ifdef USE_MPU6050_IMU
    #include "MPU6050.h"
    #include "fake_mag.h"

    #define ACCEL_SCALE 1 / 16384 // LSB/g
    #define GYRO_SCALE 1 / 131 // LSB/(deg/s)
    #define MAG_SCALE 0.3 // uT/LSB
    
    MPU6050 accelerometer;
    MPU6050 gyroscope;    
    FakeMag magnetometer;
#endif

#ifdef USE_MPU9150_IMU
    #include "MPU9150.h"

    #define ACCEL_SCALE 1 / 16384 // LSB/g
    #define GYRO_SCALE 1 / 131 // LSB/(deg/s)
    #define MAG_SCALE 0.3 // uT/LSB
    
    MPU9150 accelerometer;
    MPU9150 gyroscope;    
    MPU9150 magnetometer;
#endif

#ifdef USE_MPU9250_IMU
    #include "MPU9250.h"

    #define ACCEL_SCALE 1 / 16384 // LSB/g
    #define GYRO_SCALE 1 / 131 // LSB/(deg/s)
    #define MAG_SCALE 0.6 // uT/LSB
    
    MPU9250 accelerometer;
    MPU9250 gyroscope;    
    MPU9250 magnetometer;
#endif

#ifdef USE_GY85_IMU
    #include "ADXL345.h"
    #include "ITG3200.h"
    #include "HMC5883L.h"

    #define ACCEL_SCALE 1 / 256 // LSB/g
    #define GYRO_SCALE 1 / 14.375 // LSB/(deg/s)
    #define MAG_SCALE 0.92 * MGAUSS_TO_UTESLA // uT/LSB

    ADXL345 accelerometer;
    ITG3200 gyroscope;
    HMC5883L magnetometer;
#endif

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

#endif



//ADXL345 https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
//HMC8553L https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
//ITG320 https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf


//MPU9150 https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
//MPU9250 https://www.invensense.com/wp-content/uploads/2015/02/MPU-9150-Datasheet.pdf
//MPU6050 https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf

//http://www.sureshjoshi.com/embedded/invensense-imus-what-to-know/
//https://stackoverflow.com/questions/19161872/meaning-of-lsb-unit-and-unit-lsb
