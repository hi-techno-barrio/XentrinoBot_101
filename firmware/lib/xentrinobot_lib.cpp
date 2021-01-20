#include "Arduino.h"
#include "xentrino_base_config.h"
#include "xentrino.h"

/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/

Kinematics::Kinematics(  int max_rpm, float wheel_diameter, float wheels_x_distance, float wheels_y_distance) 
  {   
      wheel_circumference = PI* wheel_diameter;
	  robot_base = XENTRINO_BASE;	 
  }
	Kinematics::rpm Kinematics::expected_RPM(float linear_x, float linear_y, float angular_z)
		{			
			float tangential_vel;
			float x_rpm;
			float y_rpm;
			float tan_rpm;	
			
         //    case DIFFERENTIAL_DRIVE:    
			total_wheels = 2;
			linear_y=0.0;
				
			Kinematics::rpm rpm;
			// float linear_y is zero  and convert m/s to m/min			
			tangential_vel = angular_z * 60 * ((wheels_x_distance / 2) + (wheels_y_distance/ 2));
			
			x_rpm = linear_x * 60/ wheel_circumference;	
			y_rpm = linear_y * 60 / wheel_circumference;
			tan_rpm =  tangential_vel / wheel_circumference;
			
			//calculate for the target motor RPM and direction-front-left motor
			//front-left motor
			rpm.motor1 = x_rpm - y_rpm - tan_rpm;
			rpm.motor1 = constrain(rpm.motor1, -max_rpm, max_rpm);

			//front-right motor
			rpm.motor2 = x_rpm + y_rpm + tan_rpm;
			rpm.motor2 = constrain(rpm.motor2, -max_rpm, max_rpm);

			return rpm;			
		}


		Kinematics::velocities Kinematics::getVelocities(int rpm1, int rpm2)
		{
			Kinematics::velocities vel;
			float average_rps_x;
			float average_rps_y;
			float average_rps_a;

			//convert average revolutions per minute to revolutions per second
			average_rps_x = ((float)(rpm1 + rpm2) / total_wheels) / 60; // RPM
			vel.linear_x = average_rps_x * wheel_circumference; // m/s

			//convert average revolutions per minute in y axis to revolutions per second
			average_rps_y = ((float)(-rpm1 + rpm2 ) / total_wheels) / 60; // RPM
		
			vel.linear_y = 0;

			//convert average revolutions per minute to revolutions per second
			average_rps_a = ((float)(-rpm1 + rpm2) / total_wheels) / 60;
			vel.angular_z =  (average_rps_a * wheel_circumference) / ((wheels_x_distance / 2) + (wheels_y_distance / 2)); //  rad/s

    return vel;
		}
		
/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/
Motor::Motor(driver motor_driver, int pwm_pin, int motor_pinA, int motor_pinB)  
{
	motor_driver_ = motor_driver;
    pwm_pin_      = pwm_pin ;
    motor_pinA_   = motor_pinA ;
    motor_pinB_   = motor_pinB ;
	
    switch (motor_driver)
    {
         case L298P:
		    enableMOTOR( motor_driver_); 
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);
            
            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));
			break;
			
		case MOTO:
		    enableMOTOR( motor_driver_); 
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);
            
            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));
			break;
			
        case BIG_MOTO:
		    enableMOTOR( motor_driver_);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
            analogWrite(motor_pinB_, 0);
            analogWrite(motor_pinA_, 0);
            break;

    }
}


/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/
void Motor::spin(int pwm)
{
    switch (motor_driver_)
    {
		
		case L298P:
            if(pwm > 0)
            {
                digitalWrite(motor_pinA_, HIGH);
                digitalWrite(motor_pinB_, LOW);
            }
            else if(pwm < 0)
            {
                digitalWrite(motor_pinA_, LOW);
                digitalWrite(motor_pinB_, HIGH);
            }
            analogWrite(pwm_pin_, abs(pwm));

            break;
			
        case MOTO:
            if(pwm > 0)
            {
                digitalWrite(motor_pinA_, HIGH);
                digitalWrite(motor_pinB_, LOW);
            }
            else if(pwm < 0)
            {
                digitalWrite(motor_pinA_, LOW);
                digitalWrite(motor_pinB_, HIGH);
            }
            analogWrite(pwm_pin_, abs(pwm));

            break;
				
        case BIG_MOTO:
            if (pwm > 0)
            {
                analogWrite(motor_pinA_, 0);
                analogWrite(motor_pinB_, abs(pwm));
            }
            else if (pwm < 0)
            {
                analogWrite(motor_pinB_, 0);
                analogWrite(motor_pinA_, abs(pwm));
            }
            else
            {
                analogWrite(motor_pinB_, 0);
                analogWrite(motor_pinA_, 0);
            }

            break;
        
   
    }
}
/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/
void  Controller::testMotor(int pwm)
{
	 analogWrite(pwm_pin_, abs(pwm));
}

/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/
void Controller::enableMOTOR(driver motor_driver)
{
    switch(motor_driver)
    {
        case L298N:   // Teensy 2WD
		pinMode(MOTOR1_MOTO_EN1,OUTPUT);
		pinMode(MOTOR2_MOTO_EN2,OUTPUT);
		digitalWrite(MOTOR1_MOTO_EN1, HIGH);
        digitalWrite(MOTOR2_MOTO_EN2, HIGH); 
		break;
		
		case MOTO:  // Teensy 4WD
		pinMode(MOTOR2_MOTO_EN1,OUTPUT);
		pinMode(MOTOR4_MOTO_EN2,OUTPUT);
		
        digitalWrite(MOTOR2_MOTO_EN2, HIGH); 
        digitalWrite(MOTOR4_MOTO_EN4, HIGH); 
        break;
		
	   case BIG_MOTO:  // MEGA  4WD
		pinMode(MOTOR1_MOTO_EN1,OUTPUT);
		pinMode(MOTOR2_MOTO_EN2,OUTPUT);
		
		digitalWrite(MOTOR1_MOTO_EN1, HIGH);
        digitalWrite(MOTOR2_MOTO_EN2, HIGH); 
		 
        break;
		
	   case BIG_MOTO:             
		break;                  
    }
	
}  
/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/
PID::PID(float min_val, float max_val, float kp, float ki, float kd)
 
{
	min_val_ = min_val;
    max_val_ = max_val;
    kp_      = kp ;
    ki_      = ki ;
    kd_      = kd ;
}

double PID::compute(float setpoint, float measured_value)
{
    double error;
    double pid;

    //setpoint is constrained between min and max to prevent pid from having too much error
    error = setpoint - measured_value;
    integral_ += error;
    derivative_ = error - prev_error_;

    if(setpoint == 0 && error == 0)
    {
        integral_ = 0;
    }

    pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
    prev_error_ = error;

    return constrain(pid, min_val_, max_val_);
}

void PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/
Velocity::Velocity(int counts_per_rev )
{
	//ticks_ = ticks ;
	counts_per_rev_ = counts_per_rev ;
}
int Velocity::getRPM (int ticks)
{
		long encoder_ticks = ticks ;
		//this function calculates the motor's RPM based on encoder ticks and delta time
		unsigned long current_time = millis();
		unsigned long dt = current_time - prev_update_time_;

		//convert the time from milliseconds to minutes
		double dtm = (double)dt / 60000;
		double delta_ticks = encoder_ticks - prev_encoder_ticks_;

		//calculate wheel's speed (RPM)

		prev_update_time_ = current_time;
		prev_encoder_ticks_ = encoder_ticks;
		
		return (delta_ticks / counts_per_rev_) / dtm;		
}

/* ------------------------------------------------------------------------------------------------------------- */
/*     																											 */
/*     																											 */
/*     																											 */
/*---------------------------------------------------------------------------------------------------------------*/
Kalman::Kalman(float N_angle,float N_bias, float N_measure, float setAngle,float setBias) {
    /* We will set the variables like so, these can also be tuned by the user */
    Q_angle = N_angle;    //   0.001f;
    Q_bias = N_bias;     //   0.003f;
    R_measure = N_measure;  //   0.03f;
    
    angle = setAngle ;  //   0.0f; // Reset the angle
    bias = setBias ;    //   0.0f; // Reset bias

    P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman::getAngle(float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};
/* These are used to tune the Kalman filter */
void Kalman::setFilterNoises( float Q_angle, float Q_bias, float R_measure)
{
	Kalman::tuner tuner1; 
tuner1.Q_angle = Q_angle;
tuner1.Q_bias = Q_bias;
tuner1.R_measure = R_measure ;
}

void Kalman::getFilterNoises()
{
	Kalman::tuner tuner1; 
Q_angle = tuner1.Q_angle;
Q_bias = tuner1.Q_bias ;
R_measure = tuner1.R_measure;
}


// IMU  Class

bool MPU6050::begin(mpu6050_dps_t scale, mpu6050_range_t range, int mpua)
{
    // Set Address
    mpuAddress = mpua;

    Wire.begin();

    // Reset calibrate values
    dg.XAxis = 0;
    dg.YAxis = 0;
    dg.ZAxis = 0;
    useCalibrate = false;

    // Reset threshold values
    tg.XAxis = 0;
    tg.YAxis = 0;
    tg.ZAxis = 0;
    actualThreshold = 0;

    // Check MPU6050 Who Am I Register
    if (fastRegister8(MPU6050_REG_WHO_AM_I) != 0x68)
    {
	return false;
    }

    // Set Clock Source
    setClockSource(MPU6050_CLOCK_PLL_XGYRO);

    // Set Scale & Range
    setScale(scale);
    setRange(range);

    // Disable Sleep Mode
    setSleepEnabled(false);

    return true;
}




Vector MPU6050::readRawAccel(void)
{
    Wire.beginTransmission(mpuAddress);
    #if ARDUINO >= 100
	Wire.write(MPU6050_REG_ACCEL_XOUT_H);
    #else
	Wire.send(MPU6050_REG_ACCEL_XOUT_H);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(mpuAddress);
    Wire.requestFrom(mpuAddress, 6);

    while (Wire.available() < 6);

    #if ARDUINO >= 100
	uint8_t xha = Wire.read();
	uint8_t xla = Wire.read();
        uint8_t yha = Wire.read();
	uint8_t yla = Wire.read();
	uint8_t zha = Wire.read();
	uint8_t zla = Wire.read();
    #else
	uint8_t xha = Wire.receive();
	uint8_t xla = Wire.receive();
	uint8_t yha = Wire.receive();
	uint8_t yla = Wire.receive();
	uint8_t zha = Wire.receive();
	uint8_t zla = Wire.receive();
    #endif

    ra.XAxis = xha << 8 | xla;
    ra.YAxis = yha << 8 | yla;
    ra.ZAxis = zha << 8 | zla;

    return ra;
}

Vector MPU6050::readNormalizeAccel(void)
{
    readRawAccel();

    na.XAxis = ra.XAxis * rangePerDigit * 9.80665f;
    na.YAxis = ra.YAxis * rangePerDigit * 9.80665f;
    na.ZAxis = ra.ZAxis * rangePerDigit * 9.80665f;

    return na;
}


Vector MPU6050::readRawGyro(void)
{
    Wire.beginTransmission(mpuAddress);
    #if ARDUINO >= 100
	Wire.write(MPU6050_REG_GYRO_XOUT_H);
    #else
	Wire.send(MPU6050_REG_GYRO_XOUT_H);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(mpuAddress);
    Wire.requestFrom(mpuAddress, 6);

    while (Wire.available() < 6);

    #if ARDUINO >= 100
	uint8_t xha = Wire.read();
	uint8_t xla = Wire.read();
        uint8_t yha = Wire.read();
	uint8_t yla = Wire.read();
	uint8_t zha = Wire.read();
	uint8_t zla = Wire.read();
    #else
	uint8_t xha = Wire.receive();
	uint8_t xla = Wire.receive();
	uint8_t yha = Wire.receive();
	uint8_t yla = Wire.receive();
	uint8_t zha = Wire.receive();
	uint8_t zla = Wire.receive();
    #endif

    rg.XAxis = xha << 8 | xla;
    rg.YAxis = yha << 8 | yla;
    rg.ZAxis = zha << 8 | zla;

    return rg;
}

Vector MPU6050::readNormalizeGyro(void)
{
    readRawGyro();

    if (useCalibrate)
    {
	ng.XAxis = (rg.XAxis - dg.XAxis) * dpsPerDigit;
	ng.YAxis = (rg.YAxis - dg.YAxis) * dpsPerDigit;
	ng.ZAxis = (rg.ZAxis - dg.ZAxis) * dpsPerDigit;
    } else
    {
	ng.XAxis = rg.XAxis * dpsPerDigit;
	ng.YAxis = rg.YAxis * dpsPerDigit;
	ng.ZAxis = rg.ZAxis * dpsPerDigit;
    }

    if (actualThreshold)
    {
	if (abs(ng.XAxis) < tg.XAxis) ng.XAxis = 0;
	if (abs(ng.YAxis) < tg.YAxis) ng.YAxis = 0;
	if (abs(ng.ZAxis) < tg.ZAxis) ng.ZAxis = 0;
    }

    return ng;
}



// Calibrate algorithm
void MPU6050::calibrateGyro(uint8_t samples)
{
    // Set calibrate
    useCalibrate = true;

    // Reset values
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    // Read n-samples
    for (uint8_t i = 0; i < samples; ++i)
    {
	readRawGyro();
	sumX += rg.XAxis;
	sumY += rg.YAxis;
	sumZ += rg.ZAxis;

	sigmaX += rg.XAxis * rg.XAxis;
	sigmaY += rg.YAxis * rg.YAxis;
	sigmaZ += rg.ZAxis * rg.ZAxis;

	delay(5);
    }

    // Calculate delta vectors
    dg.XAxis = sumX / samples;
    dg.YAxis = sumY / samples;
    dg.ZAxis = sumZ / samples;

    // Calculate threshold vectors
    th.XAxis = sqrt((sigmaX / 50) - (dg.XAxis * dg.XAxis));
    th.YAxis = sqrt((sigmaY / 50) - (dg.YAxis * dg.YAxis));
    th.ZAxis = sqrt((sigmaZ / 50) - (dg.ZAxis * dg.ZAxis));

    // If already set threshold, recalculate threshold vectors
    if (actualThreshold > 0)
    {
	setThreshold(actualThreshold);
    }
}


// Set treshold value
void MPU6050::setThreshold(uint8_t multiple)
{
    if (multiple > 0)
    {
	// If not calibrated, need calibrate
	if (!useCalibrate)
	{
	    calibrateGyro();
	}

	// Calculate threshold vectors
	tg.XAxis = th.XAxis * multiple;
	tg.YAxis = th.YAxis * multiple;
	tg.ZAxis = th.ZAxis * multiple;
    } else
    {
	// No threshold
	tg.XAxis = 0;
	tg.YAxis = 0;
	tg.ZAxis = 0;
    }

    // Remember old threshold value
    actualThreshold = multiple;
}



// Read 8-bit from register
uint8_t MPU6050::readRegister8(uint8_t reg)
{
    uint8_t value;

    Wire.beginTransmission(mpuAddress);
    #if ARDUINO >= 100
	Wire.write(reg);
    #else
	Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(mpuAddress);
    Wire.requestFrom(mpuAddress, 1);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
	value = Wire.read();
    #else
	value = Wire.receive();
    #endif;
    Wire.endTransmission();

    return value;
}

// Write 8-bit to register
void MPU6050::writeRegister8(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(mpuAddress);

    #if ARDUINO >= 100
	Wire.write(reg);
	Wire.write(value);
    #else
	Wire.send(reg);
	Wire.send(value);
    #endif
    Wire.endTransmission();
}

int16_t MPU6050::readRegister16(uint8_t reg)
{
    int16_t value;
    Wire.beginTransmission(mpuAddress);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(mpuAddress);
    Wire.requestFrom(mpuAddress, 2);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        uint8_t vha = Wire.read();
        uint8_t vla = Wire.read();
    #else
        uint8_t vha = Wire.receive();
        uint8_t vla = Wire.receive();
    #endif;
    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}

void MPU6050::writeRegister16(uint8_t reg, int16_t value)
{
    Wire.beginTransmission(mpuAddress);

    #if ARDUINO >= 100
	Wire.write(reg);
	Wire.write((uint8_t)(value >> 8));
	Wire.write((uint8_t)value);
    #else
	Wire.send(reg);
	Wire.send((uint8_t)(value >> 8));
	Wire.send((uint8_t)value);
    #endif
    Wire.endTransmission();
}

// Read register bit
bool MPU6050::readRegisterBit(uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = readRegister8(reg);
    return ((value >> pos) & 1);
}

// Write register bit
void MPU6050::writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = readRegister8(reg);

    if (state)
    {
        value |= (1 << pos);
    } else 
    {
        value &= ~(1 << pos);
    }

    writeRegister8(reg, value);
}

