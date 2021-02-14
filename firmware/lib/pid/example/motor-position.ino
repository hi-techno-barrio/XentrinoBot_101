#include <PID.h>

#define ENCODER_A 2
#define ENCODER_B 3

#define MOTOR_DIR 12
#define MOTOR_PWM 11
#define MOTOR_MAX_PWM 255
#define MOTOR_MIN_PWM 0

#define PIN_ENABLE 4

#define PID_P 2
#define PID_I 0.1
#define PID_D 0.1

#define PID_DELAY 50

double targetAngle = 360;

int encoderPos = 0;
unsigned long lastTime;
unsigned long nextPidRUn;

PID pid (PID_P, PID_I, PID_D);

void setup() {
  pinMode (ENCODER_A,INPUT);
  pinMode (ENCODER_B,INPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  Serial.begin (115200);

  lastTime = 0;
  nextPidRUn = 0;

  pid.setLimits(-MOTOR_MAX_PWM, MOTOR_MAX_PWM);
}

void loop() {
  computeEncoderPos();

  unsigned long now = millis();

  if(now>=nextPidRUn){
    nextPidRUn += PID_DELAY;
    double dtS=(now-lastTime)/1000.0;
    double angle = getAngle();
    double speed = pid.compute(angle, targetAngle, dtS);
    applySpeed(speed);
    Serial.print("\tencoderPos=");
    Serial.print(encoderPos);
    Serial.print("\tangle=");
    Serial.print((int)angle);
    Serial.print("\t speed=");
    Serial.println(speed);

    lastTime = now;
  }

}

double getAngle(){
  return ((double)encoderPos)/158.0*360.0;
}

void applySpeed(double speed){
  if(digitalRead(PIN_ENABLE)==HIGH){
    bool forward = speed>0;
    digitalWrite(MOTOR_DIR, forward ? HIGH : LOW);
    int pwm = (int)abs(speed);
    if(pwm<MOTOR_MIN_PWM){
      pwm = MOTOR_MIN_PWM;
    }
    if(forward){
      pwm = 255-pwm;
    }
    analogWrite(MOTOR_PWM, pwm);
  }else{
    digitalWrite(MOTOR_DIR, LOW);
    digitalWrite(MOTOR_PWM, LOW);
  }
}


int encoder0PinALast = LOW;
int n = LOW;
void computeEncoderPos(){
    n = digitalRead(ENCODER_A);
    if ((encoder0PinALast == LOW) && (n == HIGH)) {
      if (digitalRead(ENCODER_B) == LOW) {
        encoderPos--;
      } else {
        encoderPos++;
      }
    }
    encoder0PinALast = n;
}
