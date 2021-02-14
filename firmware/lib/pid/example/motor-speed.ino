#include <PID.h>

#define ENCODER_A 2
#define ENCODER_B 3
#define ENCODER_TICKS 160

#define MOTOR_DIR 12
#define MOTOR_PWM 11
#define MOTOR_MAX_PWM 255
#define MOTOR_MIN_PWM 0

#define PIN_ENABLE 4

#define PID_P 0.1
#define PID_I 5
#define PID_D 0.00

#define PID_DELAY 50

double targetSpeed = 360.0; // degr/sec

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


  targetSpeed = digitalRead(PIN_ENABLE)==HIGH ? 600 : 0;

  if(now>=nextPidRUn){
    nextPidRUn += PID_DELAY;
    double dtS=((double)(now-lastTime))/1000.0;
    Serial.print("encoderPos=");
    Serial.print(encoderPos);
    double currentSpeed = (int)getAngularSpeed(dtS);
    double torque = pid.compute(currentSpeed, targetSpeed, dtS);
    //double torque = (targetSpeed-currentSpeed)*0.1;
    applySpeed(torque);
    Serial.print("\ttargetSpeed=");
    Serial.print((int)targetSpeed);
    Serial.print("\tcurrentSpeed=");
    Serial.print((int)currentSpeed);
    Serial.print("\t torque=");
    Serial.println(torque);

    lastTime = now;
  }

}

double getAngularSpeed(double dt){
  double diffA = ((double)encoderPos)/ENCODER_TICKS*360.0;
  encoderPos = 0;
  return diffA/dt;
}

void applySpeed(double speed){
  //if(digitalRead(PIN_ENABLE)==HIGH){
  if(true){
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
