
#include "ENCODER.h"

/*
  Encoder(int encA, int encB , int cpr, int index)
  - encA, encB    - encoder A and B pins
  - cpr           - counts per rotation number (cpm=ppm*4)
*/

Encoder::Encoder(Quadrature quadrature, int _encA, int _encB , float _ppr){
 
 // initiate quadrature
 var_quadrature_(quadrature);
 
 // default external pullup! 
  pullup = Pullup::EXTERN;
// Encoder measurement structure init
  // hardware pins
  pinA = _encA;
  pinB = _encB;
  // counter setup
  pulse_counter = 0;
  pulse_timestamp = 0;

  cpr = _ppr;
  A_active = 0;
  B_active = 0;
 
  // velocity calculation variables
  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = micros();
  
}


//  Encoder interrupt callback functions
// A channel
void Encoder::call_intA() {
  int A = digitalRead(pinA);
  switch (var_quadrature_){
    case Quadrature::ON:
      // CPR = 4xPPR
      if ( A != A_active ) {
        pulse_counter += (A_active == B_active) ? 1 : -1;
        pulse_timestamp = micros();
        A_active = A;
      }
      break;
    case Quadrature::OFF:
      // CPR = PPR
      if(A && !digitalRead(pinB)){
        pulse_counter++;
        pulse_timestamp = micros();
      }
      break;
  }
}
// B channel
void Encoder::call_intB() {
  int B = digitalRead(pinB);
  switch (var_quadrature_){
    case Quadrature::ON:
  //     // CPR = 4xPPR
      if ( B != B_active ) {
        pulse_counter += (A_active != B_active) ? 1 : -1;
        pulse_timestamp = micros();
        B_active = B;
      }
      break;
    case Quadrature::OFF:
      // CPR = PPR
      if(B && !digitalRead(pinA)){
        pulse_counter--;
        pulse_timestamp = micros();
      }
      break;
  }
}

/*
	Shaft angle calculation
*/
float Encoder::getAngle(){
  return  natural_direction * _2PI * (pulse_counter) / ((float)cpr);
}
/*
  Shaft velocity calculation
  function using mixed time and frequency measurement technique
*/

float Encoder::getVelocity(){
  // timestamp
  long timestamp_us = micros();
  // sampling time calculation
  float Ts = (timestamp_us - prev_timestamp_us) * 1e-6;
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3;

  // time from last impulse
  float Th = (timestamp_us - pulse_timestamp) * 1e-6;
  long dN = pulse_counter - prev_pulse_counter;

  // Pulse per second calculation (Eq.3.)
  // dN - impulses received
  // Ts - sampling time - time in between function calls
  // Th - time from last impulse
  // Th_1 - time form last impulse of the previous call
  // only increment if some impulses received
  float dt = Ts + prev_Th - Th;
  pulse_per_second = (dN != 0 && dt > Ts/2) ? dN / dt : pulse_per_second;

  // if more than 0.05 passed in between impulses
  if ( Th > 0.1) pulse_per_second = 0;

  // velocity calculation
  float velocity = pulse_per_second / ((float)cpr) * (_2PI);

  // save variables for next pass
  prev_timestamp_us = timestamp_us;
  // save velocity calculation variables
  prev_Th = Th;
  prev_pulse_counter = pulse_counter;
  return natural_direction * (velocity);
}

// getter for index pin
// return -1 if no index
int Encoder::needsAbsoluteZeroSearch(){
  return index_pulse_counter == 0;
}

// initialize counter to zero
float Encoder::initRelativeZero(){
  long angle_offset = -pulse_counter;
  pulse_counter = 0;
  pulse_timestamp = micros();
  return _2PI * (angle_offset) / ((float)cpr);
}
// initialize index to zero
float Encoder::initAbsoluteZero(){
  pulse_counter -= index_pulse_counter;
  prev_pulse_counter = pulse_counter;
  return (index_pulse_counter) / ((float)cpr) * (_2PI);
}


// encoder initialisation of the hardware pins
// and calculation variables
void Encoder::init(){

  // Encoder - check if pullup needed for your encoder
  if(pullup == Pullup::INTERN){
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);

  }else{
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
  }

  // counter setup
  pulse_counter = 0;
  pulse_timestamp = micros();
  // velocity calculation variables
  prev_Th = 0;
  pulse_per_second = 0;
  prev_pulse_counter = 0;
  prev_timestamp_us = micros();

  // initial cpr = PPR
  // change it if the mode is quadrature
  if(quadrature == Quadrature::ON) cpr = 4*cpr;

 switch(quadrature){
    case Quadrature::ON:
      // A callback and B callback
       attachInterrupt(digitalPinToInterrupt(pinA), call_intA, CHANGE);
       attachInterrupt(digitalPinToInterrupt(pinB), call_intB, CHANGE);
      break;
    case Quadrature::OFF:
      // A callback and B callback
       attachInterrupt(digitalPinToInterrupt(pinA),call_intA, RISING);
       attachInterrupt(digitalPinToInterrupt(pinB),call_intB, RISING);
      break;
      } //switch

 } //encoder


