//Volatile ENcoder
volatile long encoder_pulse_counter = 0;
volatile long direction = 1


void setup() {
    Serial.begin(9600);
    pinMode(encoder_a, INPUT_PULLUP);
    pinMode(encoder_b, INPUT_PULLUP);

    attachInterrupt(0, encoderPinChangeA, CHANGE);
    attachInterrupt(1, encoderPinChangeB, CHANGE);
}

void loop(){
    long speed = encoder_pulse_counter/2400.00*60; // Not sure, encoder not specified
    Serial.println(direction*speed);
    encoder_pulse_counter = 0; // Clear variable just before counting again 
    delay(1000);
}


void encoderPinChangeA(){
    encoder_pulse_counter += 1;
    direction = digitalRead(encoder_a) == digitalRead(encoder_b) ? -1 : 1;
}

void encoderPinChangeB(){
    encoder_pulse_counter += 1;
    direction = digitalRead(encoder_a) != digitalRead(encoder_b) ? -1 : 1;
}
