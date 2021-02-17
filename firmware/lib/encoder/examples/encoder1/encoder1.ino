
/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
#define MAX_RPM 80

#include <Encoder.h>
#include <Motor.h>
#include <PID.h>

#define arduinoInt1 2             // Arduino UNO interrupt 0
#define arduinoInt2 3             // Arduino UNO interrupt 1


Encoder encoder = Encoder(arduinoInt1, arduinoInt2);
void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
  
}

long oldPosition  = -999;

void loop() {
  long newPosition = encoder.getRPM(MAX_RPM);
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
   Serial.println(newPosition);
  }
}
