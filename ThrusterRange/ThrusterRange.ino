#include <Servo.h>

Servo esc;
int stoppedSignal = 1500; // set the stopped signal to 1500 microseconds

void setup() {
  esc.attach(9); // attach the ESC signal wire to pin 9
  delay(5000); // wait for 5 seconds to allow the ESC to initialize
  esc.writeMicroseconds(stoppedSignal); // send the stopped signal
  delay(500); // wait for half a second
}

void loop() {
  // the range of the first direction from 600 to 1400 and the other is from 1600 to 2300 
  esc.write(500);
}