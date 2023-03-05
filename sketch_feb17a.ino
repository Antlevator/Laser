//Includes the Arduino Stepper Library
#include <Stepper.h>

// Defines the number of steps per rotation
const int stepsPerRevolution = 4096/2;

const int MIN_SPEED = 1;
const int MAX_SPEED = 10;

// Creates an instance of stepper class
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper stepper1 = Stepper(stepsPerRevolution, 10, 12, 11, 13);
Stepper stepper2 = Stepper(stepsPerRevolution, 4, 6, 5, 7);

int periodInTicks = 10000;
int amount = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  amount++;
  const int val = MAX_SPEED * cos(amount / periodInTicks);
  stepper1.setSpeed(val);
  stepper1.step(stepsPerRevolution/2048);
  stepper2.setSpeed(MAX_SPEED - val);
  stepper2.step(stepsPerRevolution/2048);
  
}
