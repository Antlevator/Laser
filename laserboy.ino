
#include <AccelStepper.h>
#include <MultiStepper.h>

const int MODE_CALIBRATION = 0;
const int MODE_EXECUTION = 1;
const int MODE_FAILURE = 2;

// config
const long LOG_BAUD_RATE = 115200;
const int STEPS_PER_REV = 2048; // for the stepper motor
const int _MAX_SPEED = 200;

const int CALIBRATION_SPEED = 100;
const int CALIBRATION_ACCELERATION = 100;
const int angle_in1 = 3;
const int angle_in2 = 4;
const int angle_in3 = 5;
const int angle_in4 = 6;
const int radius_in1 = 7;
const int radius_in2 = 8;
const int radius_in3 = 9;
const int radius_in4 = 10;
// hall effect sensor is low when switched on, high otherwise
//    see  https://html.alldatasheet.com/html-pdf/55092/ALLEGRO/A3144/991/4/A3144.html
// high is Vin, low is 175mV, so a digital input should be sufficient to detect, if not, we can use an analog and look for values < (bits in ADC) * (2mV) / (Vin)
//    see  https://html.alldatasheet.com/html-pdf/55092/ALLEGRO/A3144/742/3/A3144.html
//    also https://forum.arduino.cc/t/how-does-digitalread-determine-if-pin-is-high-or-low/108873/4
// this means for an Arduino 101 (which btw is a 3.3V board with 1024 value resolution on analog input) we cant power the 4.5v minimum operating voltage 
//    of the hall effect sensor unless we use the 5v pin, then we need to make sure we dont fry the board by making sure input voltage is
//    not ever >3.3v (it will be 4.5v+, as it is set high by default), which I think means we need a voltage divider to bring it down from 3.3v to 5v logic level again.

const int angle_center_in = 11;
const int radius_center_in = 12;

const int failure_indicator_out = 13;
const int laser_out = 15;

// note, DO NOT USE PIN 13 on ATMega!!! see https://docs.arduino.cc/learn/microcontrollers/digital-pins

// TODO check current minimum on stepper driver boards, if we can get away with tying pins we can free up slip ring wires and move everything to the base, 
//    this would help with weight and complexity
//    see https://en.wikipedia.org/wiki/ULN2003A#:~:text=The%20ULN2003A%20is%20an%20integrated,SOIC%2C%20SOP%20or%20TSSOP%20packaging.

AccelStepper angleStepper(AccelStepper::FULL4WIRE, angle_in1, angle_in3, angle_in2, angle_in4);
AccelStepper radiusStepper(AccelStepper::FULL4WIRE, radius_in1, radius_in3, radius_in2, radius_in4);

MultiStepper steppers;

int mode = MODE_CALIBRATION;

// calibration step

void setup() {
  
  // set up steppers
  steppers.addStepper(angleStepper);
  steppers.addStepper(radiusStepper);
  angleStepper.setMaxSpeed(_MAX_SPEED);
  radiusStepper.setMaxSpeed(_MAX_SPEED);
  angleStepper.setAcceleration(CALIBRATION_ACCELERATION);
  radiusStepper.setAcceleration(CALIBRATION_ACCELERATION);

  // set up pin modes
  pinMode(angle_center_in, INPUT);
  pinMode(radius_center_in, INPUT);
  pinMode(failure_indicator_out, OUTPUT);
  pinMode(laser_out, OUTPUT);
  
  // set initial values of output pins
  digitalWrite(failure_indicator_out, LOW);
  digitalWrite(laser_out, LOW);
  Serial.begin(LOG_BAUD_RATE);
  Serial.write("setup complete");
}

void loop() {
  if(mode == MODE_EXECUTION) {
    execute();
  } else if(mode == MODE_CALIBRATION) {
    calibrate();
  } else if(mode == MODE_FAILURE) {
    failure();
  }
}

void failure() {
  while(true) {
    digitalWrite(failure_indicator_out, HIGH);
    delay(100);
    digitalWrite(failure_indicator_out, LOW);
    delay(100);
  }
}

void calibrate() {
  // first, move the laser pointer until the sheath's magnet triggers the hall effect sensor on the platform
//  Serial.write("calibrating radius stepper...");
//  radiusStepper.setSpeed(CALIBRATION_SPEED);
//  bool foundRadiusCenter = false;
//  for(int i = 0; i < STEPS_PER_REV; i++) {
//    radiusStepper.run();
//    if(digitalRead(radius_center_in) == HIGH) {
//      Serial.write("found center for radius, moving to opposite side to set position...");
//      radiusStepper.runToNewPosition(STEPS_PER_REV / 2);
//      radiusStepper.setCurrentPosition(0);
//      Serial.write("center position set!");
//      foundRadiusCenter = true;
//      break;
//    }
//  }
//  if(!foundRadiusCenter) {
//    Serial.write("Failed to calibrate radius stepper!");
//    mode = MODE_FAILURE;
//    return;
//  }
  
  // second, rotate the platform until it's magnet triggers the hall effect sensor on the base
  Serial.write("calibrating angle stepper...");
  angleStepper.setSpeed(CALIBRATION_SPEED);
  bool foundAngleCenter = false;
  for(int i = 0; i < STEPS_PER_REV; i++) {
    angleStepper.run();
    if(digitalRead(angle_center_in) == HIGH) {
      angleStepper.setCurrentPosition(0);
      Serial.write("center position set!");
      foundAngleCenter = true;
      break;
    }
  }
  if(!foundAngleCenter) {
    Serial.write("ERROR: Failed to calibrate angle stepper!");
    mode = MODE_FAILURE;
    return;
  }
  // we're done calibrating, start moving the laser!
  Serial.write("all calibrations complete!");
  mode = MODE_EXECUTION;
  digitalWrite(laser_out, HIGH);
}

void execute() {
  digitalWrite(failure_indicator_out, HIGH);
  delay(100);
}
