/*
Author: Robin Liu

This sketch controls four stepper motors through EasyDriver or similar driver boards.
It listens for serial input containing instructions for running the motors and executes those instructions.

Notes on movement:
- Positive # of steps rotates the motor shaft clockwise as viewed from the motor body.
- For the linear axis, clockwise rotation moves the magnet away from the workspace
- Linear: (200*8) steps per rev / 8 mm per rev = 200 steps per mm
- Angular: (200*8) steps per rev / 360 deg per rev = 4.44 steps per deg

Attribution:
- Code for serial interfacing from here: https://forum.arduino.cc/index.php?topic=288234.0

TODO:
- Make sure degrees are used throughout
*/

// Includes
#include <AccelStepper.h>
#include <Wire.h>
#include "Adafruit_MLX90393.h"
#include <math.h>

// PINS
const int stepPins[] = {2, 4, 6, 8};
const int dirPins[] = {3, 5, 7, 9};
const int motorCount = 4;
const int limitPins[] = {10, 11}; // Pull down

// Serial message variables
const byte msgNumChars = 32;
char receivedChars[msgNumChars];
bool newMsg = false; // flag to indicate if a new message has arrived

// Data reporting variables
unsigned long currentTime;
unsigned long previousReportTime;
unsigned long previousMagsenseTime;
const unsigned int reportingTimeInterval = 100; // milliseconds between outgoing messages
const unsigned int magsenseTimeInterval = 10; // milliseconds between magnetic field sensor readings FIXME: Does this sensor have relevant limitations?
float magfield_x, magfield_y, magfield_z; // Raw magnetic measurement values TODO: Figure out which axis we don't care about.
float magfield_magnitude, magfield_angle; // Derived field characteristics. These values are filtered.


// Motor control
AccelStepper Motor_A(AccelStepper::DRIVER, stepPins[0], dirPins[0]);
AccelStepper Motor_B(AccelStepper::DRIVER, stepPins[1], dirPins[1]);
AccelStepper Motor_C(AccelStepper::DRIVER, stepPins[2], dirPins[2]);
AccelStepper Motor_D(AccelStepper::DRIVER, stepPins[3], dirPins[3]);
int fluxTarget = 1;
float angleTarget = 0;

// Magnetic sensor control
Adafruit_MLX90393 magSensor = Adafruit_MLX90393();

//###########################################
//###########################################

void setup() {
  // Set up motor settings. These can be tuned to the desired performance.
  Motor_A.setMaxSpeed(1000);
  Motor_A.setAcceleration(500);
  Motor_B.setMaxSpeed(50);
  Motor_B.setAcceleration(100);
  Motor_C.setMaxSpeed(50);
  Motor_C.setAcceleration(100);
  Motor_D.setMaxSpeed(1000);
  Motor_D.setAcceleration(500);

  // Set up limit switch pins
  pinMode(limitPins[0], INPUT);
  pinMode(limitPins[1], INPUT);

  // Set up serial
  Serial.begin(9600);
  Serial.println("Serial ready");

  // Set up magnetic field sensor
  if (magSensor.begin_I2C()) { // TODO: Figure out correct pins
    Serial.println("Found a MLX90393 sensor");
  }
  // TODO: investigate built in filtering, gain, resolution, oversampling
  // https://adafruit.github.io/Adafruit_MLX90393_Library/html/class_adafruit___m_l_x90393.html
  // https://github.com/adafruit/Adafruit_MLX90393_Library

  // Start timer
  currentTime = millis();
  previousMagsenseTime = millis();
  previousReportTime = millis();
}

void loop() {
  currentTime = millis();

  // # Check for new messages in serial port
  recvWithEndMarker();

  // # Check if a new position command has been issued.
  if (newMsg == true) {
    parseMsg();
    newMsg = false;
  }

  // # Take field measurement
  if (currentTime - previousMagsenseTime > magsenseTimeInterval) {
    previousMagsenseTime = currentTime;
    getFieldValues();
  }

  // # Decide on the target position
  // Consider the following:
  // 1. External commands
  // 2. Limit switch signals
  // 3. Other safety stops (field maxima, etc.)

  // (1) For a valid message, set motor target in accordance with the message
  // This must be called every loop since this is the proportional gain controller
  setStepTarget();
  // (2) adjust target for limit switches.
  if (digitalRead(limitPins[0]) && Motor_A.targetPosition()-Motor_A.currentPosition()) > 0) {
    Motor_A.stop();
  }
  if (digitalRead(limitPins[1]) && Motor_D.targetPosition()-Motor_D.currentPosition() > 0) {
    Motor_D.stop();
  }
  // TODO: (3) adjust target for field safety

  // # Run motors -
  runMotors();

  // # Handle periodic data reporting
  if (currentTime - previousReportTime > reportingTimeInterval) {
    previousReportTime = currentTime;
    writeTelemetry();
  }
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newMsg == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= msgNumChars) {
        ndx = msgNumChars - 1;
      }
    }
    else { // end marker character was received
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newMsg = true;
    }
  }
}

void parseMsg() {
  /*
  Read through the received message

  TODO: Implement a stop signal

  # Message descriptions:
  byte 1: one byte indicating the message type

  ## Message type 'f': direct field input
  bytes 2-4: 3 char string: flux intensity (mT).
  bytes 5-9: 5 char string: field angle (rad). format #.###

  ## Message type 's': stop immediately
  */

  bool validMsg = true;
  int fluxTarget_temp = 0;
  float angleTarget_temp = 0;

  int newDigit = 0;
  byte newControlMode = 0;

  if receivedChars[0]-'a' == 'f' {
    // direct field control
    for (int i = 1; i < 4; i++) {
      // read flux intensity chars
      newDigit = receivedChars[i]-'0';
      if (newDigit >= 0 && newDigit <= 9) {
        fluxTarget_temp = (10 * fluxTarget_temp) + newDigit;
      }
      else {
        validMsg = false;
        break;
      }
    }
    for (int i = 4; i < 9; i++) {
      // read field angle chars
      if (i == 4) {
        newDigit = receivedChars[i]-'0';
        if (newDigit >= 0 && newDigit <= 9) {
          angleTarget_temp = newDigit;
        }
        else {
          validMsg = false;
          break;
        }
      }
      else if (i >= 6) {
        newDigit = receivedChars[i]-'0';
        if (newDigit >= 0 && newDigit <= 9) {
          angleTarget_temp = angleTarget_temp + (newDigit / pow(10, i-5);
        }
        else {
          validMsg = false;
          break;
        }
      }
    }
    if (validMsg) {
      fluxTarget = fluxTarget_temp;
      angleTarget = angleTarget_temp;
    }
  }
  else if receivedChars[0]-'a' == 's' {
    // Stop immediately
    // TODO:
  }
}

void runMotors() {
  // to be called as often as possible
  Motor_A.run();
  Motor_B.run();
  Motor_C.run();
  Motor_D.run();
}

void setStepTarget() {
  // Proportional controller to set the target step value
  // Apply a tolerance to avoid vibrations.
  // TODO: angle can be rotated 180 deg without consequence. Adjust for this?

  float fieldDiff = fluxTarget - magfield_magnitude; // If positive, the motors need to move closer together (negative rotation)
  float angleDiff = angleTarget - magfield_angle; // TODO: figure out directionality

  // Adjust gain values here
  long linearSteps = -20 * fieldDiff; // 1 step for every 0.05 mT of error
  long angularSteps = 4 * angleDiff; // 1 step for every 0.25 deg of error

  // Apply saturation values to prevent vibrations around the target
  if (abs(linearSteps) <= 10) { // 10 steps = 0.05 mm
    linearSteps = 0;
  }
  if (abs(angularSteps <= 1)) { // 1 step ~= 0.23 deg
    angularSteps = 0;
  }

  Motor_A.moveTo(Motor_A.currentPosition() + linearSteps);
  Motor_D.moveTo(Motor_D.currentPosition() + linearSteps);

  // TODO: Figure out directionality
  Motor_B.moveTo(Motor_B.currentPosition() + angularSteps);
  Motor_C.moveTo(Motor_C.currentPosition() - angularSteps);
}

void getFieldValues() {
  int weight_old = 4; // weight applied to old value for weighted average (new value has a weight of 1)
  // Read magnetic sensor data
  // Calculate field magnitude and angle
  // Apply low pass filter - weighted average of previous value and new value

  if (sensor.readData(&magfield_x, &magfield_y, &magfield_z)) {
    // valid reading
    float new_magfield_magnitude = sqrt(magfield_x*magfield_x + magfield_y*magfield_y + magfield_z*magfield_z); // TODO: verify correct axes used
    float new_magfield_angle = atan2(magfield_y, magfield_x); // TODO: verify correct axes used, convert to deg

    magfield_magnitude = (weight_old*magfield_magnitude + new_magfield_magnitude) / (weight + 1);
    magfield_angle = (weight_old*magfield_angle + new_magfield_angle) / (weight + 1);
  }
}

void writeTelemetry() {
  // Send messages through serial to connected computer

  // Report motor positions
  Serial.print("steps,");
  Serial.print(Motor_A.currentPosition());
  Serial.print(",");
  Serial.print(Motor_B.currentPosition());
  Serial.print(",");
  Serial.print(Motor_C.currentPosition());
  Serial.print(",");
  Serial.print(Motor_D.currentPosition());
  // Report field conditions
  Serial.print(",");
  Serial.print(magfield_magnitude);
  Serial.print(",");
  Serial.print(magfield_angle);
  // End the message
  Serial.print("\r\n");
}