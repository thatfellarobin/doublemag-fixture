/*
Author: Robin Liu

This sketch controls four stepper motors through EasyDriver or similar driver boards.
It listens for serial input containing instructions for running the motors and executes those instructions.

Notes on direction:
- Positive # of steps rotates the motor shaft clockwise as viewed from the motor body.
- For the linear axis, clockwise rotation moves the magnet away from the workspace

Attribution:
- Code for serial interfacing from here: https://forum.arduino.cc/index.php?topic=288234.0

TODO:
-
*/

// Includes
#include <AccelStepper.h>
#include <Wire.h>
#include "Adafruit_MLX90393.h"

// PINS
const int stepPins[] = {2, 4, 6, 8};
const int dirPins[] = {3, 5, 7, 9};
const int motorCount = 4;
const int limitPins[] = {10, 11}; // Pull down

// Serial message variables
const byte msgNumChars = 32;
char receivedChars[msgNumChars];
long steps; // The absolute step count target received in the most recent message.
byte activeMotor; // The motor commanded to move by the most recent message
boolean newMsg = false; // flag to indicate if a new message has arrived
boolean validMsg = false; // If the parser finds a valid message, this flag is flipped true

// Data reporting variables
unsigned long currentTime;
unsigned long previousTime;
const unsigned int timeInterval = 50; // milliseconds between outgoing messages
float mag_x, mag_y, mag_z; // Magnetic measurement values

// Motor control
AccelStepper Motor_A(AccelStepper::DRIVER, stepPins[0], dirPins[0]);
AccelStepper Motor_B(AccelStepper::DRIVER, stepPins[1], dirPins[1]);
AccelStepper Motor_C(AccelStepper::DRIVER, stepPins[2], dirPins[2]);
AccelStepper Motor_D(AccelStepper::DRIVER, stepPins[3], dirPins[3]);
long motorStepTarget[motorCount] = {}; // stores the target motor positions relative to the startup state
float speed_A, speed_B;

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

  // Set initial motor target
  Motor_A.moveTo(motorStepTarget[0]);
  Motor_B.moveTo(motorStepTarget[1]);
  Motor_C.moveTo(motorStepTarget[2]);
  Motor_D.moveTo(motorStepTarget[3]);

  // Set up limit switch pins
  pinMode(limitPins[0], INPUT);
  pinMode(limitPins[1], INPUT);

  // Set up serial
  Serial.begin(9600);
  Serial.println("Serial ready");

  if (sensor.begin()) {
    Serial.println("Found a MLX90393 sensor");
  }

  // Start timer
  currentTime = millis();
  previousTime = millis();
}

void loop() {
  // Check if there is a new message
  recvWithEndMarker();

  if (newMsg == true) {
    // Received a new message.
    //Parse it to see if it's valid, and if it is, extract the useful information.
    parseMsg();

    // If the message is valid, run the motors in accordance with the message
    if (validMsg) {
      // Make sure the command doesn't exceed the end stop.
      if (digitalRead((limitPins[0]) && activeMotor == 0 || digitalRead(limitPins[1]) && activeMotor == 3) && steps > 0) {
        // don't move the motor
      }
      else { // No limit pin problems
        motorStepTarget[activeMotor] = steps;
        Motor_A.moveTo(motorStepTarget[0]);
        Motor_B.moveTo(motorStepTarget[1]);
        Motor_C.moveTo(motorStepTarget[2]);
        Motor_D.moveTo(motorStepTarget[3]);
      }
    }
    else {
      // Improper message format
      Serial.println("Bad message.");
    }

    // Reset new message flag
    newMsg = false;
  }

  // Limit switch instant stop
  speed_A = Motor_A.speed();
  speed_B = Motor_B.speed();
  if (digitalRead((limitPins[0]) && speed_A > 0) {
    // Stop motor A as soon as possible
    Motor_A.runToPosition(Motor_A.currentPosition())
  }
  if (digitalRead((limitPins[0]) && speed_B > 0) {
    // Stop motor D as soon as possible
    Motor_B.runToPosition(Motor_A.currentPosition())
  }

  // Run motors
  Motor_A.run();
  Motor_B.run();
  Motor_C.run();
  Motor_D.run();

  // Handle periodic data reporting
  currentTime = millis();
  if (currentTime - previousTime > timeInterval) {
    previousTime = currentTime;

    // Report motor positions
    Serial.print("steps,");
    Serial.print(Motor_A.currentPosition());
    Serial.print(",");
    Serial.print(Motor_B.currentPosition());
    Serial.print(",");
    Serial.print(Motor_C.currentPosition());
    Serial.print(",");
    Serial.print(Motor_D.currentPosition());

    // Take field measurement
    if (sensor.readData(&mag_x, &mag_y, &mag_z)) {
      // Report field conditions
      Serial.print(",");
      Serial.print(mag_x);
      Serial.print(",");
      Serial.print(mag_y);
      Serial.print(",");
      Serial.print(mag_z);
    }
    else {
      // Always the same number of commas regardless of field measurement success
      Serial.print(",")
      Serial.print(",")
      Serial.print(",")
    }

    // End the message
    Serial.print("\r\n");
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

  byte 1: one byte indicating which motor is being referred to ('a', 'b', 'c', 'd')
  bytes 2-7: six bytes indicating the absolute step count to navigate towards.
    - First byte is most significant digit
    - '0' to '9'.
    - Max num steps that can be commanded is 999999
  byte 8: byte indicating the sign ('-' is negative, '+' is positive)
  */

  int newDigit = 0;
  byte newControlMode = 0;

  validMsg = true; // Assume the message is valid at first
  steps = 0;

  // Process the first byte to determine the control type
  for (int i = 0; i < 8; i++) {
    if (i == 0) { // motor byte
      activeMotor = receivedChars[i] - 'a';
      if (activeMotor < 0 || activeMotor > 3) {
        validMsg = false;
        break;
      }
    }
    else if (i > 0 && i < 7) { // step target bytes
      newDigit = receivedChars[i] - '0';
      if (newDigit >= 0 && newDigit <= 9) {
        steps = (10 * steps) + newDigit;
      }
      else {
        validMsg = false;
        break;
      }
    }
    else { // direction byte
      if (receivedChars[i] == '-') {
        steps = -steps;
      }
      else if (receivedChars[i] == '+') {
        //do nothing
      }
      else {
        validMsg = false;
        break;
      }
    }
  }
}
