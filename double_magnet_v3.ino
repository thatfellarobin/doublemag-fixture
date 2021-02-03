/*
Author: Robin Liu

This sketch controls four stepper motors through EasyDriver or similar driver boards.
It listens for serial input containing instructions for running the motors and executes those instructions.

Attribution:
- Code for serial interfacing from here: https://forum.arduino.cc/index.php?topic=288234.0

TODO:
-
*/

// Includes
#include <AccelStepper.h>

// PINS
const int stepPins[] = {2, 4, 6, 8};
const int dirPins[] = {3, 5, 7, 9};
const int motorCount = 4;

// Serial message variables
const byte msgNumChars = 32;
char receivedChars[msgNumChars];
boolean newMsg = false;
boolean validMsg = false; // If the parser finds a valid message, this flag is flipped true
unsigned long currentTime;
unsigned long previousTime;
const unsigned int timeInterval = 200; // milliseconds between position reports

// Motor control
AccelStepper Motor_A(AccelStepper::DRIVER, stepPins[0], dirPins[0]);
AccelStepper Motor_B(AccelStepper::DRIVER, stepPins[1], dirPins[1]);
AccelStepper Motor_C(AccelStepper::DRIVER, stepPins[2], dirPins[2]);
AccelStepper Motor_D(AccelStepper::DRIVER, stepPins[3], dirPins[3]);
byte controlMode = 0;
long motorPositions[motorCount] = {}; // stores the target motor positions relative to the startup state
byte activeMotor; // The motor commanded to move by the most recent message
long steps; // The number of steps commanded to move by the most recent message

//---------------------------------
//---------------------------------

void setup() {
  // Set up motor settings. These can be tuned to the desired performance.
  Motor_A.setMaxSpeed(1400);
  Motor_A.setAcceleration(500);
  Motor_B.setMaxSpeed(50);
  Motor_B.setAcceleration(100);
  Motor_C.setMaxSpeed(50);
  Motor_C.setAcceleration(100);
  Motor_D.setMaxSpeed(1400);
  Motor_D.setAcceleration(500);

  // Set initial motor target
  Motor_A.moveTo(motorPositions[0]);
  Motor_B.moveTo(motorPositions[1]);
  Motor_C.moveTo(motorPositions[2]);
  Motor_D.moveTo(motorPositions[3]);

  // Set up serial
  Serial.begin(9600);
  Serial.println("Arduino is ready");

  // Start timer
  currentTime = millis();
  previousTime = millis();
}

void loop() {
  // Check if there is a new message
  recvWithEndMarker();

  if (newMsg == true) {
    // Received a new message.
    parseMsg();

    // If the message is valid, run the motors in accordance with the message
    if (validMsg == true && controlMode == 'W') { // Waypoint control
      motorPositions[activeMotor] = steps;

      // Using moveTo() instead of move() here because we want to handle the case where a new waypoint is sent before the motor has finished moving to the old one.
      Motor_A.moveTo(motorPositions[0]);
      Motor_B.moveTo(motorPositions[1]);
      Motor_C.moveTo(motorPositions[2]);
      Motor_D.moveTo(motorPositions[3]);
    }
    else if (validMsg == true && controlMode == 'M') { // Manual control
      if (steps != 0) { // move forward or backward
        steps = steps * 400; // This value should be tuned based on the max stepper speed and the frequency of messages from the Pi.
        switch (activeMotor) {
          case 0:
            Motor_A.move(steps);
            break;
          case 1:
            Motor_B.move(steps);
            break;
          case 2:
            Motor_C.move(steps);
            break;
          case 3:
            Motor_D.move(steps);
            break;
          default:
            break;
        }
      }
      else { // stop
        switch (activeMotor) {
          case 0:
            Motor_A.stop();
            break;
          case 1:
            Motor_B.stop();
            break;
          case 2:
            Motor_C.stop();
            break;
          case 3:
            Motor_D.stop();
            break;
          default:
            break;
        }
      }

      // Update motorPositions[] with the new target.
      // This is necessary so that when you're switching from manual to waypoint control, we update the target correctly.
      motorPositions[0] = Motor_A.targetPosition();
      motorPositions[1] = Motor_B.targetPosition();
      motorPositions[2] = Motor_C.targetPosition();
      motorPositions[3] = Motor_D.targetPosition();
    }
    else {
      // Improper message format
      Serial.println("Bad message.");
    }
    newMsg = false;
  }

  //Run motors
  Motor_A.run();
  Motor_B.run();
  Motor_C.run();
  Motor_D.run();

  // Report position every desired time period
  currentTime = millis();
  if (currentTime - previousTime > timeInterval) {
    previousTime = currentTime;
    Serial.print("steps,");
    Serial.print(Motor_A.currentPosition());
    Serial.print(",");
    Serial.print(Motor_B.currentPosition());
    Serial.print(",");
    Serial.print(Motor_C.currentPosition());
    Serial.print(",");
    Serial.print(Motor_D.currentPosition());
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

  byte 1: one byte indicating the control/request type
    - 'M': manual control
    - 'W': waypoint control

  WAYPOINT CONTROL:
  The message should indicate the following:
  byte 2: one byte indicating which motor is being referred to ('a', 'b', 'c', 'd')
  bytes 3-8: six bytes indicating the absolute step count to navigate to to take.
    - First byte is most significant digit
    - '0' to '9'.
    - Max num steps that can be commanded is 999999
  byte 9: byte indicating the sign ('-' is negative, '+' or '0' is positive)

  MANUAL CONTROL:
  byte 2: one byte indicating which motor is being referred to ('a', 'b', 'c', 'd')
  byte 3: one byte indicating direction. '+' is forward, '-' is backward, '0' is stop
  */

  int newDigit = 0;
  byte newControlMode = 0;

  validMsg = true; // Assume the message is valid at first
  steps = 0;

  // Process the first byte to determine the control type
  newControlMode = receivedChars[0];
  if (newControlMode == 'M') { // Manual control
    controlMode = 'M';

    // process the next byte to determine the active motor
    activeMotor = receivedChars[1] - 'a';
    if (activeMotor < 0 || activeMotor > 3) {
      validMsg = false;
    }

    // process the next byte to determine the direction
    if (receivedChars[2] == '+') {
      steps = 1;
    }
    else if (receivedChars[2] == '-') {
      steps = -1;
    }
    else if (receivedChars[2] == '0') {
      steps = 0;
    }
    else {
      validMsg = false;
    }
  }
  else if (newControlMode == 'W') { // Waypoint control
    controlMode = 'W';
    for (int i = 1; i < 9; i++) {
      if (i == 1) { // motor byte
        activeMotor = receivedChars[i] - 'a';
        if (activeMotor < 0 || activeMotor > 3) {
          validMsg = false;
        }
      }
      else if (i > 1 && i < 8) { // steps bytes or manual control byte
        newDigit = receivedChars[i] - '0';
        if (newDigit <= 9 && newDigit >= 0) {
          steps = (10 * steps) + newDigit;
        }
        else {
          validMsg = false;
        }
      }
      else { // direction byte
        if (receivedChars[i] == '-') {
          steps = -steps;
        }
        else if (receivedChars[i] == '+'  || receivedChars[i] == '0') {
          //do nothing
        }
        else {
          validMsg = false;
        }
      }
    }
  }
  else {
    // don't change the control mode
    validMsg = false;
  }
}
