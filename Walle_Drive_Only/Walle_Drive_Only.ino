//Programmer arduinoISP

/* WALL-E CONTROLLER CODE
 ********************************************
 * Code by: Simon Bluett
 * Email:   hello@chillibasket.com
 * Version: 2.5
 * Date:    25th January 2020
 ********************************************/

/* HOW TO USE:
 * 1. Install the Adafruit_PWMServoDriver library
 *    a. In the Arduino IDE, go to Sketch->Include Library->Manage Libraries
 *    b. Search for Adafruit PWM Library, and install the latest version
 * 2. Calibrate the servo motors, using the calibration sketch provided in the
 *    GitHub repository. Paste the calibrated values between line 85 to 92
 * 3. Upload the sketch to the micro-controller, and open serial monitor at 
 *    a baud rate of 115200.
 * 4. Additional instructions and hints can be found at:
 *    https://wired.chillibasket.com/3d-printed-wall-e/
 */

#include <Wire.h>
#include "LedControl.h"
#include "binary.h"
//#include <Adafruit_PWMServoDriver.h>
//#include "Queue.hpp"
#include "MotorController.hpp"
#include "BatLevel.hpp"


// Define the pin-mapping
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define DIR_L 12           // Motor direction pins
#define DIR_R 13
#define PWM_L 10           // Motor PWM pins
#define PWM_R 11
#define BRK_L  9           // Motor brake pins
#define BRK_R  8
#define SR_OE 3           // Servo shield output enable pin ***switched from pin 10? maybe it has to go there??
/*
 DIN connects to pin 7
 CLK connects to pin 6
 CS connects to pin 5
 analog 1
*/

// Define other constants
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define FREQUENCY 10       // Time in milliseconds of how often to update servo and motor positions
#define SERVOS 7           // Number of servo motors
#define THRESHOLD 1        // The minimum error which the dynamics controller tries to achieve
#define MOTOR_OFF 6000      // Turn servo motors off after 6 seconds
#define MAX_SERIAL 5       // Maximum number of characters that can be received

// Set up motor controller classes
MotorController motorL(DIR_L, PWM_L, BRK_L, false);
MotorController motorR(DIR_R, PWM_R, BRK_R, false);

// Motor Control Variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
int pwmspeed = 255;
int moveVal = 0;
int turnVal = 0;
int turnOff = 0;


// Runtime Variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
unsigned long lastTime = 0;
unsigned long animeTimer = 0;
unsigned long motorTimer = 0;
unsigned long updateTimer = 0;
bool autoMode = false;


// Serial Parsing
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
char firstChar;
char serialBuffer[MAX_SERIAL];
uint8_t serialLength = 0;

// Servo Control - Position, Velocity, Acceleration
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
// Servo Pins:       0,   1,   2,   3,   4,   5,   6,   -,   -
// Joint Name:    head,necT,necB,eyeR,eyeL,armL,armR,motL,motR
float curpos[] = { 248, 560, 140, 475, 270, 250, 290, 180, 180};  // Current position (units)
float setpos[] = { 248, 560, 140, 475, 270, 250, 290,   0,   0};  // Required position (units)
float curvel[] = {   0,   0,   0,   0,   0,   0,   0,   0,   0};  // Current velocity (units/sec)
float maxvel[] = { 500, 750, 255,2400,2400, 500, 500, 255, 255};  // Max Servo velocity (units/sec)
float accell[] = { 350, 480, 150,1800,1800, 300, 300, 800, 800};  // Servo acceleration (units/sec^2)

// ------------------------------------------------------------------
//    INITIAL SETUP
// ------------------------------------------------------------------
void setup() {

  // Initialize serial communication for debugging
  Serial.begin(115200);

  randomSeed(analogRead(0));

  lc.shutdown(0,false);
  // Set brightness to a medium value
  lc.setIntensity(0,15); //1-15
  // Clear the display
  lc.clearDisplay(0); 

  Serial.println("Starting Program");
}

// -------------------------------------------------------------------
//    READ INPUT FROM SERIAL
// -------------------------------------------------------------------
void readSerial() {
  // Read incoming byte
  char inchar = Serial.read();

  // If the string has ended, evaluate the serial buffer
  if (inchar == '\n' || inchar == '\r') {

    if (serialLength > 0) evaluateSerial();
    serialBuffer[0] = 0;
    serialLength = 0;

  // Otherwise add to the character to the buffer
  } else {
    if (serialLength == 0) firstChar = inchar;
    else {
      serialBuffer[serialLength-1] = inchar;
      serialBuffer[serialLength] = 0;
    }
    serialLength++;

    // To prevent overflows, evalute the buffer if it is full
    if (serialLength == MAX_SERIAL) {
      evaluateSerial();
      serialBuffer[0] = 0;
      serialLength = 0;
    }
  }
}


// -------------------------------------------------------------------
//    EVALUATE INPUT FROM SERIAL
// -------------------------------------------------------------------
void evaluateSerial() {
  // Evaluate integer number in the serial buffer
  int number = atoi(serialBuffer);

  Serial.print(firstChar); Serial.println(number);

  // Motor Inputs and Offsets
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  if      (firstChar == 'X' && number >= -100 && number <= 100) turnVal = int(number * 2.55);     // Forward/reverse control
  else if (firstChar == 'Y' && number >= -100 && number <= 100) moveVal = int(number * 2.55);     // Left/right control
  else if (firstChar == 'S' && number >=  100 && number <= 100) turnOff = number;           // Steering offset
  else if (firstChar == 'O' && number >=    0 && number <= 250) curpos[7] = curpos[8] = int(number);  // Motor deadzone offset
  
  // Manual Movements with WASD
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  else if (firstChar == 'w') {    // Forward movement
    moveVal = pwmspeed;
    turnVal = 0;
  }
  else if (firstChar == 'q') {    // Stop movement
    moveVal = 0;
    turnVal = 0;
  }
  else if (firstChar == 's') {    // Backward movement
    moveVal = -pwmspeed;
    turnVal = 0;
  }
  else if (firstChar == 'a') {    // Drive & look left
    moveVal = 0;
    turnVal = -pwmspeed;
  }
  else if (firstChar == 'd') {      // Drive & look right
    moveVal = 0;
    turnVal = pwmspeed;
  }
}

// -------------------------------------------------------------------
//    MANAGE THE MOVEMENT OF THE MAIN MOTORS
// -------------------------------------------------------------------
void manageMotors(float dt) {
  // Update Main Motor Values
  setpos[7] = moveVal - turnVal - turnOff;
  setpos[8] = moveVal + turnVal + turnOff;

  // MAIN DRIVING MOTORS
  // -  -  -  -  -  -  -  -  -  -  -  -  -
  for (int i = SERVOS; i < SERVOS + 2; i++) {

    float velError = setpos[i] - curvel[i];

    // If velocity error is above the threshold
    if (abs(velError) > THRESHOLD && (setpos[i] != -1)) {

      // Determine whether to accelerate or decelerate
      float acceleration = accell[i];
      if (setpos[i] < curvel[i] && curvel[i] >= 0) acceleration = -accell[i];
      else if (setpos[i] < curvel[i] && curvel[i] < 0) acceleration = -accell[i]; 
      else if (setpos[i] > curvel[i] && curvel[i] < 0) acceleration = accell[i];

      // Update the current velocity
      float dV = acceleration * dt / 1000.0;
      if (abs(dV) < abs(velError)) curvel[i] += dV;
      else curvel[i] = setpos[i];
    } else {
      curvel[i] = setpos[i];
    }
    
    // Limit Velocity
    if (curvel[i] > maxvel[i]) curvel[i] = maxvel[i];
    if (curvel[i] < -maxvel[i]) curvel[i] = -maxvel[i];
  }

  // Update motor speeds
  motorL.setSpeed(curvel[SERVOS]);
  motorR.setSpeed(curvel[SERVOS+1]);
}


// -------------------------------------------------------------------
//    MAIN PROGRAM LOOP
// -------------------------------------------------------------------
void loop() {

  // Read any new serial messages
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  if (Serial.available() > 0){
    readSerial();
  }

  // display bat level
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  int batValue = analogRead(A1);
  float voltage = batValue*((5.0/1023.0))-.28;
  batvolt(voltage);

  // Load or generate new animations
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  //manageAnimations();


  // Move Servos and wheels at regular time intervals
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  if (updateTimer < millis()) {
    updateTimer = millis() + FREQUENCY;

    unsigned long newTime = micros();
    float dt = (newTime - lastTime) / 1000.0;
    lastTime = newTime;

    //manageServos(dt);
    manageMotors(dt);
  }
}
