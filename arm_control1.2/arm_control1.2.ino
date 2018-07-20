// arm control V1.1
// ivan patino, john so, kenneth thai, tiffany truong
// last updated 071018
// implements control for a series of Dynamixel XL-320 motors using a potentionmeter and flex sensor 
// credits to https://github.com/hackerspace-adelaide/XL320 for software library and info on wiring and use

// includes -----------------------------------------------------------------------------------------------------------------

#include "XL320.h" // XL-320 library (from https://github.com/hackerspace-adelaide/XL320)
#include <SoftwareSerial.h> // to enable both robot communication and debugging (also potenially computer based control later?)

// variable declarations ----------------------------------------------------------------------------------------------------

XL320 arm; // robot declaration
SoftwareSerial robotSerial(10, 11); // extra channels for communication (RX, TX)

int flex = A0; // flex sensor at A0; used to control the position of the arm
int pot = A1; // potentiometer at A1; used to control the orientation of the base

int flexPos; // tracks the flex sensor value
int potPos; // tracks the potentiometer value
int motorPos; // debug; can be used to track a motor value across program
int armPos;
int wristPos;
int forearmPos;

char rgb[] = "rgbypcwo"; // color array for servo LEDs (identification)

int armID = 1; // unique IDs for individual servo motors allows for simultaneous function in separate tasks 
int baseID = 2;
int elbowID = 4;
int forearmID = 3;

int timeMillis = 0; // debug; timeMillis does not pause the program while running (unlike the delay command)

// setup ---------------------------------------------------------------------------------------------------------------------

void setup() {
  pinMode(flex, INPUT); // initialize pins
  pinMode(pot, INPUT);

  Serial.begin(9600); // initialize Serial communication between Arduino and computer (debug)
  robotSerial.begin(115200); // initialize a software Serial channel for robot and computer (control)

  arm.begin(robotSerial); // initialize the robot onto the software serial channel; be sure to place the data pin in port 11
  
  arm.setJointSpeed(armID, 256); // set arm joint speed
  arm.setJointSpeed(baseID, 256); // set base joint speed; slower = more torque?
  arm.setJointSpeed(forearmID, 256);
  arm.setJointSpeed(elbowID, 256);
}

// control --------------------------------------------------------------------------------------------------------------------

void loop() {
  
  arm.LED(baseID, &rgb[2]); // base servo motor = blue
  arm.LED(armID,  &rgb[1]); // arm servo motor(s) = green
  arm.LED(elbowID,  &rgb[4]); // rotating servo motor(s) = purple
  arm.LED(forearmID, &rgb[3]); // forearm servo motor = yellow

  potPos = analogRead(pot); // values from 0 to 1023; used to control the orientation of the arm
  
  flexPos = analogRead(flex); // values from approximately 700 to 900; used to control the flex of the arm; must be transformed to be useable on the arm
  flexPos = constrain(flexPos, 700, 900); // control for extraneous readings (x < 700 or x > 900)
  armPos = map(flexPos, 700, 900, 180, 520); // map readings to servo positions s.t. the full range of the flex sensor controls the full range of the servo
  forearmPos = map(flexPos, 700, 900, 520, 180); // map readings to servo positions s.t. the full range of the flex sensor controls the full range of the servo
  
  if (timeMillis % 500 == 0) { // debug; prevents debug stream from being flooded and unreadable
    Serial.print("flex position: "); // return flex value
    Serial.println(flexPos);
    Serial.print ("pot position: "); // return pot value
    Serial.println(potPos);
   }
   
  timeMillis++; // debug; increment counter
  
  arm.moveJoint(baseID, potPos); // set base joint position to pot position
  arm.moveJoint(armID, armPos); // set arm joint position to flex position
  arm.moveJoint(forearmID, armPos); // set arm joint position to flex position  
  arm.moveJoint(elbowID, 512);
}
