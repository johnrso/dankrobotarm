// ARM CONTROL V1
// JOHN SO, 071018
// Implements control for a series of Dynamixel XL-320 motors using a potentionmeter and flex sensor. 
// Credits to https://github.com/hackerspace-adelaide/XL320 for software library and info on wiring and use.

#include "XL320.h"
#include <SoftwareSerial.h>

XL320 arm; // robot arm declaration
SoftwareSerial robotSerial(10, 11); // (RX, TX)

int flex = A0;
int pot = A1;
int flexPos;
int potPos;
int motorPos;

char rgb[] = "rgbypcwo";
int armID = 1;
int baseID = 2;

int timeMillis = 0;

void setup() {
  pinMode(flex, INPUT); // initialize pins;
  pinMode(pot, INPUT);

  Serial.begin(9600); // initialize Serial communication between Arduino and computer (debug)
  robotSerial.begin(115200); // initialize a software Serial channel for robot and computer (control)

  arm.begin(robotSerial); // initialize the robot onto the software serial channel
  arm.setJointSpeed(armID, 768); // set arm joint speed
  arm.setJointSpeed(baseID, 512); // set joint speed
}

void loop() {
  arm.LED(baseID, &rgb[2]);
  arm.LED(armID,  &rgb[1]);
  flexPos = analogRead(flex); // values from approximately 700 to 900; used to control the flex of the arm
  potPos = analogRead(pot); // values from 0 to 1023; used to control the orientation of the arm

  flexPos = constrain(flexPos, 700, 900); // control for extraneous readings (700 < x < 900)
  flexPos = map(flexPos, 700, 900, 0, 1023); // map values to servo positions s.t. the full range of the flex sensor controls the full range of the arm

  
  if (timeMillis % 500 == 0) { // debug
    Serial.print("flex position: ");
    Serial.println(flexPos);
    Serial.print ("pot position: ");
    Serial.println(potPos);
   }

  arm.moveJoint(baseID, potPos); // set base joint position to pot position
  arm.moveJoint(armID, flexPos); // set arm joint position to flex position
  
  timeMillis++; // debug
}
