// ROBOT ARM CONTROL
// IVAN PATINO, JOHN SO, KENNETH THAI, TIFFANY TRUONG
// UPDATED 071718
// THIS CODE IMPLEMENTS CONTROL FOR A SERIES OF DYNAMIXEL XL-320 MOTORS USING AN ESP-WROOM-32 MICROCONTROLLER.
// CREDITS TO https://github.com/hackerspace-adelaide/XL320 AND https://github.com/espressif/arduino-esp32 FOR RELEVANT LIBRARIES

// This code visualizes the robot arm as a 3-dimensional vector and maps the arm on a 3-d coordinate plane. The base motors control the angling (direction),
// while the arm/forearm members control the length of the arm (magnitude).

// includes --------------------

#include <XL320.h>
#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <Math.h>

// constants --------------------

String ID = "ESP32ARM";

double ARM_LENGTH = 78.65; // mm
double FOREARM_LENGTH = 51.65; // mm
double TOTAL_LENGTH = ARM_LENGTH + FOREARM_LENGTH; // 130.30 mm
double MIN_LENGTH = 59.6; // mm

int BASE_MID = 500;
int FOREARM_MID = 520;
int WRIST_MID = 520;

double LENGTH_RANGE[2] = {MIN_LENGTH, TOTAL_LENGTH};
int BASE_RANGE[2] = {250, 750}; // left to 5
int FOREARM_RANGE[2] = {150, 860}; // down to up
int ARM_RANGE[2] = {220, 500}; //  0 deg, 90 deg
int WRIST_RANGE[2] = {190, 820}; // up to down

int X_RANGE[2] = { -10, 10};
int Y_RANGE[2] = {0, 10};
int Z_RANGE[2] = {0, 10};
double MAP_LENGTH_MAX = 17.3;
double MAP_LENGTH_MIN = 0;
double RANGE_MIN = MIN_LENGTH / TOTAL_LENGTH * MAP_LENGTH_MAX; // 3.961
double MAP_RANGE[2] = {RANGE_MIN, MAP_LENGTH_MAX};
int YZ_DEGREES[2] = {0, 90};
int X_DEGREES[2] = {0, 180};
int FOREARM_DEGREES[2] = {90, 270};
int WRIST_DEGREES[2] = {270, 90};

int potPos; // debug

// initialization --------------------

BluetoothSerial SerialBT; // BT communication channel for teleoperated control
HardwareSerial robotSerial(1); // extra communication channel for robot control
XL320 arm; // robot declaration

int robotTX = 23; // hardwareSerial pins
int robotRX = 22;

char rgb[] = "rgbypcwo"; // color array for servo LED IDs

//XL320 servo motor IDs; unique and allows for independent control
int baseID = 2;
int armID = 1;
int forearmID = 4;
int wristID = 3;
int clawID = 5;

double movement[4] = {1. * BASE_MID, 1. * 500, 1. * FOREARM_MID, 1. * WRIST_MID}; // base degrees, arm degrees, forearm degrees, wrist degrees

// stores components of vector
double x_pos, y_pos, z_pos;

//store angles from axes (derived from direction cosines) and magnitude of vector
double dirX, dirY, dirZ, mag;

// function declaration --------------------

double calculateMag(int x, int y, int z) {
  return (pow(pow(x, 2) + pow(y, 2) + pow(z, 2), .5));
}

double calculateMag(int x, int y) {
  return (pow(pow(x, 2) + pow(y, 2), .5));
}

// used to calculate angle from the x-axis; derived from direction cosines of vector xi + yj + zk
double calculateDirX(double x, double y, double z) {
  return (acos(x / calculateMag(x, y, z)));
}

double calculateDirX(double x, double y) {
  return (acos(x / calculateMag(x, y)));
}

// used to calculate angle from the y-axis; derived from direction cosines of vector xi + yj + zk
double calculateDirY(double x, double y, double z) {
  return (acos(y / calculateMag(x, y, z)));
}

// used to calculate angle from the z-axis; derived from direction cosines of vector xi + yj + zk
double calculateDirZ(double x, double y, double z) {
  return (acos(z / calculateMag(x, y, z)));
}

//move all joints of the arm at once
void moveArm(double pos[4]) {
  arm.moveJoint(baseID, pos[0]);
  delay(50);
  arm.moveJoint(armID, pos[1]);
  delay(50);
  arm.moveJoint(forearmID, pos[2]);
  delay(50);
  arm.moveJoint(wristID, pos[3]);
  delay(50);
}

// used to transform data to be compatible with arm (arm radii, angling, etc)
double transformData(double value, int old_data[], int new_data[]) {
  value = constrain(value, old_data[0], old_data[1]);
  value = map(value, old_data[0], old_data[1], new_data[0], new_data[1]);
  return value;
}

int transformData(int value, int old_data[], int new_data[]) {
  value = constrain(value, old_data[0], old_data[1]);
  value = map(value, old_data[0], old_data[1], new_data[0], new_data[1]);
  return value;
}

double transformData(double value, double old_data[], double new_data[]) {
  value = constrain(value, old_data[0], old_data[1]);
  value = map(value, old_data[0], old_data[1], new_data[0], new_data[1]);
  return value;
}

// solve the required degrees for each joint of the arm given the magnitude of the arm vector
void trigSolver(double magnitude, double answers[]) {
  
  double armAngles[3]; // arm angle, forearm angle, wrist angle
  
  // in law of Cosines, long leg C is the magnitude of the vector
  // b and a are the lengths of the arms (defined in constants)
  // we need to find the measure of C in order to adjust the length of the arm
  // lastly, the measures of A and B can be found using the law of Sines

  // solving for big angle in the form c2 = a2 + b2 - 2abcosC or C = arccos(-(c2 - a2 - b2)/(2ab))

  //Serial.println(magnitude);
  magnitude = transformData(magnitude, MAP_RANGE, LENGTH_RANGE);
  Serial.println(magnitude);

  armAngles[0] = acos(-(pow(magnitude, 2) - pow(ARM_LENGTH, 2) - pow(FOREARM_LENGTH, 2)) / (2 * ARM_LENGTH * FOREARM_LENGTH)); // forearm angle
  armAngles[2] = asin(sin(armAngles[0]) * FOREARM_LENGTH / magnitude) * RAD_TO_DEG; // arm angle
  armAngles[1] = asin(sin(armAngles[0]) * ARM_LENGTH / magnitude) * RAD_TO_DEG; // wrist angle
  armAngles[0] *= RAD_TO_DEG;

  answers[1] = armAngles[1];
  answers[2] = armAngles[0];
  answers[3] = 180 + armAngles[2];
}

//boolean pythagoreanCheck() {}

// setup --------------------

void setup() {

  pinMode(34, INPUT);

  //initialize communication channels
  SerialBT.begin(ID); // for wireless communication
  Serial.begin(57600); // for wired debugging
  robotSerial.begin(115200, SERIAL_8N1, robotRX, robotTX); // for robot communication
  arm.begin(robotSerial); // initialize arm onto serial channel

  arm.setJointTorque(254, 1023); // set all motors to highest torque settings
  arm.setJointSpeed(254, 368);
}

// control

void loop() {

  // mapping robot to coordinate plane before beginning communication
  Serial.println();
  Serial.println("Initializing...");
  delay(100);
  
  //potPos = analogRead(34) / 4;
  arm.LED(baseID, &rgb[0]); // base = red
  arm.LED(armID, &rgb[1]); // arm = green
  arm.LED(forearmID, &rgb[2]); // forearm = blue
  arm.LED(wristID, &rgb[3]); // wrist = yellow
  arm.LED(clawID, &rgb[4]); // claw = cyan
  delay(100);
  moveArm(movement);
  //arm.moveJoint(clawID, potPos);
  //Serial.println(potPos);
  
  // begin UI
  Serial.println();
  Serial.println("This demo of the robot arm will point the arm in the direction that you command. ");
  Serial.println("Enter a series of coordinates (x,y,z) to point the arm in the specified direction.");
  Serial.println("-5 < x < 5, 0 < y < 5, 0 < z < 5");
  Serial.println();

  while (true) { // endless loop ensures that above messages will only send once
    Serial.print("X: ");
    while (!Serial.available()) {}
    x_pos = Serial.parseFloat();
    Serial.println(x_pos);

    Serial.print("Y: ");
    while (!Serial.available()) {}
    y_pos = Serial.parseFloat();
    Serial.println(y_pos);

    Serial.print("Z: ");
    while (!Serial.available()) {}
    z_pos = Serial.parseFloat();
    Serial.println(z_pos);

    Serial.println();

    dirX = calculateDirX(x_pos, y_pos, z_pos) * RAD_TO_DEG;
    dirY = calculateDirY(x_pos, y_pos, z_pos) * RAD_TO_DEG;
    dirZ = calculateDirZ(x_pos, y_pos, z_pos) * RAD_TO_DEG;
    mag = calculateMag(x_pos, y_pos, z_pos);

    // the mathematical definition of 3d vector angles and magnitude
    Serial.print("Magnitude: ");
    Serial.println(mag);

    Serial.println("Direction Angles: ");
    Serial.print("X-axis: ");
    Serial.println(dirX);
    Serial.print("Y-axis: ");
    Serial.println(dirY);
    Serial.print("Z-axis: ");
    Serial.println(dirZ);
    Serial.println();

    // now being translated into something that the robot can work with in a 3D environment
    dirX = calculateDirX(x_pos, y_pos) * RAD_TO_DEG;
    dirZ = 90 - (calculateDirZ(x_pos, y_pos, z_pos) * RAD_TO_DEG);

    Serial.print("Moving arm to ");
    Serial.print(dirX);
    Serial.println(" degrees from the X axis and ");
    Serial.print(dirZ);
    Serial.println(" degrees from the XY plane...");
    Serial.println();

    // translate from degrees to motor steps for base as it's not involved in magnitude calculations
    dirX = transformData(dirX, X_DEGREES, BASE_RANGE);
    
    trigSolver(mag, movement);

    Serial.print("Arm angle: ");
    Serial.println(movement[1]);
    Serial.print("Forearm angle: ");
    Serial.println(movement[2]);
    Serial.print("Wrist angle: ");
    Serial.println(movement[3]);
    
    movement[0] = dirX;
    movement[1] += dirZ;

    // now that all of the data is here, we can translate from degrees to motor steps
    movement[1] = map(movement[1], YZ_DEGREES[0], YZ_DEGREES[1], ARM_RANGE[0], ARM_RANGE[1]);// arm angle
    movement[2] = map(movement[2], FOREARM_DEGREES[0], FOREARM_DEGREES[1], FOREARM_RANGE[0], FOREARM_RANGE[1]);// forearm angle
    movement[3] = map(movement[3], WRIST_DEGREES[0], WRIST_DEGREES[1], WRIST_RANGE[0], WRIST_RANGE[1]);// wrist angle

    movement[2] = constrain(movement[2], FOREARM_RANGE[0], FOREARM_RANGE[1]);// forearm angle
    movement[3] = constrain(movement[3], WRIST_RANGE[0], WRIST_RANGE[1]);// wrist angle
    
    Serial.print("Arm angle: ");
    Serial.println(movement[1]);
    Serial.print("Forearm angle: ");
    Serial.println(movement[2]);
    Serial.print("Wrist angle: ");
    Serial.println(movement[3]);
    
    moveArm(movement); // finally, move the arm!!!!\

    // reset array for next cycle
    movement[0] = BASE_MID;
    movement[1] = 500;
    movement[2] = FOREARM_MID;
    movement[3] = WRIST_MID;
    
    Serial.println("_________________________________________");
    Serial.println();
  } // */
}
