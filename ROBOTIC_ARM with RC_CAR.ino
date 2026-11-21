
#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

// Pin Definitions
#define limitSwitch1 11
#define limitSwitch2 10
#define limitSwitch3 9
#define limitSwitch4 A3

// Stepper Motor Definitions
AccelStepper stepper1(1, 2, 5); // (Type: driver, STEP, DIR)
AccelStepper stepper2(1, 3, 6);
AccelStepper stepper3(1, 4, 7);
AccelStepper stepper4(1, 12, 13);

AccelStepper leftBackWheel(1, 47, 48);
AccelStepper leftFrontWheel(1, 49, 50);
AccelStepper rightBackWheel(1, 51, 52);
AccelStepper rightFrontWheel(1, 53, 54);

// Servo Definition
Servo gripperServo;

// Constants
const double L1 = 228.0; // mm
const double L2 = 136.5; // mm
const float theta1AngleToSteps = 44.444444;
const float theta2AngleToSteps = 35.555555;
const float phiAngleToSteps = 10;
const float zDistanceToSteps = 100;
const int timeSlice = 3000;

// Variables
double x = 10.0;
double y = 10.0;
double theta1, theta2, phi, z;
int stepper1Position, stepper2Position, stepper3Position, stepper4Position;
String content = "";
int data[11];
int theta1Array[50];
int theta2Array[50];
int phiArray[50];
int zArray[50];
int gripperArray[50];
int directionArray[50];
int delays[50];
int positionsCounter = 0;

// Function Prototypes
void initializePins();
void initializeSteppers();
void initializeServo();
void homing();
void readSerialData();
void saveData();
void clearData();
void executeStoredSteps();
void moveToPosition();
void serialFlush();
void controlRCCar(int direction);
void setCarMotorSpeeds(int speed);
void moveForward();
void moveBackward();
void moveLeft();
void moveRight();
void moveLeftForward();
void moveRightForward();
void moveLeftBackward();
void moveRightBackward();
void stopCar();

void setup() {
  Serial.begin(9600);
  initializePins();
  initializeSteppers();
  initializeServo();
  homing();
}

void loop() {
  if (Serial.available()) {
    readSerialData();
    if (data[0] == 1) {
      saveData();
    }
    if (data[0] == 2) {
      clearData();
    }
  }
  if (data[1] == 1) {
    executeStoredSteps();
  } else {
    moveToPosition();
  }
  delay(100);
  gripperServo.write(data[6]);
  delay(300);
}

void initializePins() {
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  pinMode(limitSwitch4, INPUT_PULLUP);
}

void initializeSteppers() {
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);
  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(2000);
  stepper4.setMaxSpeed(4000);
  stepper4.setAcceleration(2000);

  leftBackWheel.setMaxSpeed(4000);
  leftBackWheel.setAcceleration(2000);
  leftFrontWheel.setMaxSpeed(4000);
  leftFrontWheel.setAcceleration(2000);
  rightBackWheel.setMaxSpeed(4000);
  rightBackWheel.setAcceleration(2000);
  rightFrontWheel.setMaxSpeed(4000);
  rightFrontWheel.setAcceleration(2000);
}

void initializeServo() {
  gripperServo.attach(A0, 600, 2500);
  delay(1000);
  for (int i = 0; i < 50; i++) {
    delays[i] = 0;
  }
}

void homing() {
  // Homing Stepper4
  while (digitalRead(limitSwitch4) != 0) {
    stepper4.setSpeed(1500);
    stepper4.runSpeed();
    stepper4.setCurrentPosition(0); // When limit switch pressed set position to 0 steps
  }
  delay(20);
  stepper4.moveTo(0);
  while (stepper4.currentPosition() != 0) {
    stepper4.run();
  }

  // Homing Stepper3
  while (digitalRead(limitSwitch3) != 0) {
    stepper3.setSpeed(-500);
    stepper3.runSpeed();
    stepper3.setCurrentPosition(-800); // When limit switch pressed set position to 0 steps
  }
  delay(20);
  stepper3.moveTo(-800);
  while (stepper3.currentPosition() != -800) {
    stepper3.run();
  }

  // Homing Stepper2
  while (digitalRead(limitSwitch2) != 1) {
    stepper2.setSpeed(500);
    stepper2.runSpeed();
    stepper2.setCurrentPosition(800); // When limit switch pressed set position to -5440 steps
  }
  delay(20);
  stepper2.moveTo(100);
  while (stepper2.currentPosition() != 200) {
    stepper2.run();
  }

  // Homing Stepper1
  while (digitalRead(limitSwitch1) != 1) {
    stepper1.setSpeed(500);
    stepper1.runSpeed();
    stepper1.setCurrentPosition(800); // When limit switch pressed set position to 0 steps
  }
  delay(20);
  stepper1.moveTo(100);
  while (stepper1.currentPosition() != 200) {
    stepper1.run();
  }
}

void readSerialData() {
  content = Serial.readString();
  for (int i = 0; i < 11; i++) {
    int index = content.indexOf(",");
    data[i] = atol(content.substring(0, index).c_str());
    content = content.substring(index + 1);
  }
}

void saveData() {
  theta1Array[positionsCounter] = data[2] * theta1AngleToSteps;
  theta2Array[positionsCounter] = data[3] * theta2AngleToSteps;
  phiArray[positionsCounter] = data[4] * phiAngleToSteps;
  zArray[positionsCounter] = data[5] * zDistanceToSteps;
  gripperArray[positionsCounter] = data[6];
  delays[positionsCounter] = data[8] * timeSlice;
  directionArray[positionsCounter] = data[9];
  positionsCounter++;
}

void clearData() {
  memset(theta1Array, 0, sizeof(theta1Array));
  memset(theta2Array, 0, sizeof(theta2Array));
  memset(phiArray, 0, sizeof(phiArray));
  memset(zArray, 0, sizeof(zArray));
  memset(gripperArray, 0, sizeof(gripperArray));
  memset(delays, 0, sizeof(delays));
  memset(directionArray, 0, sizeof(directionArray));
  positionsCounter = 0;
}

void executeStoredSteps() {
  for (int i = 0; i <= positionsCounter - 1; i++) {
    if (data[1] == 0) break;
    moveToStep(i);
    while (steppersRunning()) {
      runSteppers();
    }
    if (i == 0) {
      gripperServo.write(gripperArray[i]);
    } else if (gripperArray[i] != gripperArray[i - 1]) {
      gripperServo.write(gripperArray[i]);
      delay(800);
    }
    controlRCCar(directionArray[i]);
    delay(500);
    updateSerialData();
    if (data[1] == 0) break;
  }
}

void moveToPosition() {
  stepper1Position = data[2] * theta1AngleToSteps;
  stepper2Position = data[3] * theta2AngleToSteps;
  stepper3Position = data[4] * phiAngleToSteps;
  stepper4Position = data[5] * zDistanceToSteps;

  setStepperSpeeds(data[7]);
  moveSteppersToPosition();
  while (steppersRunning()) {
    runSteppers();
  }
}

void serialFlush() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void controlRCCar(int direction) {
  setCarMotorSpeeds(data[7]);
  switch (direction) {
    case 1: moveForward(); break;
    case 2: moveBackward(); break;
    case 3: moveLeft(); break;
    case 4: moveRight(); break;
    case 5: moveLeftForward(); break;
    case 6: moveRightForward(); break;
    case 7: moveLeftBackward(); break;
    case 8: moveRightBackward(); break;
    default: stopCar(); break;
  }
}

void setCarMotorSpeeds(int speed) {
  leftBackWheel.setSpeed(speed);
  leftFrontWheel.setSpeed(speed);
  rightBackWheel

.setSpeed(speed);
  rightFrontWheel.setSpeed(speed);
}

void moveForward() {
  leftBackWheel.runSpeed();
  leftFrontWheel.runSpeed();
  rightBackWheel.runSpeed();
  rightFrontWheel.runSpeed();
}

void moveBackward() {
  leftBackWheel.runSpeed();
  leftFrontWheel.runSpeed();
  rightBackWheel.runSpeed();
  rightFrontWheel.runSpeed();
}

void moveLeft() {
  leftBackWheel.runSpeed();
  leftFrontWheel.runSpeed();
  rightBackWheel.runSpeed();
  rightFrontWheel.runSpeed();
}

void moveRight() {
  leftBackWheel.runSpeed();
  leftFrontWheel.runSpeed();
  rightBackWheel.runSpeed();
  rightFrontWheel.runSpeed();
}

void moveLeftForward() {
  leftBackWheel.runSpeed();
  leftFrontWheel.runSpeed();
  rightBackWheel.runSpeed();
  rightFrontWheel.runSpeed();
}

void moveRightForward() {
  leftBackWheel.runSpeed();
  leftFrontWheel.runSpeed();
  rightBackWheel.runSpeed();
  rightFrontWheel.runSpeed();
}

void moveLeftBackward() {
  leftBackWheel.runSpeed();
  leftFrontWheel.runSpeed();
  rightBackWheel.runSpeed();
  rightFrontWheel.runSpeed();
}

void moveRightBackward() {
  leftBackWheel.runSpeed();
  leftFrontWheel.runSpeed();
  rightBackWheel.runSpeed();
  rightFrontWheel.runSpeed();
}

void stopCar() {
  leftBackWheel.stop();
  leftFrontWheel.stop();
  rightBackWheel.stop();
  rightFrontWheel.stop();
}

void moveToStep(int i) {
  stepper1.moveTo(theta1Array[i]);
  stepper2.moveTo(theta2Array[i]);
  stepper3.moveTo(phiArray[i]);
  stepper4.moveTo(zArray[i]);
}

bool steppersRunning() {
  return stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0;
}

void runSteppers() {
  stepper1.run();
  stepper2.run();
  stepper3.run();
  stepper4.run();
}

void moveSteppersToPosition() {
  stepper1.moveTo(stepper1Position);
  stepper2.moveTo(stepper2Position);
  stepper3.moveTo(stepper3Position);
  stepper4.moveTo(stepper4Position);
}

void setStepperSpeeds(int speed) {
  stepper1.setSpeed(speed);
  stepper2.setSpeed(speed);
  stepper3.setSpeed(speed);
  stepper4.setSpeed(speed);
}

void updateSerialData() {
  serialFlush();
  content = Serial.readString();
  for (int j = 0; j < 11; j++) {
    int index = content.indexOf(",");
    data[j] = atol(content.substring(0, index).c_str());
    content = content.substring(index + 1);
  }
}