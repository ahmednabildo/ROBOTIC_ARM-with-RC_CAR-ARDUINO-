#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

// Board: Arduino Mega 2560 (pins 47-54 do not exist on an Uno)

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
const int timeSlice = 3000;      // ms per time unit
const int maxPositions = 50;     // size of the stored arrays

// Variables
double x = 10.0;
double y = 10.0;
double theta1, theta2, phi, z;
long stepper1Position, stepper2Position, stepper3Position, stepper4Position;
String content = "";
int data[11];

// long (not int): int is only 16 bit on the Mega, so e.g. 11 * 3000 overflows
long theta1Array[maxPositions];
long theta2Array[maxPositions];
long phiArray[maxPositions];
long zArray[maxPositions];
int gripperArray[maxPositions];
int directionArray[maxPositions];
long delays[maxPositions];
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
void moveToStep(int i);
bool steppersRunning();
void runSteppers();
void moveSteppersToPosition();
void setStepperSpeeds(int speed);
void updateSerialData();
void controlRCCar(int direction, long duration);
void driveWheels(int lb, int lf, int rb, int rf, int speed, long duration);
void stopCar();

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100); // default is 1000 ms, which made every command wait 1 s
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
  for (int i = 0; i < maxPositions; i++) {
    delays[i] = 0;
  }
}

void homing() {
  // Homing Stepper4
  stepper4.setSpeed(1500);
  while (digitalRead(limitSwitch4) != 0) {
    stepper4.runSpeed();
  }
  stepper4.setCurrentPosition(0); // limit switch pressed -> this is position 0
  delay(20);

  // Homing Stepper3
  stepper3.setSpeed(-500);
  while (digitalRead(limitSwitch3) != 0) {
    stepper3.runSpeed();
  }
  stepper3.setCurrentPosition(-800); // limit switch pressed -> this is position -800
  delay(20);

  // Homing Stepper2
  stepper2.setSpeed(500);
  while (digitalRead(limitSwitch2) != 1) {
    stepper2.runSpeed();
  }
  stepper2.setCurrentPosition(800); // limit switch pressed -> this is position 800
  delay(20);
  stepper2.moveTo(100);
  while (stepper2.distanceToGo() != 0) { // was: currentPosition() != 200 (never true -> hang)
    stepper2.run();
  }

  // Homing Stepper1
  stepper1.setSpeed(500);
  while (digitalRead(limitSwitch1) != 1) {
    stepper1.runSpeed();
  }
  stepper1.setCurrentPosition(800); // limit switch pressed -> this is position 800
  delay(20);
  stepper1.moveTo(100);
  while (stepper1.distanceToGo() != 0) { // was: currentPosition() != 200 (never true -> hang)
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
  // Do not write past the end of the arrays
  if (positionsCounter >= maxPositions) {
    return;
  }
  theta1Array[positionsCounter] = data[2] * theta1AngleToSteps;
  theta2Array[positionsCounter] = data[3] * theta2AngleToSteps;
  phiArray[positionsCounter] = data[4] * phiAngleToSteps;
  zArray[positionsCounter] = data[5] * zDistanceToSteps;
  gripperArray[positionsCounter] = data[6];
  delays[positionsCounter] = (long)data[8] * timeSlice; // long math, int would overflow
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

    setStepperSpeeds(data[7]);
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

    controlRCCar(directionArray[i], delays[i]);
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

// ---------------- RC car (mecanum wheels) ----------------
// Each wheel gets +1 (forward), -1 (backward) or 0 (stopped).
// If one wheel turns the wrong way, flip it with setPinsInverted(true, false, false)
// in initializeSteppers() (typical for the right-side motors).
void controlRCCar(int direction, long duration) {
  int lb = 0; // left back
  int lf = 0; // left front
  int rb = 0; // right back
  int rf = 0; // right front

  switch (direction) {
    case 1: // forward
      lb = 1;  lf = 1;  rb = 1;  rf = 1;
      break;
    case 2: // backward
      lb = -1; lf = -1; rb = -1; rf = -1;
      break;
    case 3: // strafe left
      lb = 1;  lf = -1; rb = -1; rf = 1;
      break;
    case 4: // strafe right
      lb = -1; lf = 1;  rb = 1;  rf = -1;
      break;
    case 5: // diagonal left-forward
      lb = 1;  lf = 0;  rb = 0;  rf = 1;
      break;
    case 6: // diagonal right-forward
      lb = 0;  lf = 1;  rb = 1;  rf = 0;
      break;
    case 7: // diagonal left-backward
      lb = 0;  lf = -1; rb = -1; rf = 0;
      break;
    case 8: // diagonal right-backward
      lb = -1; lf = 0;  rb = 0;  rf = -1;
      break;
    default: // 0 or unknown -> stay still
      stopCar();
      return;
  }

  driveWheels(lb, lf, rb, rf, data[7], duration);
}

// Runs the wheels at the given speed for 'duration' ms, then stops them.
// runSpeed() makes at most ONE step per call, so it has to be called in a loop.
void driveWheels(int lb, int lf, int rb, int rf, int speed, long duration) {
  leftBackWheel.setSpeed(lb * speed);
  leftFrontWheel.setSpeed(lf * speed);
  rightBackWheel.setSpeed(rb * speed);
  rightFrontWheel.setSpeed(rf * speed);

  unsigned long startTime = millis();
  while (millis() - startTime < (unsigned long)duration) {
    leftBackWheel.runSpeed();
    leftFrontWheel.runSpeed();
    rightBackWheel.runSpeed();
    rightFrontWheel.runSpeed();
  }

  stopCar();
}

void stopCar() {
  leftBackWheel.setSpeed(0);
  leftFrontWheel.setSpeed(0);
  rightBackWheel.setSpeed(0);
  rightFrontWheel.setSpeed(0);
}

// ---------------- Arm helpers ----------------
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

// run() ignores setSpeed(); it follows setMaxSpeed() + acceleration.
// A max speed of 0 would make the motors never move (and the while loops never end),
// so a speed <= 0 keeps the previous max speed.
void setStepperSpeeds(int speed) {
  if (speed <= 0) {
    return;
  }
  if (speed > 4000) {
    speed = 4000;
  }
  stepper1.setMaxSpeed(speed);
  stepper2.setMaxSpeed(speed);
  stepper3.setMaxSpeed(speed);
  stepper4.setMaxSpeed(speed);
}

// Only read when a new command has actually arrived.
// Before, the buffer was flushed first, so readString() timed out on an empty string,
// every data[] became 0 and playback stopped after the first step.
void updateSerialData() {
  if (Serial.available() > 0) {
    readSerialData();
  }
}
