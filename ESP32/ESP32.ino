#include "canmotorlib.h"
#define INTERVAL 5000
#define MAX_BUF_SIZE 256
#define debug true

int dataIndex = 0;
char receivedData[MAX_BUF_SIZE];

int value = 0;
int key = 0;
int state = 0;

unsigned long previousMillisDebug = 0;
const int intervalDebug = 5;

long lastChangeTime = 0;
canmotorlib motor(1, "AK80_6_V1");

float *motorData;
const float kp = 18.5;
const float kd = 0.5;
float torueLimit = 0.7;
const int IgnoreTorque = 0;
int countIgnoreTorque = 0;

float targetPosition = 0;
float currentRegulatorPosition = 0;
float step = 0;

float openPosition = 0;
float closePosition = 500;
float previousTorque = 0;
int previousState = 0;

bool calibration = true;
bool savePosition = false;

String receivedString = "";
int stateGripper = 1;

void setup() {
  Serial.begin(115200);
  delay(8000);
  motor.disable_motor();

  while (!motor.getMotorInit()) {
    delay(1000);
    motorData = motor.enable_motor();
  }

  currentRegulatorPosition = motor.radians_to_degrees(motorData[0]);
  delay(1000);

  motorData = motor.send_deg_command(currentRegulatorPosition, 0, kp, kd, 0);
  delay(2000);
  motor.setVelocity(1.0);
  _calibrationTool();
}

void loop() {
  unsigned long currentMillisDebug = millis();
  unsigned long currentMillis = millis();

  dataIndex = 0;
  previousState = state;
  readSerialData();

  if (previousState != state) {
    savePosition = false;
    countIgnoreTorque = 0;
    if (state == 0) {
      stateGripper = 0;
      targetPosition = openPosition;
    }
    else {
      stateGripper = 1;
      targetPosition = closePosition;
    }

    step = 0.8 * step;
  }

  if (stateGripper == 1) {
    if (abs(motorData[2]) < torueLimit || savePosition == false) {
      step = motor.getStep(currentRegulatorPosition, targetPosition);
      currentRegulatorPosition += step;

      if (abs(motor.radians_to_degrees(motorData[1])) > 30) {
        if (abs(motorData[2]) > previousTorque) {
          countIgnoreTorque++;
          if (countIgnoreTorque > IgnoreTorque) {
            savePosition = true;
          }
        }
      }
    }
    else {
      savePosition = true;
      motor.setCurrentStep(0);
    }
  }
  else {
    if (abs(motorData[2]) < torueLimit && key == 1 || savePosition == false) {
      step = motor.getStep(currentRegulatorPosition, targetPosition);
      currentRegulatorPosition += step;

      if (abs(motor.radians_to_degrees(motorData[1])) > 30) {
        if (abs(motorData[2]) > previousTorque) {
          countIgnoreTorque++;
          if (countIgnoreTorque > IgnoreTorque) {
            savePosition = true;
          }
        }
      }
    }
    else {
      savePosition = true;
      motor.setCurrentStep(0);
    }
  }

  previousTorque = abs(motorData[2]);
  motorData = motor.send_deg_command(currentRegulatorPosition, 10, kp, kd, 0);

  if (debug) {
    if (currentMillisDebug - previousMillisDebug >= intervalDebug) {
      previousMillisDebug = currentMillisDebug;
      Serial.print("\tTarget: ");
      Serial.print(targetPosition);
      Serial.print("\tCurrent: ");
      Serial.print(currentRegulatorPosition);
      Serial.print("\tTorque: ");
      Serial.print(motorData[2]);
      Serial.print("\tMotor speed: ");
      Serial.println(motor.radians_to_degrees(motorData[1]));
      Serial.print("\tMotor data: ");
      Serial.println(motor.radians_to_degrees(motorData[0]));
    }
  }

  delay(10);
}

void _calibrationTool() {
  targetPosition = -800;
  float torueLimitCalibration = 0.6;

  while (calibration) {
    dataIndex = 0;
    readSerialData();

    if (key == 8 || abs(motorData[2]) > torueLimitCalibration) {
      openPosition += motor.radians_to_degrees(motorData[0]);
      closePosition += motor.radians_to_degrees(motorData[0]);
      calibration = false;
      savePosition = true;
      targetPosition = openPosition;
      motor.setCurrentStep(0);
      motor.setVelocity(4.0);
    }
    else {
      step = motor.getStep(currentRegulatorPosition, targetPosition);
      currentRegulatorPosition += step;
    }

    unsigned long currentMillisDebug = millis();

    motorData = motor.send_deg_command(currentRegulatorPosition, 10, kp, kd, 0);
    if (debug) {
      if (currentMillisDebug - previousMillisDebug >= intervalDebug) {
        previousMillisDebug = currentMillisDebug;
        Serial.print("\tTarget: ");
        Serial.print(targetPosition);
        Serial.print("\tCurrent: ");
        Serial.print(currentRegulatorPosition);
        Serial.print("\tTorque: ");
        Serial.print(motorData[2]);
        Serial.print("\tMotor speed: ");
        Serial.println(motor.radians_to_degrees(motorData[1]));
        Serial.print("\tMotor data: ");
        Serial.println(motor.radians_to_degrees(motorData[0]));
      }
    }
    delay(10);
  }

}

bool readSerialData() {
  bool readData = false;
  while (Serial.available() > 0 && dataIndex < MAX_BUF_SIZE - 1) {
    char incomingChar = Serial.read();
    receivedData[dataIndex] = incomingChar;
    dataIndex++;
    delay(2);
  }

  receivedData[dataIndex] = '\0';

  if (dataIndex > 0) {
    value = 0;
    for (int i = 0; i < dataIndex - 1; i++) {
      value = value * 10 + (receivedData[i] - 48);
    }

    key = value / 10;
    state = value % 10;
    readData = true;
  }
  return readData;
}
