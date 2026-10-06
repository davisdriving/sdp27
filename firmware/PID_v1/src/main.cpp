/*
 * SDP27 — Wheeled Bipedal Robot
 * UMass Amherst — Senior Design Project
 *
 * Controller: ESP32-S3-WROOM-1-N16R8
 * IMU: Adafruit BNO085 — SPI
 *
 * BNO085 connections:
 *   VIN  -> ESP32 3.3V
 *   GND  -> ESP32 GND
 *   CS   -> GPIO10
 *   DI   -> GPIO11 (MOSI)
 *   SCL  -> GPIO12 (SCK)
 *   SDA  -> GPIO13 (MISO)
 *   INT  -> GPIO9
 *   RST  -> GPIO5
 *   P0   -> 3.3V
 *   P1   -> 3.3V
 * 
 * Rotation and gyro reports arrive separately. Cache the latest of each,
 * then run control at up to 100 Hz while both readings are recent.
 * The existing balance angle is roll (sensor X axis), in degrees.
 * Gyro X approximates its derivative for motion about the balance axis.
 * D = -Kd * gyroX_deg_s; no numerical angle differentiation is used.
 */

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <SPI.h>
#include <sh2.h>
#include "utils.h"
#include <math.h>
#include <stdlib.h>

#define BNO08X_RESET 5  // SPI pins
#define BNO08X_INT 9
#define BNO08X_CS 10
#define BNO08X_MOSI 11
#define BNO08X_SCK 12
#define BNO08X_MISO 13

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

bool haveRotationVector = false;
bool haveGyroscope = false;

RotationVector rotationVector;
Gyroscope gyroscope;

float roll = 0.0f;
double rollRateDegS = 0.0;

// PID gains: PWM/deg, PWM/(deg*s), PWM*s/deg, respectively.
double Kp = 60.0, Ki = 30.0, Kd = 10.0;
double Setpoint = -2.8, Input = 0.0, Output = 0.0;
const uint32_t control_period_us = 10000UL; // Target 100 Hz.
const uint32_t imu_timeout_us = 50000UL;
const double outputLimit = 255.0;
double integralTerm = 0.0;
uint32_t lastControlUs = 0;
uint32_t lastRotationUs = 0;
uint32_t lastGyroUs = 0;
bool controlStarted = false;

// Assumes sensor X is the balance axis, as in the original roll formula.
// Gyro X should be positive when the measured roll is increasing.
// For substantial multi-axis motion, body gyro X is not exactly Euler roll rate.
const double gyroSign = 1.0;

static double limitPWM(double value) {
  if (value > outputLimit) return outputLimit;
  if (value < -outputLimit) return -outputLimit;
  return value;
}

static void resetController() {
  Output = 0.0;
  integralTerm = 0.0;
  controlStarted = false;
  haveRotationVector = false;
  haveGyroscope = false;
  runMotor(0);
}

// Configure here so the existing utils.cpp / utils.h need no edits.
static bool enableControlReports() {
  const bool rotationOK = bno08x.enableReport(
      SH2_GAME_ROTATION_VECTOR, control_period_us);
  const bool gyroOK = bno08x.enableReport(
      SH2_GYROSCOPE_CALIBRATED, control_period_us);
  if (!rotationOK) Serial.println(F("Could not enable game rotation vector"));
  if (!gyroOK) Serial.println(F("Could not enable calibrated gyroscope"));
  return rotationOK && gyroOK;
}


void readSerialTuning() {
  static char command[32];
  static size_t commandLength = 0;
  static bool commandOverflow = false;

  // Limit serial work per pass so incoming data cannot monopolize loop().
  for (size_t processed = 0;
       processed < sizeof(command) && Serial.available() > 0;
       ++processed) {
    char character = Serial.read();
    if (character == '\r') continue;

    if (character == '\n') {
      if (commandOverflow) {
        Serial.println(F("Command too long: use p=x, i=x, d=x, or s=x"));
        commandLength = 0;
        commandOverflow = false;
        continue;
      }

      command[commandLength] = '\0';
      char *cursor = command;
      while (*cursor == ' ' || *cursor == '\t') cursor++;
      char parameter = *cursor;
      bool valid = parameter != '\0';
      if (valid) cursor++;

      while (*cursor == ' ' || *cursor == '\t') cursor++;
      if (*cursor != '=') valid = false;
      else cursor++;

      while (*cursor == ' ' || *cursor == '\t') cursor++;
      char *valueEnd;
      double value = strtod(cursor, &valueEnd);
      if (valueEnd == cursor || !isfinite(value)) valid = false;
      cursor = valueEnd;
      while (*cursor == ' ' || *cursor == '\t') cursor++;
      if (*cursor != '\0') valid = false;

      if (parameter != 'p' && parameter != 'P' &&
          parameter != 'i' && parameter != 'I' &&
          parameter != 'd' && parameter != 'D' &&
          parameter != 's' && parameter != 'S') valid = false;

      // Keep the same nonnegative-gain tuning convention.
      if (parameter != 's' && parameter != 'S' && value < 0.0) valid = false;

      if (valid) {
        switch (parameter) {
          case 'p': case 'P': Kp = value; break;
          case 'i': case 'I': Ki = value; break;
          case 'd': case 'D': Kd = value; break;
          case 's': case 'S': Setpoint = value; break;
        }

        // Disabling I also removes any previously accumulated I output.
        if (Ki == 0.0) integralTerm = 0.0;
        Serial.print(F("Set "));
        Serial.print(parameter);
        Serial.print('=');
        Serial.println(value, 4);
      } else if (commandLength > 0) {
        Serial.println(F("Invalid: use p=x, i=x, d=x, s=x; gains must be >= 0"));
      }
      commandLength = 0;
    } else if (commandOverflow) {
      continue; // Discard the entire overlong line, including its tail.
    } else if (commandLength < sizeof(command) - 1) {
      command[commandLength++] = character;
    } else {
      commandOverflow = true;
      commandLength = 0;
    }
  }
}

void setup(void) {
  Serial.begin(115200);

  SPI.begin(BNO08X_SCK, BNO08X_MISO, BNO08X_MOSI, BNO08X_CS); 

  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  initMotors();

  Setpoint = 0;

  Serial.println(F("Serial tuning: p=x, i=x, d=x, s=x; end each command with a newline."));

  Serial.println("Balancing robot starting...");
  for (int i = 5; i > 0; i--) {
    Serial.print("Starting in ");
    Serial.print(i);
    Serial.println(" seconds...");
    delay(1000);
  }

  // Start reports after the countdown, avoiding queued startup measurements.
  if (!enableControlReports()) {
    runMotor(0);
    while (1) { delay(10); }
  }
  resetController();
}

void loop() {
  readSerialTuning();

  if (bno08x.wasReset()) {
    resetController();
    if (!enableControlReports()) {
      while (1) { delay(10); }
    }
    return;
  }

  // Each call returns one report. Neither report alone starts the controller.
  if (bno08x.getSensorEvent(&sensorValue)) {
    const uint32_t receivedUs = micros();
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      rotationVector.i = sensorValue.un.gameRotationVector.i;
      rotationVector.j = sensorValue.un.gameRotationVector.j;
      rotationVector.k = sensorValue.un.gameRotationVector.k;
      rotationVector.real = sensorValue.un.gameRotationVector.real;
      rotationVector.accuracy = sensorValue.status & 0x03;
      roll = atan2f(
          2.0f * (rotationVector.real * rotationVector.i
                  + rotationVector.j * rotationVector.k),
          1.0f - 2.0f * (rotationVector.i * rotationVector.i
                        + rotationVector.j * rotationVector.j)) * 180.0f / PI;
      if (!isfinite(roll)) {
        resetController();
        return;
      }
      haveRotationVector = true;
      lastRotationUs = receivedUs;
    } else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      // BNO085 calibrated gyro is in rad/s; angle and gains use degrees.
      rollRateDegS = gyroSign * sensorValue.un.gyroscope.x * 180.0 / PI;
      if (!isfinite(rollRateDegS)) {
        resetController();
        return;
      }
      haveGyroscope = true;
      lastGyroUs = receivedUs;
    }
  }

  const uint32_t now = micros();
  if (!haveRotationVector || !haveGyroscope) return;
  if ((uint32_t)(now - lastRotationUs) > imu_timeout_us ||
      (uint32_t)(now - lastGyroUs) > imu_timeout_us) {
    resetController();
    return;
  }

  const uint32_t elapsedUs = (uint32_t)(now - lastControlUs);
  if (controlStarted && elapsedUs < control_period_us) return;
  const double dt = controlStarted ? elapsedUs * 1.0e-6
                                   : control_period_us * 1.0e-6;
  lastControlUs = now;
  controlStarted = true;

  Input = roll;
  const double error = Setpoint - Input;
  const double pTerm = Kp * error;
  // Derivative on measurement: increasing roll gives a negative D term.
  // Do NOT divide gyro rate by dt; it is already an angular velocity.
  // A setpoint change therefore does not cause a derivative kick.
  const double dTerm = -Kd * rollRateDegS;

  // Conditional integration: do not wind I further into output saturation.
  if (Ki == 0.0) {
    integralTerm = 0.0;
  } else {
    const double candidateI = limitPWM(integralTerm + Ki * error * dt);
    const double candidateOutput = pTerm + candidateI + dTerm;
    if ((candidateOutput >= -outputLimit && candidateOutput <= outputLimit) ||
        (candidateOutput > outputLimit && error < 0.0) ||
        (candidateOutput < -outputLimit && error > 0.0)) {
      integralTerm = candidateI;
    }
  }
  Output = limitPWM(pTerm + integralTerm + dTerm);
  runMotor(-Output); // Preserve the original motor polarity.

  static uint32_t lastPrintUs = 0;
  if ((uint32_t)(now - lastPrintUs) >= 200000UL && Serial.availableForWrite() >= 96) {
    lastPrintUs = now;
    Serial.print(F("Roll: "));
    Serial.print(Input, 1);
    Serial.print(F(" deg, rate: "));
    Serial.print(rollRateDegS, 1);
    Serial.print(F(" deg/s, D: "));
    Serial.print(dTerm, 1);
    Serial.print(F(", PWM command: "));
    Serial.println(Output, 1);
  }
}
