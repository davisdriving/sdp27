#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <SPI.h>
#include <sh2.h>
#include "utils.h"

// ESP32-S3 -> L298N. 
const uint8_t IN1_PIN = 4; 
const uint8_t IN2_PIN = 6; 
const uint8_t ENA_PIN = 7;
const uint8_t IN3_PIN = 17;
const uint8_t IN4_PIN = 16;
const uint8_t ENB_PIN = 15;

static bool motorDirectionKnown = false;
static bool motorForward = false;

void setReports(void) { // define which reports to enable (BNO085 has many different options)
  Serial.println("Setting game rotation vector and gyroscope reports");
  if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
}

void calibrateIMU(void) {
  Serial.println("Calibrating IMU...");

  bno08x.getSensorEvent(&sensorValue);

  int result = sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO);
  if (result != SH2_OK) {
    Serial.print("Could not enable calibration: ");
    Serial.println(result);
  }

  Serial.println("Please rotate the robot in all directions until calibration is complete.");
  Serial.println("Press 'c' to save calibration data to IMU flash when finished.");
  bool calibrationComplete = false;
  while (calibrationComplete == false) {
    if (!bno08x.getSensorEvent(&sensorValue)) {
      continue;
    }

    // if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    //   rotationVector.accuracy = sensorValue.status & 0x03;
    //   Serial.print("Game vector accuracy: ");
    //   Serial.println(rotationVector.accuracy);
    // }

    // if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    //   gyroscope.accuracy = sensorValue.status & 0x03;
    //   Serial.print("Gyroscope accuracy: ");
    //   Serial.println(gyroscope.accuracy);
    // }

    if (Serial.available() && Serial.read() == 'c') {
      int result = sh2_saveDcdNow();
      Serial.println(result == SH2_OK ? "Calibration saved": "Calibration save failed");
      calibrationComplete = true;
    }
  }
}

void initMotors(void) {
  motorDirectionKnown = false;

  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);

  pinMode(ENB_PIN, OUTPUT);
  digitalWrite(ENB_PIN, LOW);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  coastMotor();
}

void coastMotor(void) {
  digitalWrite(ENA_PIN, LOW);  // Disable both bridges: coast.
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(ENB_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}


void runMotor(int speed) {
  constrain(speed, -255, 255);
  if (speed == 0) {
    coastMotor();
    return;
  }

  const bool forward = speed > 0;
  if (motorDirectionKnown && forward != motorForward) {
    coastMotor();
    delay(5);
  }
  speed = abs(speed);

  digitalWrite(IN1_PIN, forward ? HIGH : LOW);
  digitalWrite(IN2_PIN, forward ? LOW : HIGH);
  digitalWrite(IN3_PIN, forward ? HIGH : LOW);
  digitalWrite(IN4_PIN, forward ? LOW : HIGH);

  analogWrite(ENA_PIN, speed);
  analogWrite(ENB_PIN, speed);
  motorForward = forward;
  motorDirectionKnown = true;
}