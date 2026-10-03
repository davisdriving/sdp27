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
 *   SCL  -> GPIO12 (SCK)
 *   DI   -> GPIO11 (MOSI)
 *   SDA  -> GPIO13 (MISO)
 *   CS   -> GPIO10
 *   INT  -> GPIO9
 *   RST  -> GPIO5
 *   P0   -> 3.3V
 *   P1   -> 3.3V
 * 
 *  NOTE: Gyroscope and and rotation vector are NOT simultaneously available. The BNO08x will only send one report at a time, 
 *  so the code must wait for both reports to be received before processing. Average time difference between the reports is 2ms.
 */

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <SPI.h>
#include <sh2.h>
#include "utils.h"

#define BNO08X_RESET 5  // SPI pins
#define BNO08X_INT 9
#define BNO08X_CS 10
#define BNO08X_MOSI 11
#define BNO08X_SCK 12
#define BNO08X_MISO 13
#define forward_pin 18
#define backward_pin 8


Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

bool haveRotationVector = false;
bool haveGyroscope = false;

bool motorState = false; // false = forward, true = backward

RotationVector rotationVector;
Gyroscope gyroscope;

float roll;

void setup(void) {
  Serial.begin(115200);

  pinMode(forward_pin, OUTPUT);
  pinMode(backward_pin, OUTPUT);
  digitalWrite(forward_pin, LOW);
  digitalWrite(backward_pin, LOW);

  SPI.begin(BNO08X_SCK, BNO08X_MISO, BNO08X_MOSI, BNO08X_CS); 

  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports();
  calibrateIMU();
  initMotors();

  Serial.println("Reading events");
  delay(100);
}



void loop() {
  digitalWrite(forward_pin, LOW);
  digitalWrite(backward_pin, LOW);

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    haveRotationVector = true;
    rotationVector.i = sensorValue.un.rotationVector.i;
    rotationVector.j = sensorValue.un.rotationVector.j;
    rotationVector.k = sensorValue.un.rotationVector.k;
    rotationVector.real = sensorValue.un.rotationVector.real;
    rotationVector.accuracy = sensorValue.status & 0x03;
    float roll = atan2f(
    2.0f * (rotationVector.real * rotationVector.i
          + rotationVector.j * rotationVector.k),
    1.0f - 2.0f * (rotationVector.i * rotationVector.i
                + rotationVector.j * rotationVector.j)) * 180.0f / PI;

    Serial.print("roll: ");
    Serial.print(roll);

    if (roll < 0) { // move forward
      digitalWrite(forward_pin, HIGH);
      
      if (motorState == true) { // motor going back, need to change direction
        coastMotor();
        delay(100);
      }
      runMotor(false);
      motorState = false;
    
    }
    else { // roll > 0, move back
      digitalWrite(backward_pin, HIGH);
      if (motorState == false) { // motor going back, need to change direction
        coastMotor();
        delay(100);
      }
      runMotor(true);
      motorState = true;
    }
  }
  // if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
  //   haveGyroscope = true;
  //   gyroscope.x = sensorValue.un.gyroscope.x;
  //   gyroscope.y = sensorValue.un.gyroscope.y;
  //   gyroscope.z = sensorValue.un.gyroscope.z;
  //   gyroscope.accuracy = sensorValue.status & 0x03; // Extract accuracy from status bits
  // }

//   if (haveRotationVector && haveGyroscope) {
//     sinroll = 2 * (rotationVector.real * rotationVector.j - rotationVector.k * rotationVector.i);
//     roll = asin(sinroll) * 180.0 / PI;
//     Serial.print("roll: ");
//     Serial.print(roll);
//     Serial.print(" | Rotation Accuracy: ");
//     Serial.print(rotationVector.accuracy);
//     Serial.print(" | Gyroscope Accuracy: ");
//     Serial.print(gyroscope.accuracy);
//     Serial.print(" | Gyro X: ");
//     Serial.print(gyroscope.x);
//     Serial.print(" | Gyro Y: ");
//     Serial.print(gyroscope.y);
//     Serial.print(" | Gyro Z: ");
//     Serial.println(gyroscope.z);
//     haveRotationVector = false;
//     haveGyroscope = false;
//   }
}