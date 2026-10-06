#pragma once

#include <Adafruit_BNO08x.h>
#include <sh2.h>

struct RotationVector {
	float i;
	float j;
	float k;
	float real;
	float accuracy;
};

struct Gyroscope {
	float x;
	float y;
	float z;
	float accuracy;
};

extern Adafruit_BNO08x bno08x;
extern sh2_SensorValue_t sensorValue;
extern RotationVector rotationVector;
extern Gyroscope gyroscope;

void calibrateIMU(void);
void setReports(void);

// ESP32-S3 -> L298N. Pin values are defined in utils.cpp.
// Remove the ENA and ENB jumpers for GPIO control.
extern const uint8_t IN1_PIN;
extern const uint8_t IN2_PIN;
extern const uint8_t ENA_PIN;
extern const uint8_t IN3_PIN;
extern const uint8_t IN4_PIN;
extern const uint8_t ENB_PIN;

// Timing for the optional forward/backward test, defined in utils.cpp.
extern const unsigned long RUN_TIME_MS;
extern const unsigned long REVERSAL_PAUSE_MS;

// Call once from setup() before using any motor function.
// Configures both bridges, leaves both motors coasting, and resets the test.
void initMotors(void);

// Controls both motors together with signed PWM from -255 to 255.
// Negative and positive values select opposite directions; zero coasts.
// Direction reversals coast briefly before applying the new direction.
void runMotor(int speed);

// Disables both bridges so both motors coast.
void coastMotor(void);