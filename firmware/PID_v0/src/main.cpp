#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>
#include <AccelStepper.h>
#include <math.h>
#include <stdlib.h>

#define step_pin A2
#define dir_pin 13

AccelStepper stepper(AccelStepper::DRIVER, step_pin, dir_pin);
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

double Setpoint, Input, Output;

// pid config
double Kp=0, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// accel filtering
double filteredPitch = 0;
const double LPF_CUTOFF_HZ = 10.0;
const double CONTROL_HZ = 100.0;
const double alpha =
  (2.0 * PI * LPF_CUTOFF_HZ) /
  (2.0 * PI * LPF_CUTOFF_HZ + CONTROL_HZ);

// motor config
const int full_steps_per_rev = 200;
const int microsteps = 8;
const int steps_per_rev = full_steps_per_rev * microsteps; // 1600 steps per rev
const int control_period_us = 10000; // 100Hz 
const int MAX_RPM = 60;
uint32_t lastControlUs = 0;

void readSerialTuning() {
  static char command[32];
  static size_t commandLength = 0;

  while (Serial.available()) {
    char character = Serial.read();
    if (character == '\r') continue;

    if (character == '\n') {
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
          parameter != 'd' && parameter != 'D') valid = false;

      if (valid) {
        switch (parameter) {
          case 'p': case 'P': Kp = value; break;
          case 'i': case 'I': Ki = value; break;
          case 'd': case 'D': Kd = value; break;
        }

        myPID.SetTunings(Kp, Ki, Kd);
        Serial.print(F("Set "));
        Serial.print(parameter);
        Serial.print('=');
        Serial.println(value, 4);
      } else if (commandLength > 0) {
        Serial.println(F("Invalid tuning command"));
      }
      commandLength = 0;
    } else if (commandLength < sizeof(command) - 1) {
      command[commandLength++] = character;
    } else {
      commandLength = 0;
    }
  }
}

void initLIS() {
  Serial.println("initializing LIS3DH");
  if (! lis.begin(0x18)) {   // 0x18 is i2c addr of LIS3DH
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");
  // lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  // lis.setPerformanceMode(LIS3DH_MODE_LOW_POWER);
  Serial.print("Performance mode set to: ");
  switch (lis.getPerformanceMode()) {
    case LIS3DH_MODE_NORMAL: Serial.println("Normal 10bit"); break;
    case LIS3DH_MODE_LOW_POWER: Serial.println("Low Power 8bit"); break;
    case LIS3DH_MODE_HIGH_RESOLUTION: Serial.println("High Resolution 12bit"); break;
  }

  // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

    case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("1.6 Khz Low Power"); break;
  }
}

void setup() {
  Serial.begin(115200);

  initLIS();

  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);

  Setpoint = 1; // we want the bot to be level, equates nonzero deg accel read
  
  stepper.setMaxSpeed(MAX_RPM * steps_per_rev / 60.0);
  stepper.setMinPulseWidth(2); // Check the driver datasheet for its requirement.

  myPID.SetOutputLimits(-MAX_RPM, MAX_RPM);
  myPID.SetSampleTime(control_period_us / 1000);
  myPID.SetMode(AUTOMATIC);
}

void loop() {  
  readSerialTuning();
  stepper.runSpeed(); // Call as often as possible; do not put a delay here.

  uint32_t now = micros();
  if ((uint32_t)(now - lastControlUs) >= control_period_us) {
    lastControlUs = now;

  sensors_event_t event;
  lis.getEvent(&event);   
  float x = event.acceleration.x;   
  float y = event.acceleration.y;   
  float z = event.acceleration.z;   

  Input = atan2(x, sqrt(y * y + z * z)) * 180.0 / PI; // pitch of the bot is our error 
  filteredPitch = alpha * Input + (1.0 - alpha) * filteredPitch;
  Input = filteredPitch;
  myPID.Compute();
  stepper.setSpeed(-1*Output * steps_per_rev / 60.0);

  static uint32_t lastPrintUs = 0;
  if ((uint32_t)(now - lastPrintUs) >= 200000UL && Serial.availableForWrite() >= 48) {
    lastPrintUs = now;
    Serial.print(F("Pitch: "));
    Serial.print(Input, 1);
    Serial.print(F(" deg, RPM command: "));
    Serial.println(Output, 1);
  }
  }
}