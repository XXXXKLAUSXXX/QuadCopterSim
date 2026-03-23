#include <Arduino.h>
#include <Wire.h>

//Gyro
const int MPU_addr = 0x68;
float rollAngle = 0;
float pitchAngle = 0;
float alpha = 0.1; // factor for complimentary filter
float dt = 0.01;

//PID
double kPAngle = 0.6, kIAngle = 3.5, kDAngle = 0.03;
double kPRate = 2, kIRate = 12, kDRate = 0;
// ^ van CarbonAeronautics

double pitchAngleI, pitchAngleD, desiredPitchRate, pitchAngleError, pitchAngleLastError;
double pitchRateI, pitchRateD, pitchInput, pitchRateError, pitchRateLastError;

double rollAngleI, rollAngleD, desiredRollRate, rollAngleError, rollAngleLastError;
double rollRateI, rollRateD, rollInput, rollRateError, rollRateLastError;

double targetPitch = 0, targetRoll = 0;

int hover = 1150;

int motorInputNW, motorInputNE, motorInputSE, motorInputSW;

// New average formula (complimentary filter)
void newAverage(float roll, float pitch, float Gx, float Gy) {
  rollAngle = (1 - alpha) * (rollAngle  + Gx * dt) + alpha * roll;
  pitchAngle = (1 - alpha) * (pitchAngle + Gy * dt) + alpha * pitch;
}

void setup() {
  //Wire
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // select PWR_MGMT_1 register
  Wire.write(0);     // wake up
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);  // select ACCEL_CONFIG register
  Wire.write(0);     // configure accelerometer to 16384 LSB/g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // GYRO_CONFIG
  Wire.write(0);     // default 250 deg/s
  Wire.endTransmission(true);

  Serial.begin(115200);

  pitchAngleI = 0;
  rollAngleI = 0;
  pitchRateI = 0;
  rollRateI = 0;
}

void loop() {
  bool newAvg = true;
  unsigned long startTime = micros(); // for dt calculation

  // --- Read accelerometer ---
  int16_t rawAcX, rawAcY, rawAcZ;
  float AcX, AcY, AcZ;
  float roll, pitch;

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // first register of accel measurements
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true); // request 6 registers (x, y and z accel)

  if (Wire.available() == 6) {
    rawAcX = Wire.read() << 8 | Wire.read();  // first read brings highest bits to the left, second fills in the rest with the lower 8 bits
    rawAcY = Wire.read() << 8 | Wire.read();
    rawAcZ = Wire.read() << 8 | Wire.read();
    AcX = (float)rawAcX / 16384.0;  // divide by LSB Sensitivity
    AcY = (float)rawAcY / 16384.0;
    AcZ = (float)rawAcZ / 16384.0;

    roll = AcY * 90;
    pitch = AcX * 90;
  } 
  else {
    newAvg = false;
  }

  // --- Read gyroscope ---
  int16_t rawGyX, rawGyY, rawGyZ;
  float Gx, Gy;

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);  // first register of gyro
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true); // request 6 registers (x, y and z gyro)

  if (Wire.available() == 6) {
    rawGyX = Wire.read() << 8 | Wire.read();  // first read brings highest bits to the left, second fills in the rest with the lower 8 bits
    rawGyY = Wire.read() << 8 | Wire.read();
    rawGyZ = Wire.read() << 8 | Wire.read();
    
    Gx = rawGyX / 131.0;  // convert to deg/s
    Gy = rawGyY / 131.0;
  } 
  else {
    newAvg = false;
  }

  if (newAvg) {
    newAverage(roll, pitch, Gx, Gy);
  }

  // --- outer PID ---
  pitchAngleError = targetPitch - pitchAngle;
  pitchAngleI = pitchAngleI + pitchAngleError * dt;
  pitchAngleI = constrain(pitchAngleI, -400 / kIAngle, 400 / kIAngle);
  pitchAngleD = (pitchAngleError - pitchAngleLastError) / dt;
  desiredPitchRate = kPAngle * pitchAngleError + kIAngle * pitchAngleI + kDAngle * pitchAngleD;
  desiredPitchRate = constrain(desiredPitchRate, -400, 400);
  pitchAngleLastError = pitchAngleError;

  rollAngleError = targetRoll - rollAngle;
  rollAngleI = rollAngleI + rollAngleError * dt;
  rollAngleI = constrain(rollAngleI, -400 / kIAngle, 400 / kIAngle);
  rollAngleD = (rollAngleError - rollAngleLastError) / dt;
  desiredRollRate = kPAngle * rollAngleError + kIAngle * rollAngleI + kDAngle * rollAngleD;
  desiredRollRate = constrain(desiredRollRate, -400, 400);
  rollAngleLastError = rollAngleError;

  // --- inner PID ---
  pitchRateError = desiredPitchRate - Gy;
  pitchRateI = pitchRateI + pitchRateError * dt;
  pitchRateI = constrain(pitchRateI, -400 / kIRate, 400 / kIRate);
  pitchRateD = (pitchRateError - pitchRateLastError) / dt;
  pitchInput = kPRate * pitchRateError + kIRate * pitchRateI + kDRate * pitchRateD;
  pitchInput = constrain(pitchInput, -400, 400);
  pitchRateLastError = pitchRateError;

  rollRateError = desiredRollRate - Gx;
  rollRateI = rollRateI + rollRateError * dt;
  rollRateI = constrain(rollRateI, -400 / kIRate, 400 / kIRate);
  rollRateD = (rollRateError - rollRateLastError) / dt;
  rollInput = kPRate * rollRateError + kIRate * rollRateI + kDRate * rollRateD;
  rollInput = constrain(rollInput, -400, 400);
  rollRateLastError = rollRateError;

  // --- calculate motor inputs ---
  motorInputNE = (hover - rollInput + pitchInput); // front right - counter clockwise
  motorInputSE = (hover - rollInput - pitchInput); // rear right - clockwise
  motorInputSW = (hover + rollInput - pitchInput); // rear left  - counter clockwise
  motorInputNW = (hover + rollInput + pitchInput); // front left - clockwise

  motorInputNE = constrain(motorInputNE, 1000, 2000);
  motorInputSE = constrain(motorInputSE, 1000, 2000);
  motorInputSW = constrain(motorInputSW, 1000, 2000);
  motorInputNW = constrain(motorInputNW, 1000, 2000);

  unsigned long elapsed = micros() - startTime;
  dt = elapsed / 1000000.0; // Update dt dynamically


  Serial.print("Roll: ");
  Serial.print(rollAngle);
  Serial.print(" | Pitch: ");
  Serial.println(pitchAngle);
  
  Serial.print("NE: ");
  Serial.print(motorInputNE);
  Serial.print(" | SE: ");
  Serial.print(motorInputSE);
  Serial.print(" | SW: ");
  Serial.print(motorInputSW);
  Serial.print(" | NW: ");
  Serial.println(motorInputNW);
}