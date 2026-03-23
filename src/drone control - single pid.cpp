#include <Arduino.h>
#include <Wire.h>

// Gyro
/**/
const int MPU_addr = 0x68;
/*
Unit:??
Ranges: ? - ?
*/
float rollAngle = 0;
/*
Unit:??
Ranges: ? - ?
*/
float pitchAngle = 0;
/*
Unit:??
Ranges: ? - ?
*/
float alpha = 0.1; // factor for complimentary filter
/*
Unit:??
Ranges: ? - ?
*/
float dt = 0.01;

// PID
/*
Unit:??
Ranges: ? - ?
*/
double kP = 0.5, kI = 0.05, kD = 0.1; // ubicoders

/*
Unit:??
Ranges: ? - ?
*/
double pitchI, pitchD, pitchPID, pitchError, pitchLastError;

/*
Unit:??
Ranges: ? - ?
*/
double rollI, rollD, rollPID, rollError, rollLastError;

/*
Unit:??
Ranges: ? - ?
*/
double targetPitch = 0, targetRoll = 0;

/*
Unit:??
Ranges: ? - ?
Description: ?
*/
int hover = 1150;

/*
Unit:??
Ranges: ? - ?
*/
int motorInputNW, motorInputNE, motorInputSE, motorInputSW;

/*
New average formula (complimentary filter)
Better if this returns a struct that can be used to update the global variables instead of doing it immidiatly. Because this makes your code less flexible and more ambigious.
 */
void newAverage(float roll, float pitch, float Gx, float Gy)
{
  rollAngle = (1 - alpha) * (rollAngle + Gx * dt) + alpha * roll;
  pitchAngle = (1 - alpha) * (pitchAngle + Gy * dt) + alpha * pitch;
}

void setup()
{
  // Wire // Should be a function
  {
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B); // select PWR_MGMT_1 register
    Wire.write(0);    // wake up
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x1C); // select ACCEL_CONFIG register
    Wire.write(0);    // configure accelerometer to 16384 LSB/g
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x1B); // GYRO_CONFIG
    Wire.write(0);    // default 250 deg/s
    Wire.endTransmission(true);

    Serial.begin(115200);
  }
  // What's going on here?
  pitchI = 0;
  rollI = 0;
}

void loop()
{
  // Why is this not a global variable?
  bool newAvg = true;
  // Why this is also not a global variable? Seems to me that starttime could be a const unsigned long
  unsigned long startTime = micros(); // for dt calculation

  // --- Read accelerometer --- //Should be a function
  int16_t rawAcX, rawAcY, rawAcZ;
  float AcX, AcY, AcZ;
  float roll, pitch;
  {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // first register of accel measurements
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 6, true); // request 6 registers (x, y and z accel)

    if (Wire.available() == 6)
    {
      rawAcX = Wire.read() << 8 | Wire.read(); // first read brings highest bits to the left, second fills in the rest with the lower 8 bits
      rawAcY = Wire.read() << 8 | Wire.read();
      rawAcZ = Wire.read() << 8 | Wire.read();
      AcX = (float)rawAcX / 16384.0; // divide by LSB Sensitivity
      AcY = (float)rawAcY / 16384.0;
      AcZ = (float)rawAcZ / 16384.0;

      roll = AcY * 90;
      pitch = AcX * 90;
    }
    else
    {
      newAvg = false;
    }
  }
  // --- Read gyroscope --- // Should be a function
  // These variables should be global to increase flexibility of your program.
  int16_t rawGyX, rawGyY, rawGyZ;
  /*
  unit: degrees
  ranges: ? - ?
  */
  float Gx, Gy;
  {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x43); // first register of gyro
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 6, true); // request 6 registers (x, y and z gyro)

    if (Wire.available() == 6)
    {
      rawGyX = Wire.read() << 8 | Wire.read(); // first read brings highest bits to the left, second fills in the rest with the lower 8 bits
      rawGyY = Wire.read() << 8 | Wire.read();
      rawGyZ = Wire.read() << 8 | Wire.read();

      Gx = rawGyX / 131.0; // convert to deg/s
      Gy = rawGyY / 131.0;
    }
    else
    {
      newAvg = false;
    }

    if (newAvg)
    {
      newAverage(roll, pitch, Gx, Gy);
    }
  }

  // --- PID --- // This must be a function
  {
    pitchError = targetPitch - pitchAngle;
    pitchI = pitchI + pitchError * dt;
    pitchI = constrain(pitchI, -400 / kI, 400 / kI);
    pitchD = (pitchError - pitchLastError) / dt;
    pitchPID = kP * pitchError + kI * pitchI + kD * pitchD;
    pitchPID = constrain(pitchPID, -400, 400);
    pitchLastError = pitchError;

    rollError = targetRoll - rollAngle;
    rollI = rollI + rollError * dt;
    rollI = constrain(rollI, -400 / kI, 400 / kI);
    rollD = (rollError - rollLastError) / dt;
    rollPID = kP * rollError + kI * rollI + kD * rollD;
    rollPID = constrain(rollPID, -400, 400);
    rollLastError = rollError;
  }

  // --- calculate motor inputs --- // Should be a function
  {
    motorInputNE = (hover - rollPID + pitchPID /*- yawPID*/); // front right - counter clockwise
    motorInputSE = (hover - rollPID - pitchPID /*+ yawPID*/); // rear right - clockwise
    motorInputSW = (hover + rollPID - pitchPID /*- yawPID*/); // rear left  - counter clockwise
    motorInputNW = (hover + rollPID + pitchPID /*+ yawPID*/); // front left - clockwise

    motorInputNE = constrain(motorInputNE, 1000, 2000);
    motorInputSE = constrain(motorInputSE, 1000, 2000);
    motorInputSW = constrain(motorInputSW, 1000, 2000);
    motorInputNW = constrain(motorInputNW, 1000, 2000);

    unsigned long elapsed = micros() - startTime;
    dt = elapsed / 1000000.0; // Update dt dynamically
  }
  // Should be a function named: Arduino_PrintInfo
  {
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
}

/*
mia
  motoren moeten nog gedefineerd worden
  standard pwm is 1000 - 2000
    our pwm is ?

geen yaw

gyro alpha te groot ?

cascaded PID?
(angle error -> PID -> desired rate -> PID -> motor input) (ep 16 carbon)
*/

/*
tweak kP, kI & kD

kP
te hoog: overshoot de goal snel en met hoge frequentie
te laag: het reageert langzaam op een nieuwe input

kI
te hoog: langzame oscillations (overshoots)
te laag: het drift terug naar de voorheen gecorrigeerde angle. angle wordt niet goed vast gehouden

kD
te hoog: motoren worden twitchy
te laag: overshoots
*/