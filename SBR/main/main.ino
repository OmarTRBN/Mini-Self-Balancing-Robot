#include <Wire.h>
#include "mpu.h"
#include "KalmanFilter.h"

MPU6050 mpu;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;

void setup() 
{
  Serial.begin(9600);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }
  mpu.calibrateGyro();
}

void loop()
{
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  // Calculate Pitch & Roll from accelerometer (deg)
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;

  // Kalman filter
  kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  kalRoll = kalmanX.update(accRoll, gyr.XAxis);

//  Serial.print(accPitch);
//  Serial.print(":");
//  Serial.print(accRoll);
//  Serial.print(":");
  Serial.print(kalPitch);
//  Serial.print(":");
//  Serial.print(kalRoll);
//  Serial.print(":");
//  Serial.print(acc.XAxis);
//  Serial.print(":");
//  Serial.print(acc.YAxis);
//  Serial.print(":");
//  Serial.print(acc.ZAxis);
//  Serial.print(":");
//  Serial.print(gyr.XAxis);
//  Serial.print(":");
//  Serial.print(gyr.YAxis);
//  Serial.print(":");
//  Serial.print(gyr.ZAxis);

  Serial.println();
}
