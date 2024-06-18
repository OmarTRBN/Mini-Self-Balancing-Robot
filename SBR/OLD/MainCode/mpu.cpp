#include "mpu.h"

OmarMPU::OmarMPU(const byte mpuAddress) : MPU(mpuAddress)
{
  // Constructor initialization
}

void OmarMPU::initMPU()
{
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void OmarMPU::readData()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14);

  /*
  // acc/16384.0 for +-2g and 0.00006103515625 = 1/16384
  // temp/340+36.53 and 0.0029411764705882 = 1/340
  // gyro/131.0 for +-2g and 0.0076335877862595 = 1/131
  // accX = (Wire.read() << 8 | Wire.read()) * 0.00006103515625;
  // accY = (Wire.read() << 8 | Wire.read()) * 0.00006103515625;
  // accZ = (Wire.read() << 8 | Wire.read()) * 0.00006103515625;
  // temp = (Wire.read() << 8 | Wire.read()) * 0.0029411764705882 + 36.53;
  // gyroX = (Wire.read() << 8 | Wire.read()) * 0.0076335877862595;
  // gyroY = (Wire.read() << 8 | Wire.read()) * 0.0076335877862595;
  // gyroZ = (Wire.read() << 8 | Wire.read()) * 0.0076335877862595;
  */

  accX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
  temp = (Wire.read() <<8 | Wire.read()) / 340 + 36.53;
  gyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  gyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}

void OmarMPU::calculateAngles()
{
  // Switch on low pass filter (LPF)
  Wire.beginTransmission(MPU);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Configure accelerometer settings
  // Full scale range 0f +-8g
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Get accelerometer values
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  accX = (Wire.read()<<8 | Wire.read()) / 4096.0;
  accY = (Wire.read()<<8 | Wire.read()) / 4096.0;
  accZ = (Wire.read()<<8 | Wire.read()) / 4096.0;
  
  // Configure gyroscope output
  Wire.beginTransmission(MPU);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission(); 

  previousTime = currentTime; // Previous time is stored before the actual time read
  currentTime = millis(); // Current time actual time read
  elapsedTime = (float)((currentTime - previousTime) / 1000.0); // Divide by 1000 to get seconds

  // Get gyrosopce values
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  
  gyroX= (Wire.read()<<8 | Wire.read()) / 65.5;
  gyroY= (Wire.read()<<8 | Wire.read()) / 65.5;
  gyroZ= (Wire.read()<<8 | Wire.read()) / 65.5;

  rollGyro = rollGyro + gyroX*elapsedTime;
  pitchGyro = pitchGyro + gyroY*elapsedTime;
  
  rollAcc=atan(accY/sqrt(accX*accX+accZ*accZ))*1/(3.142/180.0);
  pitchAcc=-atan(accX/sqrt(accY*accY+accZ*accZ))*1/(3.142/180.0);
}
