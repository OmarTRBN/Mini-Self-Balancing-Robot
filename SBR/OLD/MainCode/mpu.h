#ifndef OmarMPU_h
#define OmarMPU_h

#include <Arduino.h>
#include <Wire.h>

class OmarMPU
{
public:
  OmarMPU(const byte mpuAddress);
  void initMPU();
  void readData();
  void calculateAngles();
  
  float accX, accY, accZ, temp, gyroX=0, gyroY=0, gyroZ=0, rollAcc, pitchAcc, rollGyro, pitchGyro;
  
private:
  const int MPU;
  uint32_t previousTime=0, currentTime=0;
  float elapsedTime=0.0;
};

#endif
