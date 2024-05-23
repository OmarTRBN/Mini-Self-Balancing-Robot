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
  
  float accX, accY, accZ, temp, gyroX, gyroY, gyroZ;

private:
  const int MPU;
};

#endif
