#ifndef OmarMPU_h
#define OmarMPU_h

#include <Arduino.h>
#include <Wire.h>

void initMpu();
void getMpuValues(float *accX, float *accY, float  *accZ, float *gyroX, float *gyroY, float*gyroZ);

#endif
