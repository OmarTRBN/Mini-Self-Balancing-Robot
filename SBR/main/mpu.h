#ifndef mpu_h
#define mpu_h

#include "Arduino.h"
#include <Wire.h>

void initMpu();
void getValues(float *accX, float *accY, float *accZ, float *gyroX, float *gyroY, float *gyroZ);

#endif
