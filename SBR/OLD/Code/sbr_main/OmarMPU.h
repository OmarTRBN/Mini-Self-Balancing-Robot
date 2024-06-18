#ifndef OmarMPU_h
#define OmarMPU_h

#include <Wire.h>

class OmarMPU
{
public:
    OmarMPU(const int mpuAddress);
    void initMPU();
    void readData();
    
    const int MPU;
    float accX, accY, accZ, temp, gyroX, gyroY, gyroZ;
};

#endif
