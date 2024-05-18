#include "OmarMPU.h"

OmarMPU::OmarMPU(const int mpuAddress) : MPU(mpuAddress)
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
