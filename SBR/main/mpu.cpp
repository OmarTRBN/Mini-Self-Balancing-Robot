#include "mpu.h"

void initMpu()
{
    Wire.begin();
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
}

void getMpuValues(float *accX, float *accY, float *accZ, float *gyroX, float *gyroY, float *gyroZ)
{
    // Switch on low pass filter (LPF)
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();

    // Configure accelerometer settings
    // Full scale range 0f +-8g
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    // Get accelerometer values
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    *accX = (Wire.read() << 8 | Wire.read()) / 4096.0;
    *accY = (Wire.read() << 8 | Wire.read()) / 4096.0;
    *accZ = (Wire.read() << 8 | Wire.read()) / 4096.0;

    // Configure gyroscope output
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x8);
    Wire.endTransmission();

    // Get gyrosopce values
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);

    *gyroX = (Wire.read() << 8 | Wire.read()) / 65.5;
    *gyroY = (Wire.read() << 8 | Wire.read()) / 65.5;
    *gyroZ = (Wire.read() << 8 | Wire.read()) / 65.5;
}
