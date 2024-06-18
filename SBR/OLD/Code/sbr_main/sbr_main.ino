#include "OmarMPU.h"

OmarMPU mpuObj(0x68);

void setup()
{
    Serial.begin(9600);
    mpuObj.initMPU();
}

void loop()
{
    mpuObj.readData();
    printData();
}

void printData()
{
    Serial.print("AccX: ");
    Serial.print(mpuObj.accX);
    Serial.print(", AccY: ");
    Serial.print(mpuObj.accY);
    Serial.print(", AccZ: ");
    Serial.print(mpuObj.accZ);
    Serial.print(", Temp: ");
    Serial.print(mpuObj.temp / 340.00 + 36.53);
    Serial.print(", GyroX: ");
    Serial.print(mpuObj.gyroX);
    Serial.print(", GyroY: ");
    Serial.print(mpuObj.gyroY);
    Serial.print(", GyroZ: ");
    Serial.println(mpuObj.gyroZ);
}