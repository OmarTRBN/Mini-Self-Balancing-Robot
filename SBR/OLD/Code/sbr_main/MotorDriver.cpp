#include "MotorDriver.h"

MotorDriver::MotorDriver(int enA_pin, int enB_pin, int in1_pin, int in2_pin, int in3_pin, int in4_pin)
{
    enA = enA_pin;
    enB = enB_pin;
    in1 = in1_pin;
    in2 = in2_pin;
    in3 = in3_pin;
    in4 = in4_pin;
}

void MotorDriver::initMotorDriver()
{
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
}

void MotorDriver::backward()
{
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
}

void MotorDriver::forward()
{
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
}

void MotorDriver::clockWise()
{
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
}

void MotorDriver::counterClockWise()
{
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
}

void MotorDriver::stopMotors()
{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    
    analogWrite(enA, 0);
    analogWrite(enB, 0);
}

void MotorDriver::setPWMValue(int enA_speed, int enB_speed)
{
    analogWrite(enA, enA_speed);
    analogWrite(enB, enB_speed);
}
