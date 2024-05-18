#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>

class MotorDriver
{
public:
  MotorDriver(int enA_pin, int enB_pin, int in1_pin, int in2_pin, int in3_pin, int in4_pin);
  void initMotorDriver();
  void forward();
  void backward();
  void clockWise();
  void counterClockWise();
  void stopMotors();
  void setPWMValue(int enA_speed, int enB_speed);
  
private:
  int enA;
  int enB;
  int in1;
  int in2;
  int in3;
  int in4;
};

#endif
