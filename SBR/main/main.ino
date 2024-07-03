#include <Wire.h>
#include "mpu.h"
#include "KalmanFilter.h"

// Motors
#define PWMB 11
#define BIN2 10
#define BIN1 9
#define PWMA 5
#define AIN2 6
#define AIN1 7
#define STBY 8

// MPU 
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float accPitch = 0;
float accRoll = 0;

// Kalman Filter
//double Q_a_pitch=0.001, Q_b_pitch=0.003, R_pitch=0.05;
//KalmanFilter2D kalmanPitch(Q_a_pitch, Q_b_pitch, R_pitch);
//double kalPitch = 0; 
double Q_a_roll=0.001, Q_b_roll=0.003, R_roll=0.02; 
KalmanFilter2D kalmanRoll(Q_a_roll, Q_b_roll, R_roll);
double kalRoll = 0;

// PID Controller
float pidError=0.0, prevPidError=0.0, prevPrevPidError=0.0, integral=0.0, derivative=0.0;
const float Kp=25.5, Ki=315.3, Kd=0.7;
unsigned long pidTimeNow=0, pidTimePrev=0; float deltaT=0.0;
int outputPID=0;

void setup() 
{
  pinMode(PWMB, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
  Serial.begin(9600);

  // Initialize MPU6050
  initMpu();
}

void loop()
{
  getValues(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  
  // Calculate Pitch & Roll from accelerometer (deg)
  //  accPitch = (double) atan( (double)-accX / (double)sqrt(accY*accY + accZ*accZ) ) * 180.0/3.141593;
  accRoll  = (double) atan( (double)accY / sqrt(accX*accX + accZ*accZ) ) * 180.0/3.141593;

  // Kalman filter
  //  kalPitch = kalmanPitch.estimate(accPitch, gyroY);
  kalRoll = kalmanRoll.estimate(accRoll, gyroX);
  
  // PID Controller
  pidTimeNow = micros();
  deltaT = (float)(pidTimeNow-pidTimePrev)/1.0e6;

  pidError = -2.5 - (float)kalRoll; // Proportional term
  integral += pidError*deltaT; // Integral term
  derivative = (float)(3*pidError-4*prevPidError+prevPrevPidError)/(2.0*deltaT); // Derivative term
  outputPID = (int) (Kp*pidError + Ki*integral + Kd*derivative);

  // Processing the PID output
  proccesOutput();
  analogWrite(PWMA, outputPID);
  analogWrite(PWMB, outputPID);
  prevPidError = pidError;
  prevPrevPidError = prevPidError;
  pidTimePrev=pidTimeNow;

  // Sending data
//  Serial.print("Control Input: ");
//  Serial.print(outputPID);
  Serial.print(", Kalman roll angle: ");
  Serial.println(kalRoll);
//  Serial.print(", Kalman pitch angle: ");
//  Serial.println(kalPitch);
}

void proccesOutput()
{
  if (outputPID>0)
  {
    moveForward();
    if (outputPID>255)
    {
      outputPID=255;
    }
  }
  else
  {
    outputPID=-outputPID;
    moveBackward();
    if (outputPID>255)
    {
      outputPID=255;
    }
  }
}

void moveForward()
{
  digitalWrite(AIN1, 1);
  digitalWrite(AIN2, 0);
  digitalWrite(BIN1, 0);
  digitalWrite(BIN2, 1);
}

void moveBackward()
{
  digitalWrite(AIN1, 0);
  digitalWrite(AIN2, 1);
  digitalWrite(BIN1, 1);
  digitalWrite(BIN2, 0);
}
