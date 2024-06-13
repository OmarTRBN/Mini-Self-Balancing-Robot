#include "mpu.h"

OmarMPU mpuObj(0x68);
uint32_t loopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2;
float Kalman1DOutput[]={0,0};

void setup()
{
  Serial.begin(9600);
  mpuObj.initMPU();
}

void loop()
{
  mpuObj.calculateAngles();
  
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, mpuObj.gyroX, mpuObj.angleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, mpuObj.gyroY, mpuObj.anglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  
  printData();
  
  while (micros() - loopTimer < 4000);
  loopTimer=micros();
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

void printData()
{
  Serial.print("Roll: ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" ,Pitch: ");
  Serial.println(KalmanAnglePitch);
}
