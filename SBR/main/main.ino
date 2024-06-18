#include "mpu.h"

float roll = 0.0, pitch = 0.0, yaw = 0.0, rollAcc;
float accX, accY, accZ, gyroX, gyroY, gyroZ;

void setup() 
{
  Serial.begin(9600);
    initMpu();
}

void loop()
{
  
    getMpuValues(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
//    rollAcc=atan(accY/sqrt(accX*accX+accZ*accZ))*1/(3.142/180.0);
//    kalmanFilter(&roll, rollAcc, gyroX);
    
    printData();
    delay(1);
}

void kalmanFilter(float *angleKalman, float Zk, float newRate)
{   
    static float Q_angle = 0.003, bias = 0.0, Q_bias = 0.00005, R_measure=0.003;
    static float S, K[2], P[2][2]={{0, 0}, {0, 0}}, P00_temp, P01_temp, y, dt;
    static uint32_t currentKalmanTime, lastKalmanTime=0.0;
    
    currentKalmanTime = millis();
    dt = (currentKalmanTime - lastKalmanTime) / 1000.0;
    lastKalmanTime = currentKalmanTime; 
    
    // Predict X(k+1,k) = F*X(k,k)
    *angleKalman += dt * (newRate - bias);

    // Update error covariance matrix P(k+1,k) = F*P(k,k)*F_T + Q
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    
    // Compute Kalman gain
    S = 1.0 / (P[0][0] + R_measure);
    K[0] = P[0][0] * S;
    K[1] = P[1][0] * S;

    // Update estimate with measurement zk (newAngle)
    y = Zk - *angleKalman;
    *angleKalman += K[0] * y;
    bias += K[1] * y;
    
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
}

void printData()
{
//    Serial.print("Roll: ");
//    Serial.println(roll);
//    Serial.print( "Pitch: ");
//    Serial.print(pitch);
//    Serial.print(" Yaw: ");
//    Serial.println(yaw);
     Serial.print("aX = "); Serial.print(accX);
    Serial.print(" | aY = "); Serial.print(accY);
    Serial.print(" | aZ = "); Serial.print(accZ);
    Serial.print(" | gX = "); Serial.print(gyroX);
    Serial.print(" | gY = "); Serial.print(gyroY);
    Serial.print(" | gZ = "); Serial.print(gyroZ);

     Serial.print(" | Time = ");
     Serial.println(micros());
}
