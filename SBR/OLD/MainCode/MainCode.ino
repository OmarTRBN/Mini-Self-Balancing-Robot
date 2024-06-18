#include "mpu.h"

// Define the Kalman filter parameters
double Q_angle = 0.001; // Process noise variance for the accelerometer
double Q_bias = 0.003;  // Process noise variance for the gyro bias
double R_measure = 0.01; // Measurement noise variance

// Kalman filter variables for roll
//double angle_roll = 0; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
//double bias_roll = 0;  // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
//double P_roll[2][2] = {{0, 0}, {0, 0}}; // Error covariance matrix - 2x2 matrix

// Kalman filter variables for pitch
double angle_pitch = 0;
double bias_pitch = 0;
double P_pitch[2][2] = {{0, 0}, {0, 0}};

unsigned long lastTime, currentLoopTime;
double dt;

OmarMPU mpuObj(0x68);

void setup()
{
  Serial.begin(9600);
  mpuObj.initMPU();
  lastTime = millis();
}

void loop()
{
  mpuObj.calculateAngles();
  
  currentLoopTime = millis();
  dt = (currentLoopTime - lastTime) / 1000.0;
  lastTime = currentLoopTime;

//  double roll = getKalmanAngle(&angle_roll, &bias_roll, P_roll, mpuObj.rollAcc, mpuObj.rollGyro, dt);
  double pitch = getKalmanAngle(&angle_pitch, &bias_pitch, P_pitch, mpuObj.pitchAcc, mpuObj.pitchGyro, dt);
  Serial.print("PitchAcc: ");
  Serial.print(mpuObj.pitchAcc);
  Serial.print(" PitchGyro: ");
  Serial.print(mpuObj.pitchGyro);
  Serial.print(" Pitch: ");
  Serial.println(pitch);
//  Serial.print(" Pitch: ");
//  Serial.println(pitch);

  delay(10); // Small delay to allow serial output
}

double getKalmanAngle(double *angleK, double *bias, double P[2][2], double newAngle, double newRate, double dt) {
    // Predict
    double rate = newRate - *bias;
    *angleK += dt * rate;

    // Update error covariance matrix
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Compute Kalman gain
    double S = P[0][0] + R_measure;
    double K[2]; // Kalman gain - 2x1 matrix
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Update estimate with measurement zk (newAngle)
    double y = newAngle - *angleK;
    *angleK += K[0] * y;
    *bias += K[1] * y;

    // Update the error covariance matrix
    double P00_temp = P[0][0];
    double P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return *angleK;
}
