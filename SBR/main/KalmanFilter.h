#ifndef kalman_h
#define kalman_h

#include "Arduino.h"

class KalmanFilter2D
{
public:
  KalmanFilter2D(double Q_a, double Q_b, double R_m);
  double estimate(double Zn, double W);
  
private:
  double Qa, Qb, R;
  double Xa, Xb;
  double Paa, Pab, Pba, Pbb, Ka, Kb; // Covariance matrix (2x2) and Kalman gain (2x1)
  double Paa_temp, Pab_temp, Pba_temp, Pbb_temp;
  double S, y;
  double dt, kt;
};

#endif
