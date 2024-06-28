#include "KalmanFilter.h"

KalmanFilter2D::KalmanFilter2D(double Q_a, double Q_b, double R_m)
{
  Qa=Q_a; Qb=Q_b; R=R_m;

  Xa=0; Xb=0;

  // Kalman gain (2x1 matrix)
  Ka=0; Kb=0;
  
  // Covariance matrix (2x2 matrix)
  Paa=0; Pab=0;
  Pba=0; Pbb=0;

  kt = double(micros()); // Convert micro second to second
}

double KalmanFilter2D::estimate(double Zn, double W)
{
  dt = (double)(micros() - kt) / 1000000.0;
  
  // x = Fx + Gu
  Xa += dt*(W-Xb); 

  // P = FPF' + Q
  Paa -= dt*(Pab+Pba - dt*Pbb + Qa);
  Pab -= dt*Pbb;
  Pba -= dt*Pbb;
  Pbb += Qb*dt;

  // K = PH'/(HPH'+R)
  S = Paa+R;
  Ka = Paa/S;
  Kb = Pba/S;

  // x = x + K*(Zn - Hx)
  y = Zn - Xa;
  Xa += Ka*y;
  Xb += Kb*y;

  // P = (I-KnH)*P*(I-KnH)' + KnRKn'
  Paa_temp = (1 - Ka) * Paa;
  Pab_temp = (1 - Ka) * Pab;
  Pba_temp = Pba - Kb * Paa;
  Pbb_temp = Pbb - Kb * Pba;

  Paa = Paa_temp;
  Pab = Pab_temp;
  Pba = Pba_temp;
  Pbb = Pbb_temp;
  
  kt = (double)micros();
  return Xa;
}
