#include <iostream>
#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp = Kp;
  Ki = Ki;
  Kd = Kd;
}

void PID::InitCTE(double cte) {
  prev_error = cte;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error += cte; 
  d_error = cte - prev_cte;
  prev_cte = cte;
}

double PID::TotalError() {
  return -(Kp * p_error + Ki * i_error + Kd * d_error);
}

