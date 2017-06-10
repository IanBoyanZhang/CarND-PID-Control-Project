#include <iostream>
#include <cmath>
#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  _accu_error_sq = 0;
  _step_count = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp = Kp;
  Ki = Ki;
  Kd = Kd;
}

void PID::InitCTE(double cte) {
  _prev_cte = cte;
}

void PID::InitPotentialChange(double dKd, double dKi, double dKp) {
  _dKd = dKd;
  _dKi = dKi;
  _dKp = dKp;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error += cte; 
  d_error = cte - _prev_cte;
  _prev_cte = cte;
}

double PID::TotalError() {
  return Kp * p_error + Ki * i_error + Kd * d_error;
}

double PID::Control(double scalar) {
  return -scalar * TotalError();
}

/**
 * For main loop to decide should reset current run
 * @return
 */
double PID::GetAccuError(){
  // Potential Change
  return _accu_error_sq;
}

/**
 * TODO: cache Total error value
 * @return MSE
 */
double PID::_store_error() {
  _accu_error_sq += pow(TotalError(), 2);
  _step_count += 1;
  return _accu_error_sq/_step_count;
}
