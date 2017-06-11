#include <iostream>
#include <cmath>
#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  _accu_error_sq = 0;
  _step_count = 0;
  _twiddle_iter = 0;
  _best_ever_mse = numeric_limits<double>::max();

  /**
   * input state
   * 0 un-initialized
   * 1 From forward
   * 2 From backward
   * Otherwise error state
   */
  _input_state = 1;

  /**
   * output state
   * 0 un-initialized
   * 1 Uphill and progress
   * 2 Downhill
   * 3 Reverse
   * Otherwise error state
   */
  _output_state = 0;

  /**
   * 0: Kp
   * 1: Ki
   * 2: Kd
   */
  _params_index = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
/*  Kp = Kp;
  Ki = Ki;
  Kd = Kd;*/

  _p_vector.push_back(Kp);
  _p_vector.push_back(Ki);
  _p_vector.push_back(Kd);
}

void PID::InitCTE(double cte) {
  _prev_cte = cte;
}

void PID::InitPotentialChange(double dKp, double dKi, double dKd) {
  _dp_vector.push_back(dKp);
  _dp_vector.push_back(dKi);
  _dp_vector.push_back(dKd);
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error += cte; 
  d_error = cte - _prev_cte;
  _prev_cte = cte;
}

double PID::TotalError() {
  double Kp = _p_vector[0];
  double Ki = _p_vector[1];
  double Kd = _p_vector[2];
  return Kp * p_error + Ki * i_error + Kd * d_error;
}

double PID::Control(double scalar) {
  return -scalar * TotalError();
}

/**
 * For main loop to decide should reset current run
 * @return
 */
double PID::GetMSE(){
  // Potential Change
  return _accu_error_sq/_step_count;
}

/**
 *
 * @param tol
 * @param err
 * @return _best_ever_mse
 */
double PID::Twiddle(double tol, double mse) {
  if (_GetDpSum() < tol) {
    _best_ever_mse = mse;
    return _best_ever_mse;
  }

  switch (_input_state) {
    case 1:
      _ProgressDescentDirection(1, _params_index);
      break;
    case 2:
      _ProgressDescentDirection(-2, _params_index);
      break;
    default:
      cout << "Error twiddle input state" << endl;
      break;
  }

  if (mse < _best_ever_mse) {
    _output_state = 1;
  } else {
    switch (_input_state) {
      case 1:
        _output_state = 3;
        break;
      case 2:
        _output_state = 2;
        break;
      default:
        cout << "Error twiddle input state" << endl;
        break;
    }
  }

  switch (_output_state) {
    case 1:
      _ScalePotentialChange(1.1, _params_index);
      // Progress to next parameter tuning
      _twiddle_iter = (_twiddle_iter + 1) % 3;
      _input_state = 1;
      break;
    case 2:
      _ScalePotentialChange(0.9, _params_index);
      // Progress to next parameter tuning
      _twiddle_iter = (_twiddle_iter + 1) % 3;
      _input_state = 1;
      break;
    case 3:
      _input_state = 2;
      // Progress to next parameter for tuning
      _twiddle_iter = (_twiddle_iter + 1) % 3;
      break;
    default:
      cout << "Error twiddle output state" << endl;
  }

}

void PID::Next() {
  _accu_error_sq += pow(TotalError(), 2);
  _step_count += 1;
}

double PID::_GetDpSum() {
  // Last value indicates return type
  return std::accumulate(_dp_vector.begin(), _dp_vector.end(), 0.0);
}

void PID::_ScalePotentialChange(double scale, size_t index) {
  _dp_vector[index] *= scale;
}

void PID::_ProgressDescentDirection(double scale, size_t index) {
  _p_vector[index] += scale * _dp_vector[index];
}


