#include <iostream>
#include <cmath>
#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {

}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
/*  Kp = Kp;
  Ki = Ki;
  Kd = Kd;*/

  SetP(Kp, Ki, Kd);

  _accu_error_sq = 0;
  _step_count = 0;
  _twiddle_iter = 0;
  _best_ever_mse = numeric_limits<double>::max();
  _prev_cte = 0;

  _is_minima = false;

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
   * 1 Uphill
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
    _is_minima = true;
    return _best_ever_mse;
  }

  switch (_input_state) {
    case 1:
      // p += dp
      _ProgressDescentDirection(1, _params_index);
      break;
    case 2:
      // p -= 2 * dp
      _ProgressDescentDirection(-2, _params_index);
      break;
    default:
      cout << "Error twiddle input state" << endl;
      break;
  }

  if (mse < _best_ever_mse) {
    _output_state = 1;
    _best_ever_mse = mse;
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
      // dp *= 1.1
      _ScalePotentialChange(1.1, _params_index);
      // Progress to next parameter tuning
      _twiddle_iter = (_twiddle_iter + 1) % 3;
      _input_state = 1;
      break;
    case 2:
      // dp *= 0.9
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

  return _best_ever_mse;
}

/**
 * Accumulate MSE
 */
double PID::Next() {
  _accu_error_sq += pow(TotalError(), 2);
  _step_count += 1;
  return GetMSE();
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

bool PID::ReachMinima() {
  return _is_minima;
}

vector<double> PID::GetP() {
  return _p_vector;
}

vector<double> PID::GetDp() {
  return _dp_vector;
}

void PID::SetP(double Kp, double Ki, double Kd) {
  _p_vector.push_back(Kp);
  _p_vector.push_back(Ki);
  _p_vector.push_back(Kd);
}
