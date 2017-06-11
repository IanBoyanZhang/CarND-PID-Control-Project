#include "SimplePI.h"

SimplePI::SimplePI() {
  _integral = 0;
}

SimplePI::~SimplePI() {}

void SimplePI::Init(double Kp, double Ki) {
  _Kp = Kp;
  _Ki = Ki;
}

void SimplePI::SetDesired(double desired) {
  _set_point = desired;
  _integral = 0;
  //std::cout << "Error: " << _set_point << std::endl;
}

double SimplePI::UpdateError(double measurement) {
  double error = _set_point - measurement;
  _integral += error;
  return _Kp * error + _Ki * _integral;
}

double SimplePI::Control(double measurement) {
  return -UpdateError(measurement);
}
