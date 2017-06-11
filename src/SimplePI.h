#ifndef PID_SIMPLEPI_H
#define PID_SIMPLEPI_H

#include <iostream>

class SimplePI {
public:
  SimplePI();

  virtual ~SimplePI();

  void Init(double Kp, double Ki);

  double UpdateError(double measurement);

  void SetDesired(double desired);

  double Control(double measurement);

private:
  double _Kp;

  double _Ki;

  double _set_point;

  double _integral;
};


#endif //PID_SIMPLEPI_H
