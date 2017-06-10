#ifndef PID_TOOLS_H
#define PID_TOOLS_H


class Tools {
public:
  /**
   * Constructor
   */
  Tools(double Kp, double Ki, double Kd);
  /**
   * Destructor
   */
  virtual ~Tools();

  void twiddle();

private:

  double Kp_;
  double Ki_;
  double Kd_;

  /**
   * Potential change
   */
  double dKp_;
  double dKi_;
  double dKd_;
};


#endif //PID_TOOLS_H
