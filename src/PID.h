#ifndef PID_H
#define PID_H

using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;



  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  void InitCTE(double cte);

  void InitPotentialChange(double dKd, double dKi, double dKp);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double Control(double scalar);

  double GetAccuError();

  /**
   * Setting tolerance
   * @param tol
   * @return
   */
  double Twiddle(double tol);

private:
  double _prev_cte;
  /**
   * Accumulative factor over complete run
   */
  double _accu_error_sq;

  unsigned int _step_count;

  double _dKp;
  double _dKi;
  double _dKd;

  // Maximum and minimum allowable integrator state
  double _iMax;

  double _iMin;

  double _store_error();
};

#endif /* PID_H */
