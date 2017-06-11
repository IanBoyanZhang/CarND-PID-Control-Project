#ifndef PID_H
#define PID_H

#include <vector>
#include <numeric>
#include <cmath>
#include <ctime>

using namespace std;

class PID {
public:


  /*
  * Coefficients
  */ 
  /*double Kp;
  double Ki;
  double Kd;*/

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

  void InitPotentialChange(double dKp, double dKi, double dKd);
  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double dt);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double Control(double scalar);

  double GetMSE();

  /**
   * Setting tolerance
   * @param tol
   * @return
   */
  double Twiddle(double tol, double mse);

  double Next();

  bool ReachMinima();

  vector<double> GetP();

  vector<double> GetDp();

  void SetP(double Kp, double Ki, double Kd);

private:
  /*
   * Errors
   */
  double p_error;
  double i_error;
  double d_error;

  double _prev_cte;
  /**
   * Accumulative factor over complete run
   */
  double _accu_error_sq;

  unsigned int _step_count;

  double _best_ever_mse;

  /**
   * Twiddle loop State Machine state variables
   */
  unsigned int _input_state;

  unsigned int _output_state;

  // Kp Ki Kd iterator
  unsigned int _params_index;

  bool _is_minima;

  double _GetDpSum();

  void _ScalePotentialChange(double scale, size_t index);

  void _ProgressDescentDirection(double scale, size_t index);

  vector<double> _dp_vector;
  vector<double> _p_vector;
  // Maximum and minimum allowable integrator state
  double _iMax;

  double _iMin;

};

#endif /* PID_H */
