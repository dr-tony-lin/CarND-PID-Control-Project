#ifndef _TUNE_TWIDDLE_H_
#define _TUNE_TWIDDLE_H_

#define VERBOSE_OUT

#include "Eigen/Dense"
using Eigen::VectorXd;

class Twiddle {
public:
  /**
   * Run the simulation model, and return the squared mean error
   * @param p the coefficient vector
   * @param target the target value to reach
   * @param steps the steps required to reach a convergence 
   * @param dt the delta time for each step
   */ 
  virtual double run(const VectorXd &p, const double target = 0, const int steps = 100, const double dt = 0.05) = 0;

  /**
   * Twiddle the coefficient vector
   * @param p the coefficient vector
   * @param the target value to reach
   * @param steps the steps assumed for convergence
   * @param dt the delta time for each step
   * @param threshold the adjustment threshold
   */ 
  double twiddle(VectorXd &p, const double target, const int steps, const double dt, double threshold);
};

#endif