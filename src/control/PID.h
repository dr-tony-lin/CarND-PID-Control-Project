#ifndef _CONTROL_PID_H_
#define _CONTROL_PID_H_

class PID {
  bool initial = true;
  double target;     // control target
  double error;      // the error
  double error_sum;  // sum of error
  double derror;     // error derivative

  /*
  * Coefficients
  */
  double Kp;  // Proprotional control coefficient
  double Ki;  // Integral control coefficient
  double Kd;  // Derivative control coefficient

 public:
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /**
   * Set the control's target value
   */
  void setTarget(double value) { target = value; };

  /**
   * Return the control's target
   */
  double getTarget() { return target; };

  /**
   * Update the value under control
   */
  void updateValue(double value) { updateError(value - target); };

  /*
  * Initialize PID.
  */
  void init(double Kp, double Ki, double Kd);

  /**
   * Return current error
   */ 
  double getError() { return error;}

  /**
   * Return the current updated value
   */ 
  double getValue() { return target + error;}

  /*
  * Update the PID error variables given cross track error.
  * @param value the error
  */
  void updateError(double value);

  /*
  * Calculate the PID control value to apply.
  */
  double getControl();
};

#endif /* PID_H */
