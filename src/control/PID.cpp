#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::init(double Kp, double Kd, double Ki) {
  setPID(Kp, Kd, Ki);
  error = 0;
  error_sum = 0;
  derror = 0;
  initial = true;
}

void PID::setPID(double Kp, double Kd, double Ki) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::updateError(double value) {
  if (initial) {
    initial = false;
    error = value;
  }

  derror = value - error;
  error_sum += value;
  error = value;
}

double PID::getControl() {
  return -Kp * error - Kd * derror - Ki * error_sum;
}

