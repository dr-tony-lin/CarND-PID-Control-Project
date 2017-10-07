#include <iostream>
#include "CarTwiddle.h"

using namespace std;
using Eigen::VectorXd;

default_random_engine CarTwiddle::generator;

double normalizeAngle(double a) {
  while (a >= M_PI) a -= 2. * M_PI;
  while (a < -M_PI) a += 2. * M_PI;
  return a;
}

CarTwiddle::CarTwiddle(double length, double x, double y, double yaw,
                       double velocity, double noise[2], double steering_drift) {
  this->length = length;
  this->x = x;
  this->y = y;
  this->yaw = yaw;
  this->velocity = velocity;
  this->noise[0] = noise[0];
  this->noise[1] = noise[1];
  this->steering_drift = steering_drift;
  rand_a = normal_distribution<double>(0, noise[0]);
  rand_yawd = normal_distribution<double>(0, noise[1]);
}

CarTwiddle::CarTwiddle(CarTwiddle &another) {
  *this = another;
}

CarTwiddle::CarTwiddle(CarTwiddle *another) {
  *this = *another;
}

CarTwiddle& CarTwiddle::operator=(const CarTwiddle &another) {
  length = another.length;
  x = another.x;
  y = another.y;
  yaw = another.yaw;
  velocity = another.velocity;
  noise[0] = another.noise[0];
  noise[1] = another.noise[1];
  steering_drift = another.steering_drift;
  rand_a = normal_distribution<double>(0, noise[0]);
  rand_yawd = normal_distribution<double>(0, noise[1]);
  return *this;
}

void CarTwiddle::setMode(int mode) {
  assert(mode ==1 || mode == 2);
  this->mode = mode;
}

// Implements a simple car motion model
void CarTwiddle::move(double dt, double steering, double acceleration) {
  // clamp the steering angle
  if (steering > max_steering) steering = max_steering;
  if (steering < -max_steering) steering = -max_steering;

  // perturb the acceleration and steering angle with gaussian noise
  acceleration += rand_a(generator);
  steering += rand_yawd(generator) + steering_drift;

  double dist = (velocity + acceleration * dt / 2) * dt;
  // Compute turing angle from steering angle
  double turn = tan(steering) * dist / length;
  double new_yaw = normalizeAngle(yaw + turn);

  if (fabs(turn) > EPSILON) {  // turn is not 0
    // Compute the turn radius
    double radius = dist / turn;
    // update x and y
    x += radius * (sin(new_yaw) - sin(yaw));
    y += radius * (cos(yaw) - cos(new_yaw));

  #ifdef VERBOSE_OUT
    cout << "Move: " << turn << " " << yaw << " " << new_yaw << " " << dist << " " << steering << " " << radius << " " << velocity << endl;
  #endif
  } else {  // turn is 0
    // update x and y
    x += dist * cos(yaw);
    y += dist * sin(yaw);
  }

  // update velocity
  velocity += acceleration * dt;
  // update yaw
  yaw = new_yaw;
}

double CarTwiddle::run(const VectorXd &p, const double target, const int steps, const double dt) {
  // Backup the original settings
  CarTwiddle origin(*this);
  // Initialize PID
  pid.init(p[0], p[1], p[2]);
  pid.setTarget(target);
  double error = 0;
#ifdef VERBOSE_OUT
  cout << "Coeff: " << p[0] << " " << p[1] << " " << p[2] << endl;
#endif
  for (int i = 0; i < 2 * steps; i++) {
      double err = pid.getError();
      if (i >= steps) { // compute squared sum of error
        error += err*err;
      }
      // update PID value
      pid.updateValue(mode == STEERING_MODE? y: velocity);
      // Get new PID control value
      double control = pid.getControl();
      // Apply control value to move the car
      mode == STEERING_MODE? move(dt, control, 0): move(0.05, 0, control);
#ifdef VERBOSE_OUT
      if (i <= steps) {
        cout << "Car moved with PID: " << control << ", " << x << " " << y << " " << yaw << " " << velocity << endl;
      } else {
        cout << "Car moved with PID: " << control << ", " << x << " " << y << " " << yaw << " " << velocity << " " << error / (i - steps) << endl;
      }
#endif
  }

  *this = origin;
  error /= steps;
#ifdef VERBOSE_OUT
  cout << "Car out: " << x << " " << y << " " << yaw << " " << velocity << endl;
#endif
  return error;
}