#ifndef _TUNE_CARTWIDDLE_H_
#define _TUNE_CARTWIDDLE_H_

#include <math.h>
#include <random>
#include "Eigen/Dense"
#include "Twiddle.h"
#include "../control/PID.h"

#define EPSILON 1E-6

class CarTwiddle: public Twiddle {
public:
  const int STEERING_MODE = 1;
  const int ACCELERATION_MODE = 2;

private:
  static std::default_random_engine generator;
  
  double length;
  double x, y;
  double yaw;
  double velocity;
  double noise[2];
  double steering_drift;
  double max_steering = M_PI/ 4.0;
  double max_acceleration = 10;
  double max_deceleration = -20;
  double max_velocity = 34;
  int mode = STEERING_MODE;

  PID pid;

  // Random distributions
  std::normal_distribution<double> rand_a;
  std::normal_distribution<double> rand_yawd;
public:
  /**
   * Cconstructor
   * @param length length of the car
   * @param x x coordinate of the car
   * @param y y coordinate of the car
   * @param yaw the yaw angle
   * @param velocity velocity of the car alone the yaw angle
   * @param noise noise vector for acceleration, and yaw
   * @param steering_drift the steering shift
   */ 
  CarTwiddle(double length, double x, double y, double yaw, double velocity, double noise[2], double steering_drift = 0);

  /**
   * Copy constructor
   * @param another reference to another CarTwiddle to copy from
   */ 
  CarTwiddle(CarTwiddle &another);

  /**
   * Copy constructor
   * @param another pointer to another CarTwiddle to copy from
   */ 
  CarTwiddle(CarTwiddle *another);

  /**
   * Assignment operator
   * @param another reference to another CarTwiddle to assign from
   */ 
  CarTwiddle& operator=(const CarTwiddle &another);

  /**
   * Set the simulation mode, can be:
   * STEERING_MODE or ACCELERATION_MODE
   */
  void setMode(int mode);

  /**
   * Move the car
   * @param dt the time to move
   * @param steering the steering angle
   * @param acceleration the acceleration
   */ 
  virtual void move(double dt, double steering, double acceleration = 0);

  double run(const Eigen::VectorXd &t, const double target, const int steps, const double dt);
};

#endif