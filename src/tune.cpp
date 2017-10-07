#include <math.h>
#include <iostream>
#include "Eigen/Dense"
#include "tune/CarTwiddle.h"

using Eigen::VectorXd;

int main(int argc, char* argv[]) {
  double noise[2] = {0.3, 0.003}; // some small random noise
  double dt = 0.1;
  double drift = 0.;
  int steps = 100;
  double velocity = 60;
  double target = 0;
  double length = 2;
  bool throttle = false;

  // Process command line options
  for (int i = 1; i < argc; i++) {
    if (std::string((argv[i])) == "-dt") { // st
      if (sscanf(argv[++i], "%lf", &dt) != 1 || dt <= 0) {
        std::cerr << "Invalid dt: " << argv[i] << std::endl;
        exit(-1);
      }
    } else if (std::string((argv[i])) == "-steps") { // steps
      if (sscanf(argv[++i], "%d", &steps) != 1 || steps <= 0) {
        std::cerr << "Invalid steps: " << argv[i] << std::endl;
        exit(-1);
      }
    } else if (std::string((argv[i])) == "-speed") { // velocity
      if (sscanf(argv[++i], "%lf", &velocity) != 1 || velocity <= 0) {
        std::cerr << "Invalid speed: " << argv[i] << std::endl;
        exit(-1);
      }
    } else if (std::string((argv[i])) == "-len") { // car length
      if (sscanf(argv[++i], "%lf", &length) != 1 || length <= 0) {
        std::cerr << "Invalid distance between center of mass and fromt wheel: " << argv[i] << std::endl;
        exit(-1);
      }
    } else if (std::string((argv[i])) == "-target") { // target value
      if (sscanf(argv[++i], "%lf", &target) != 1) {
        std::cerr << "Invalid target: " << argv[i] << std::endl;
        exit(-1);
      }
    } else if (std::string((argv[i])) == "-drift") { // steering drift
      if (sscanf(argv[++i], "%lf", &drift) != 1) {
        std::cerr << "Invalid drift: " << argv[i] << std::endl;
        exit(-1);
      }
    } else if (std::string((argv[i])) == "-throttle") { // tune speed acceleration
      throttle = true;
    } else {
      std::cerr << "Unknown option: " << argv[i] << std::endl;
      exit(-1);
    }
  }

  CarTwiddle car(2.0, 0, 1, 0, velocity, noise, drift);
  if (throttle) {
    car.setMode(car.ACCELERATION_MODE);
    VectorXd accel_p(3);
    car.twiddle(accel_p, target, steps, dt, 0.0001);
    std::cout << "Acceleration coefficient: " << accel_p[0] << ", " << accel_p[1] << ", " << accel_p[2] << std::endl;
  }
  else {
    car.setMode(car.STEERING_MODE);
    VectorXd steering_p(3);
    double error = car.twiddle(steering_p, target, steps, dt, 0.0001);
    std::cout << "Error: " << error << ", Steering coefficients: " << steering_p[0] << ", " << steering_p[1] << ", " << steering_p[2] << std::endl;  
  }
}