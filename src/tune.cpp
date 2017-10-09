#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "tune/CarTwiddle.h"
#ifdef PLOT_WITH_MATPLOT
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

using Eigen::VectorXd;

int main(int argc, char* argv[]) {
  double noise[2] = {0.0, 0.0}; // no noise
  double dt = 0.1; // delta time
  double drift = 0.; // steering drift
  int steps = 100; // number of simulation steps
  double velocity = 100; // velocity
  double target = 0; // target value
  double y = 1; // y coordinate
  double length = 2.5; // vehicle length
  bool accel = false; // true for acceleration mode, false for steering mode

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
    } else if (std::string((argv[i])) == "-y") { // current value
      if (sscanf(argv[++i], "%lf", &y) != 1 || steps <= 0) {
        std::cerr << "Invalid y: " << argv[i] << std::endl;
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
    } else if (std::string((argv[i])) == "-accel") { // tune speed acceleration
      accel = true;
    } else {
      std::cerr << "Unknown option: " << argv[i] << std::endl;
      exit(-1);
    }
  }

  for (int v = 30; v <= velocity; v += 10) {
    CarTwiddle car(length, 0, y, 0, v * 1.61 * 1000 / 3600.0, noise, drift);

    VectorXd steering_p(3);
    VectorXd accel_p(3);
    if (accel) {
      car.setMode(car.ACCELERATION_MODE);
      double error = car.twiddle(accel_p, v + target * 1.61 * 1000 / 3600.0, steps, dt, 0.0001);
      std::cout  << "Speed: " << v << ", Acceleration coefficient: " << accel_p[0] << ", " << accel_p[1] << ", " << accel_p[2] << ", Error: " << error << std::endl;
    }
    else {
      car.setMode(car.STEERING_MODE);
      double error = car.twiddle(steering_p, target, steps, dt, 0.0001);
      std::cout << "Speed: " << v << ", Steering: " << steering_p[0] << ", " << steering_p[1] << ", " << steering_p[2] << ", Error: " << error << std::endl; 
    }
#ifdef PLOT_WITH_MATPLOT
    std::vector<double> x_trajectory{};
    std::vector<double> y_trajectory{};
    if (accel) {
      car.run(accel_p, v + target * 1.61 * 1000 / 3600.0, steps, dt, &x_trajectory, &y_trajectory);
    } else {
      car.run(steering_p, target, steps, dt, &x_trajectory, &y_trajectory);
    }
    std::string plot_specs[] = {"b", "r", "g", "c", "y", "m", "k", "w"};
    plt::plot(x_trajectory, y_trajectory, plot_specs[(v/10-3)%8]);
    plt::show();
#endif
  }
}

//./tune -steps 1000 -dt 0.01 -y 1 -speed 100
//./tune -steps 1000 -dt 0.01 -y 1 -speed 100 -accel -target 10