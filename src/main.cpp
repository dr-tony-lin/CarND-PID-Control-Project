#include <uWS/uWS.h>
#include <iostream>
#include <math.h>
#include "json.hpp"
#include "control/PID.h"
#include "utils/Reducer.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

/**
 * Simple logic to adjust speed according to steering angle
 * @param angle the angle
 * @param max the maximal speed permitted
 */ 
double computeSpeedTarget(double angle, double max) {
  double y = fabs(angle);
  if (y < 0.02) return max;
  if (y < 0.075 ) return std::fmin(max,95);
  if (y < 0.08 ) return std::fmin(max, 90);
  if (y < 0.085 ) return std::fmin(max, 85);
  if (y < 0.0875 ) return std::fmin(max, 75);
  if (y < 0.1 ) return std::fmin(max, 75);
  if (y < 0.15 ) return std::fmin(max, 65);
  if (y < 0.175 ) return std::fmin(max, 40);
  if (y < 0.2 ) return std::fmin(max, 35);
  if (y < 0.3 ) return std::fmin(max, 30);
  if (y < 0.5 ) return std::fmin(max, 25);
  return std::fmin(max, 20);
}

/**
 * Simple logic to compute throttle from acceleration.
 * @param accel the acceleration to reach
 * @param target the target speed
 * @param max_accel the maximal acceleration
 * @param max_decel the maximal deceleration
 */ 
double computeThrottle(double accel, double target, double max_accel, double max_decel) {
  // the throttle to keep if no accel, divide by 10 is a rough estimate of target speed,
  // and throttle to keep for the speed
  double keep = target / 10.0;
  if (accel >= 0) { // acceleration
    if (accel < 0.001) { // small acceleration, keep the mimimal throttle
      return keep;
    }
    else { // otherwise compute the throttle
      return std::fmin(1, keep + (1 - keep) * accel / max_accel); // max accel is 5
    }
  }
  else {
    if (accel > -20) { // deceleration
      return -1;
    } else if (accel > -10) { // deceleration
      return -0.95 + (1 - 0.95) * accel / max_decel;
    } else if (accel > -5) { // deceleration
      return -0.9 + (1 - 0.9) * accel / max_decel;
    }
    return -0.85 + (1 - 0.85) * accel / max_decel;
  }
}

/**
 * define a function to return size of array, C++ compiler can infer the template
 * parameters when this template is used in a correct context
 * @param arr reference to an array of T of SIZE
 */ 
template<size_t SIZE, class T> inline size_t array_size(T (&arr)[SIZE]) {
  return SIZE;
}

int main(int argc, char* argv[])
{
  uWS::Hub h;

#ifdef APPLY_LOWPASS_FILTER
  double s_coeffs[3] = {0.3, 4.5, 0};
#else
  double s_coeffs[3] = {0.108, 3.52, 0}; // {0.119058, 3.23448, 0};
#endif
  // ./tune -steps 1500 -dt 0.01 -y 1 -speed 100
  // Speed: 50, Steering: 0.0595525, 3.29189, 0, Error: 4.36856e-50
  // Speed: 60, Steering: 0.362673, 6.29715, 0, Error: 4.19443e-163
  // Speed: 70, Steering: 0.0771785, 2.63884, 0, Error: 1.7917e-81
  // Speed: 80, Steering: 0.0790413, 2.31635, 0, Error: 4.48461e-95
  // Speed: 90, Steering: 0.116606, 2.43286, 0, Error: 6.02029e-135
  // Speed: 100, Steering: 0.178943, 2.60117, 0, Error: 8.21503e-200
  double v_coeffs[3] = {13.5795, -11.4359, 0};
  // ./tune -steps 1000 -dt 0.01 -y 1 -speed 80 -accel -target 5
  // Speed: 30, Acceleration coefficient: 11.4359, -4.641, 0, Error: 4.54384e-28
  // Speed: 40, Acceleration coefficient: 11.4359, -4.641, 0, Error: 4.54384e-28
  // Speed: 50, Acceleration coefficient: 11.4359, -6.1051, 0, Error: 4.54384e-28
  // Speed: 60, Acceleration coefficient: 11.4359, -7.71561, 0, Error: 4.54384e-28
  // Speed: 70, Acceleration coefficient: 13.5795, -11.4359, 0, Error: 8.07794e-28
  // Speed: 80, Acceleration coefficient: 24.5227, -21.3843, 0, Error: 2.01948e-28

  double max_speed = 100;
  double max_accel = 8;
  double max_decel = -20;

  // Process command line options
  for (int i = 1; i < argc; i++) {
    if (std::string((argv[i])) == "-s") { // steering PID coefficients
      if (sscanf(argv[++i], "%lf", &s_coeffs[0]) != 1) {
        std::cerr << "Invalid yaw PID coefficient: " << argv[i] << std::endl;
        exit(-1);
      }
      if (sscanf(argv[++i], "%lf", &s_coeffs[1]) != 1) {
        std::cerr << "Invalid yaw PID coefficient: " << argv[i] << std::endl;
        exit(-1);
      }
      if (sscanf(argv[++i], "%lf", &s_coeffs[2]) != 1) {
        std::cerr << "Invalid yaw PID coefficient: " << argv[i] << std::endl;
        exit(-1);
      }
    } else if (std::string((argv[i])) == "-v") { // vecocity coefficient
      if (sscanf(argv[++i], "%lf", &v_coeffs[0]) != 1) {
        std::cerr << "Invalid veclocity PID coefficient: " << argv[i] << std::endl;
        exit(-1);
      }
      if (sscanf(argv[++i], "%lf", &v_coeffs[1]) != 1) {
        std::cerr << "Invalid veclocity PID coefficient: " << argv[i] << std::endl;
        exit(-1);
      }
      if (sscanf(argv[++i], "%lf", &v_coeffs[2]) != 1) {
        std::cerr << "Invalid veclocity PID coefficient: " << argv[i] << std::endl;
        exit(-1);
      }
    } else if (std::string((argv[i])) == "-max_speed") { // maximum speed
      if (sscanf(argv[++i], "%lf", &max_speed) != 1) {
        std::cerr << "Invalid y: " << argv[i] << std::endl;
        exit(-1);
      }
    }
  }

  // PID controller for steering
  PID pid_steering;
  pid_steering.init(s_coeffs[0], s_coeffs[1], s_coeffs[2]);

  // PID controller for acceleration
  PID pid_accel;
  pid_accel.init(v_coeffs[0], v_coeffs[1], v_coeffs[2]); 

  // Use the mean of past 5 readings to determine the speed of the vehicle
  Reducer<double> angleReducer(5);

  // Use the mean of past 40 readings to determine the curverature
  Reducer<double> curvReducer(35);
  Reducer<double> speedReducer(35);

  h.onMessage([&pid_steering, &pid_accel, &angleReducer, &curvReducer, &speedReducer, max_speed, max_accel, max_decel]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>()); 
          double speed = std::stod(j[1]["speed"].get<std::string>());
          // what is the steering angle from the simulator? and the unit, is it the yaw instead?
          // As it is very off from values sent to the simulator 
          double angle = std::stod(j[1]["steering_angle"].get<std::string>()); 
          double steer_value;
          
          // UPdate the steering PID error
          pid_steering.updateError(cte);
          // Get the PID control value,and normalize it to [-1, 1] range
          steer_value = pid_steering.getControl() / (0.25 * M_PI);
          
          // Add the angle to the reducer
          angleReducer.push(fabs(angle));
          
#ifdef APPLY_LOWPASS_FILTER
          curvReducer.push(angle);
          speedReducer.push(speed);
          // Apply a low pass filter once we have enough samples
          if (curvReducer.getNumberOfSamplesReceived() >= 200) {
            // Comoute the average turns from the time averaged angles
            // steering offset = atan(turn*length/velocity)
            double turn = curvReducer.sum() / curvReducer.size();
            turn *= 2.5/speedReducer.mean<double>(); // 2.5m is the vehicle length
            double steer_curve = atan(turn);
            steer_value -= steer_curve;
#ifdef VERBOSE_OUT
            std::cout << "curve adjustment: " << steer_curve << std::endl;
#endif
          }
#endif
          // Clamp steering value to [-1, 1] range
          if (steer_value > 1) {
            steer_value = 1;
          }
          if (steer_value < -1) {
            steer_value = -1;
          }

          // Get the average of the past angle readings
          double reduced_angle = angleReducer.mean<double>();

          // Determing the speed from the mean angle
          double targetSpeed = computeSpeedTarget(deg2rad(reduced_angle), max_speed);
          // The scceleration or deceleration
          double speed_adjustment = targetSpeed - speed;
          // Update the acceleration PID error
          pid_accel.updateError(-speed_adjustment);
          // Compute the acceleration/deceleration, 1 second to reach the target
          double accelDecel = pid_accel.getControl() / 1.0;

          // Clamp the acceleration to [max_decel, max_accel]
          if (accelDecel > max_accel) {
            accelDecel =  max_accel;
          }
          else if (accelDecel < max_decel) {
            accelDecel = max_decel;
          }

          double throttle = computeThrottle(accelDecel, targetSpeed, max_accel, max_decel);
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " current: " << angle << "(" << deg2rad(angle) << ","
                    << reduced_angle << ")" << std::endl;
          std::cout << "Speed adjustment: " << speed_adjustment << ", current: " << speed << ", accel: " << accelDecel << std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    //ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
