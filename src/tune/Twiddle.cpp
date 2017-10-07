#include "Twiddle.h"
#include <iostream>

double Twiddle::twiddle(VectorXd &p, const double target, const int steps, const double dt, double threshold) {
  p.setZero();
  VectorXd pp(p.size());
  pp.setZero();
  VectorXd dp(p.size());
  dp.fill(1.);
  double best = run(pp, target, steps, dt);
  double error = 0;
  while (dp.sum() > threshold) {
    for (int i = 0; i < p.size(); i++) {
      pp[i] += dp[i];
      error = run(pp, target, steps, dt);
      if (error < best) {
        best = error;
        p = pp;
        dp[i] *= 1.1;
      } else {
        pp[i] -= 2 * dp[i];
        error = run(pp, target, steps, dt);
        if (error < best) {
          best = error;
          p = pp;
          dp[i] *= 1.1;
        } else {
          pp[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
#ifdef VERBOSE_OUT
      std::cout << "Twiddle: " << p[0] << " " << p[1] << " " << p[2] << ", " << dp[0] << " " << dp[1] << " " << dp[2] << ", Error: " << error << " " << best << std::endl;
#endif
    }
  }

  return best;
}