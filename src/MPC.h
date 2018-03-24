#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


class MPCResult {

  public:

  vector<double> predicted_xs;
  vector<double> predicted_ys;
  
  vector<double> predicted_steering_angles;
  vector<double> predicted_throttles;
  double cte;
  double cost;

  double next_steering_angle();
  double next_throttle();
  
  MPCResult();
  virtual ~MPCResult();
};


class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  MPCResult Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};


#endif /* MPC_H */
