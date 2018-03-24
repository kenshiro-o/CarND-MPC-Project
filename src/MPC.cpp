#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 40;
double dt = 0.05;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
const double ref_v = 50;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // fg[0] stores the cost
    for(unsigned int i = 0; i < fg.size(); ++i){
      fg[i] = 0.0;
    }
    fg[0] = setCost(vars);    

    // Now we set up the constraints of the model
    // All indices are offset by 1 because we store the cost at position 0    
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];    

    // We define the rest of the constraints in relation to their value at t-1
    for(unsigned int t = 1; t < N; ++t){      
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];      

      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];      

      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> psi1 = vars[psi_start + t];      
      
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> v1 = vars[v_start + t];      

      AD<double> delta0 = vars[delta_start + t - 1];
      
      AD<double> a0 = vars[a_start + t - 1];      

      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> cte1 = vars[cte_start + t];      

      AD<double> epsi0 = vars[epsi_start + t - 1];
      AD<double> epsi1 = vars[epsi_start + t];
      
      // We can now set up the rest of the constraints
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      
      // We do psi0 - ... because in the simulator a negative value implies a right turn
      // and a positive one implies a left turn
      fg[1 + psi_start + t] = psi1 - (psi0 - (v0 / Lf) * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
                
      // AD<double> fx = poly(x0);
      AD<double> fx = coeffs[0] + coeffs[1] * x0 + coeffs[2] * (x0 * x0) + coeffs[3] * (x0 * x0 * x0);
      
      // AD<double> fprime_x = poly_derivative(x0);
      AD<double> fprime_x = coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * (x0 * x0);      
      
      AD<double> desired_psi = CppAD::atan(fprime_x);      

      fg[1 + cte_start + t] = cte1 - (fx - y0 + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - (psi0 - desired_psi + (v0 / Lf) * delta0 * dt);        
    }
  }

  private:

    // Returns the computed cost based off our variables
    AD<double> setCost(const ADvector& vars){
      AD<double> cost = 0.0;
      double d1 = 0;
      // First step is to add cte, epsi as well as velocity difference to cost
      for (unsigned int t = 0; t < N; t++) {
        cost += 1000 * CppAD::pow(vars[cte_start + t], 2);
        cost += 10000 * CppAD::pow(vars[cte_start + t] + vars[delta_start + t], 2);
        cost += 1 * CppAD::pow(vars[epsi_start + t], 2);
        cost += 1 * CppAD::pow(vars[v_start + t] - ref_v, 2);
      }

      // Then we want to minimise the use of actuators for a smoother ride
      for (unsigned int t = 0; t < N - 1; t++) {
        cost += 1000 * CppAD::pow(vars[delta_start + t], 2);
        cost += 1 * CppAD::pow(vars[a_start + t], 2);
      }

      // Finally. we want to minimise sudden changes between successive states
      for(unsigned int t = 0; t < N - 2; ++t){
        cost += 1 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        // cost += 1000 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);

        // TODO possibly take into account velocity changes
      }

      return cost;
    }

    // Simple utility function to compute f(x)
    AD<double> poly(const AD<double> &x){
      // Be very careful with 0 terms - return right away if input is 0
      if(x == 0.0){
        return 0.0;
      }

      AD<double> d = 0.0;      
      for(unsigned int i = 0; i < coeffs.size(); ++i){
        d += coeffs[i] * CppAD::pow(x, i);
      }
      return d;
    }

    // Simple utility function to compute f'(x)
    AD<double> poly_derivative(const AD<double> &x){
      // Be very careful with 0 terms - return right away if input is 0
      if(x == 0.0){
        return 0.0;
      }

      AD<double> d = 0.0;
      for(unsigned int i = 1; i < coeffs.size(); ++i){
        int exp = i - 1;
        d += i * CppAD::pow(x, exp) * coeffs[i];
      }
      return d;
    }
};


//
// MPCResult class definition implementation.
//
MPCResult::MPCResult() {}
MPCResult::~MPCResult() {}

double  MPCResult::next_steering_angle(){
  return predicted_steering_angles[0];

  double sum = 0.0;
  int steps = 6;
  for(unsigned int i = 0; i < steps; ++i){
    sum += predicted_steering_angles[i];
  }
  
  return sum / steps;
}

double MPCResult::next_throttle(){
  return predicted_throttles[0];

  cout << "NEXT THROTTLE CALC" << endl;
  double sum = 0.0;
  int steps = 6;
  for(unsigned int i = 0; i < steps; ++i){
    sum += predicted_throttles[i];
  }
  
  return sum / steps;
}

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

MPCResult MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 6 * N + 2 * (N - 1);

  // TODO: Set the number of constraints
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // non-actuator lower and upper bound values should be close to 0
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1;
    vars_upperbound[i] = 0.5;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  
  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  cout << "Solution length is " << solution.x.size() << endl;

  MPCResult res;
  res.cost = cost;
  vector<double> next_xs;
  vector<double> next_ys;  
  vector<double> next_steers;  
  vector<double> next_throttles;  
  auto solution_vector = solution.x;
  for(unsigned int j = 1; j < N; ++j){
    next_xs.push_back(solution_vector[x_start + j]);
    next_ys.push_back(solution_vector[y_start + j]);
    
    next_steers.push_back(solution_vector[delta_start + j - 1]);
    next_throttles.push_back(solution_vector[a_start + j - 1]);
  }

  // TODO smooth out steering angle and throttle
  int steps = 5;
  for(unsigned int i = 0; i < N - steps - 1; ++i){
    double sum_steer = 0.0;
    double sum_throttle = 0.0;
    for(int j = i; j < i + steps; ++j){
      sum_steer += next_steers[j];
      sum_throttle += next_throttles[j];
    }
    next_steers[i] = sum_steer / steps;
    next_throttles[i] = sum_throttle / steps;
  }

  // TODO Should we recompute v as well?

  res.predicted_xs = next_xs;
  res.predicted_ys = next_ys;
  res.predicted_steering_angles = next_steers;
  res.predicted_throttles = next_throttles;
  

  return res;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  // TODO why are delta_start and a_start not offset by 1 ??!
  // return {solution.x[x_start + 1],   solution.x[y_start + 1],
  //         solution.x[psi_start + 1], solution.x[v_start + 1],
  //         solution.x[cte_start + 1], solution.x[epsi_start + 1],
  //         solution.x[delta_start],   solution.x[a_start]};
}
