#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Model Predictive Control uses an optimizer to find the control inputs and minimize the cost function
// We only execute the very first set of control inputs; brings vehicle to a new state and process is repeated
// The model, cost and constraints comprise the solver: Ipopt

// TODO: Set the timestep length and duration
// Prediction horizon is the duration of future predictions (prediction_horizon = N * dt)
// Prediction horizon should be as large as possible, but no more than a few seconds
// Number of time steps in the horizon
size_t N = 10; // 5 goes nowhere; 8 makes front tail of MPC trajectory tails off to right often
// How much times elapses between actuations in seconds; smaller is better
double dt = 0.1;

// Lf value assumes the model presented in the classroom is used.
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on flat terrain.
// Lf was tuned until the the radius formed by the simulating model
// presented in the classroom matched the previous radius.
//
// This is the length from front to the center of gravity; that has a similar radius.
const double Lf = 2.67;

// Reference, or desired states for each
double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 130;

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

    // The cost is stored in the first element of 'fg'
    // Any additions to the cost should be added to 'fg[0]'
    fg[0] = 0;

    // Define weights for different terms of objective
    const double cte_w = 1500;
    const double epsi_w = 2000;
    const double v_w = 1; // 100 can't make sharpest turn
    const double actuator_w = 10;
    const double change_steer_w = 1000; // 200 pretty good, 20: can't make sharpest curve
    const double change_accel_w = 10; // 10 good

    // Reference State Cost
    // The part of the cost based on the reference state
    for (size_t t = 0; t < N; ++t) {
      // High coeff = more attention paid to variables (by the cost function)
      fg[0] += cte_w * CppAD::pow(vars[cte_start + t] - ref_cte, 2);  // cross track error
      fg[0] += epsi_w * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);  // orientation error
      fg[0] += v_w * CppAD::pow(vars[v_start + t] - ref_v, 2);  // velocity error
    }

    // Minimize the use of actuators
    // Minimize change-rate; constrain erratic control inputs
    // Goal is smooth turning and smooth accel/decel
    for (size_t t = 0; t < N - 1; ++t) {
      fg[0] += actuator_w * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += actuator_w * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations
    // Make control decisions more consistent/smoother
    // The next control input should be similar to the current one
    for (size_t t = 0; t < N - 2; ++t) {
      fg[0] += change_steer_w * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += change_accel_w * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Setup Constraints
    //
    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (size_t t = 1; t < N; ++t) {
      // To use CppAD effectively, we have to use its types instead of standard lib types
      // Standard math operations are overloaded so calling +,-,*,/ will work if using CppAD<double>
      // The state at time t:
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // The state at time t + 1:
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // Only consider the actuation at time t
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
      // desired psi
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);

      // The idea here is to constrain this value to be 0.
      // CppAD can compute derivatives and pass these to the solver.
      //
      // Recall the equations for the model:
      // x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      // y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      // psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      // v[t] = v[t-1] + a[t-1] * dt
      // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
      //
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;

  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators are a
  // 2-element vector and there are 10 timesteps. The number of variables is:
  // 4 * 10 + 2 * 9
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  size_t n_vars = N * 6 + (N -1) * 2;

  // TODO: Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // TODO: Set lower and upper limits for variables.

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.

  // Set all non-actuators upper and lower limits
  // to the max negative and positive values.
  for (size_t i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -numeric_limits<float>::max();
    vars_upperbound[i] = +numeric_limits<float>::max();
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians)
  double max_degrees = 25;
  double max_radians = max_degrees * M_PI / 180;

  //std::cout << "lower" << -max_radians << "\n";
  //std::cout << "upper" << +max_radians << "\n";
  for (size_t i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -max_radians;
    vars_upperbound[i] = +max_radians;
  }

  // Acceleration/deceleration upper and lower limits
  for (size_t i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }


  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (size_t i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Force solver to start from current state in optimization
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

  // Ipopt is the tool used to optimize the control inputs; it's able to find locally optimal values (non-liner problems)
  // It keeps the constraints set directly to the actuators and the constraints defined by the vehicle model.
  // Ipopt requires we give it the jacobians and hessians directly; it does not compute them for us.
  // CppAD library is used for automatic differentiation; no need to manually compute derivatives

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

  // TODO: Return the first actuator values. The variables can be accessed with `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result;

  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for (size_t i = 0; i < N - 1; i++) {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }

  return result;
}
