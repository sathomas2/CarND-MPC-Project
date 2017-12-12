#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 50;
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

// Velocity goal
double ref_v = 20;

// State and actuator variables are stored in a single vector, so to make life
// easier establish when each one starts
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
// delta and acceleration have one less because there is no value for each at timestamp 0
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
    //
    // Cost is stored at 'fg[0]' and here I'll store some of the cost here at timestamp t = 0 before the loop.
    // Remainder of cost will be added in loop below.
    fg[0] = 0;
    // Squared cross-track error
    fg[0] += CppAD::pow(vars[cte_start], 2);
    // Squared heading error
    fg[0] += CppAD::pow(vars[epsi_start], 2);
    // Squared difference of current velocity and reference velocity
    fg[0] += 0.1*CppAD::pow(vars[v_start] - ref_v, 2);
    
    // Initialize constraints. Add 1 to each of the starting indices because cost is at index 0.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    
    // Rest of constraints.
    for (int t=1; t<N; ++t) {
      AD<double> x1 = vars[x_start + t];
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y1 = vars[y_start + t];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v1 = vars[v_start + t];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi1 = vars[epsi_start + t];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      AD<double> delta = vars[delta_start + t - 1];
      AD<double> a = vars[a_start + t - 1];
      // y-value according to fitted polynomial
      AD<double> f = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0;
      
      // arctangent of f'(x)
      AD<double> psides = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0);
      
      // Cost function
      // First the ones above for rest of timestamps.
      fg[0] += CppAD::pow(cte1, 2);
      fg[0] += CppAD::pow(epsi1, 2);
      fg[0] += 0.1*CppAD::pow(v1 - ref_v, 2);
      //fg[0] += CppAD::pow(v1 - v0, 2);
      // Squared delta and squared acceleration so vehicle doesn't turn or speed up too aggressively
      //fg[0] += CppAD::pow(delta, 2);
      //fg[0] += CppAD::pow(a, 2);
      // Same idea as above but squared difference between current and previous timestamp to help
      // make control decisions consistent.
      if (t<N-1) {
        fg[0] += CppAD::pow(vars[a_start + t] - a, 2);
        // Penalize delta change heavily to combat overshooting
        fg[0] += 200*CppAD::pow(vars[delta_start + t] - delta, 2);
      }

      // Rest of cost constraints, using vehicle model, so that this value of fg always equals 0.
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1  - (psi0 - v0/Lf * delta * dt);
      fg[1 + v_start + t] = v1 - (v0 + a * dt);
      fg[1 + cte_start + t] = cte1 - ((f - y0) + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides) - v0/Lf * delta * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

void MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  typedef CPPAD_TESTVECTOR(double) Dvector;
  // Break out state for readability
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  // Number of model variables (includes both states and inputs).
  size_t n_vars = 6 * N + 2 * (N - 1);
  // Number of constraints
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  // Initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set all non-actuators lower and upper limits to a large number
  for (int i=0; i<delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  // Set lower and upper limits of delta (i.e., steering angle) to -1 and 1
  for (int i=delta_start; i<a_start; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  // Set lower and upper limits of acceleration (i.e., throttle) to -1 and 1
  for (int i=a_start; i<n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
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

  // Check solution values
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Set current steer_angle and throttle
  steer_angle = solution.x[delta_start];
  throttle = solution.x[a_start];
  mpc_psi = solution.x[psi_start + 1];
  mpc_v = solution.x[v_start + 1];
  mpc_cte = solution.x[cte_start + 1];
  mpc_epsi = solution.x[epsi_start + 1];
  //clear old ones, then set x and y predictions to plot within simulator
  mpc_ptsx.clear();
  mpc_ptsy.clear();
  for (int i=1; i<N; ++i) {
    mpc_ptsx.push_back(solution.x[x_start + i]);
    mpc_ptsy.push_back(solution.x[y_start + i]);
  }
}
