#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Timestep length and duration
size_t N = 20;
double dt = 0.05;

// This value was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// State and actuator variables are stored in a single vector to make life
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

// Set weights for each part of the cost function to make it easier to tune.
double k_epsi = 32;
double k_depsi = 25;
double k_cte = 0.05;
double k_dcte = 0.375;
double k_delta = 350;
double k_ddelta = 100000;
double k_v = 0.25;
double k_a = 1.0;
double k_da = 50.0;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  // Velocity goal
  double ref_v;
  FG_eval(Eigen::VectorXd coeffs, double ref_v) {
    this->coeffs = coeffs;
    this->ref_v = ref_v;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    //
    // Cost is stored at 'fg[0]' and here I'll store some of the cost here at timestamp t=0.
    // Remainder of cost will be added in loop below.
    fg[0] = 0;
    // Squared cross-track error
    fg[0] += k_cte*CppAD::pow(vars[cte_start], 2);
    // Squared heading error
    fg[0] += k_epsi*CppAD::pow(vars[epsi_start], 2);
    // Squared difference of current velocity and reference velocity
    fg[0] += k_v*CppAD::pow(vars[v_start] - ref_v, 2);
    
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
      fg[0] += k_cte*CppAD::pow(cte1, 2);
      fg[0] += k_epsi*CppAD::pow(epsi1, 2);
      fg[0] += k_v*CppAD::pow(v1 - ref_v, 2);
      // Squared delta cte and delta epsi to combat oscillation
      fg[0] += k_dcte*CppAD::pow(cte1 - cte0, 2);
      fg[0] += k_depsi*CppAD::pow(epsi1 - epsi0, 2);
      // Squared acceleration and delta so vehicle doesn't speed up or turn too aggressively
      fg[0] += k_a*CppAD::pow(a, 2);
      fg[0] += k_delta*CppAD::pow(delta, 2);
      // Same idea as above but squared difference between current and previous timestamp to help
      // make control decisions consistent.
      if (t<N-1) {
        fg[0] += k_da*CppAD::pow(vars[a_start + t] - a, 2);
        // Penalize delta change heavily to combat overshooting
        fg[0] += k_ddelta*CppAD::pow(vars[delta_start + t] - delta, 2);
      }

      // Constraints, using vehicle model, so that this value of fg always equals 0.
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
MPC::MPC() {
}
MPC::~MPC() {}

void MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs,  double ref_v) {
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
  // Set lower and upper limits of delta (i.e., steering angle) to -25 and 25 degrees (in radians)
  for (int i=delta_start; i<a_start; ++i) {
    vars_lowerbound[i] = -0.43633231;
    vars_upperbound[i] = 0.43633231;
  }
  // Set lower and upper limits of acceleration (i.e., throttle) to -0.1 (no need to brake
  // a lot on this track) and 1
  for (int i=a_start; i<n_vars; ++i) {
    vars_lowerbound[i] = -0.1;
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
  FG_eval fg_eval(coeffs, ref_v);
  
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // Set the solver to have a maximum time limit of 0.5 seconds.
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
  
  // Set current steer_angle (divide by 25 degrees in radians so between [-1,1] and throttle
  steer_angle = solution.x[delta_start] / 0.43633231;
  throttle = solution.x[a_start];
  
  //clear old ones, then set x and y predictions to plot within simulator
  mpc_ptsx.clear();
  mpc_ptsy.clear();
  for (int i=1; i<N-1; ++i) {
    mpc_ptsx.push_back(solution.x[x_start + i]);
    mpc_ptsy.push_back(solution.x[y_start + i]);
  }
  
  // The below helps to see how different parts of the cost function affect total cost.
  // Can uncomment to help during tuning.
  //double cost_cte = 0;
  //double cost_epsi = 0;
  //double cost_v= 0;
  //double cost_dcte = 0;
  //double cost_depsi = 0;
  //double cost_a = 0;
  //double cost_delta = 0;
  //double cost_da = 0;
  //double cost_ddelta = 0;
  //for (int i=0; i<N; ++i) {
    //cost_cte += k_cte*CppAD::pow(solution.x[cte_start + i], 2);
    //cost_epsi += k_epsi*CppAD::pow(solution.x[epsi_start + i], 2);
    //cost_v += k_v*CppAD::pow(solution.x[v_start + i] - ref_v, 2);
    //if (i < N-1) {
      //cost_dcte += k_dcte*CppAD::pow(solution.x[cte_start + i + 1] - solution.x[cte_start + i], 2);
      //cost_depsi += k_depsi*CppAD::pow(solution.x[epsi_start + i + 1] - solution.x[epsi_start + i], 2);
      //cost_a += k_a*CppAD::pow(solution.x[a_start + i], 2);
      //cost_delta += k_delta*CppAD::pow(solution.x[delta_start + i], 2);
    //}
    //if (i < N-2) {
      //cost_da += k_da*CppAD::pow(solution.x[a_start + i + 1] - solution.x[a_start + i], 2);
      //cost_ddelta += k_ddelta*CppAD::pow(solution.x[delta_start + i + 1] - solution.x[delta_start + i], 2);
    //}
  //}
  //double total_cost = cost_cte+cost_epsi+cost_v+cost_dcte+cost_depsi+cost_a+cost_delta+cost_da+cost_ddelta;
  //double per_cte = cost_cte/total_cost;
  //double per_epsi = cost_epsi/total_cost;
  //double per_v = cost_v/total_cost;
  //double per_dcte = cost_dcte/total_cost;
  //double per_depsi = cost_depsi/total_cost;
  //double per_a = cost_a/total_cost;
  //double per_delta = cost_delta/total_cost;
  //double per_da = cost_da/total_cost;
  //double per_ddelta = cost_ddelta/total_cost;
  //cout << "check COST " << total_cost << endl;
  //cout << "% epsi " << per_epsi << endl;
  //cout << "% depsi " << per_depsi << endl;
  //cout << "% cte " << per_cte << endl;
  //cout << "% dcte " << per_dcte << endl;
  //cout << "% delta " << per_delta << endl;
  //cout << "% ddelta " << per_ddelta << endl;
  //cout << "% v " << per_v << endl;
  //cout << "% a " << per_a << endl;
  //cout << "% da " << per_da << endl;
}
