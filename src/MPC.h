#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  double steer_angle;
  double throttle;
  double mpc_psi;
  double mpc_v;
  double mpc_cte;
  double mpc_epsi;
  vector<double> mpc_ptsx;
  vector<double> mpc_ptsy;
  
  MPC();
  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
