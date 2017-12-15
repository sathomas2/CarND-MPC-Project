#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  // Actuators
  double steer_angle;
  double throttle;
  
  // Vectors of predicted route coordinates to show in simulator 
  vector<double> mpc_ptsx;
  vector<double> mpc_ptsy;
  
  MPC();
  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double ref_v);
};

#endif /* MPC_H */
