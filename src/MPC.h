#ifndef MPC_H
#define MPC_H

/*
 * These commented extern varriables were used when these are being passed as arguments through main function
 * for tuning
 */
//extern double dt;
//extern double w1;
//extern double w2;
//extern double w3;
//extern double w4;
//extern double ref_v;

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
