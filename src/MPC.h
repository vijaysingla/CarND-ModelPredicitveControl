#ifndef MPC_H
#define MPC_H
extern double dt;
//extern int N;
extern double w1;
extern double w2;
extern double w3;
extern double w4;
extern double w5;
extern double ref_v;
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
