#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
int N = 10;
double dt ;
 double ref_v;
 double w1;
 double w2;
 double w3;
 double w4;
 double w5;

 // Reference velocity defined
 //double ref_v = 20;

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

// Defining the start value for the states which will be present in vector vars,so that its easy to
// index the values
int x_start =0;
int y_start = x_start+N;
int psi_start = y_start +N;
int v_start = psi_start +N;
int cte_start = v_start +N;
int epsi_start = cte_start +N ;
int delta_start = epsi_start +N;
int a_start = delta_start +N -1 ;



class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
	  fg[0] = 0;
	  for (int t =0; t<N; t++)
	  {
		  // cost is based on error between cross track error and ref cross track error
		  // here ref cross track error is assumed to be zero
		  fg[0] += w1*CppAD::pow(vars[cte_start+t],2);
		  // cost is based on error between epsi error and ref epsi error
		  // here epsi error is assumed to be zero
		  fg[0] +=w2*CppAD::pow(vars[epsi_start+t],2);
		  // cost is also based on error between velocity and ref velocity
		  fg[0] += 1*CppAD::pow(vars[v_start + t] - ref_v, 2);
	  }
	  for (int t=0; t<N-1; t++)
		  // penalising the use of actuators
	  {
		  fg[0] +=w3*CppAD::pow(vars[delta_start + t], 2);
		  fg[0] +=w4*CppAD::pow(vars[a_start + t], 2);

	  }
	  // penalising the derivatives of actuators
	  // helps in smoothen the transiitons
	  for (int t = 0; t < N - 2; t++)
	  {
	        fg[0] += 100*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
	        fg[0] += 100*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
	  }

	  // Constraints Set Up
	  // Intial condition constriants
	  fg[1+x_start] =  vars[x_start];
	  fg[1 + y_start] = vars[y_start];
	  fg[1 + psi_start] = vars[psi_start];
	  fg[1 + v_start] = vars[v_start];
	  fg[1 + cte_start] = vars[cte_start];
	  fg[1 + epsi_start] = vars[epsi_start];

	  // Constraints occuring due to model state matix representation
	  for (int t = 1; t < N; t++)
	  {
	        // The state at time t+1 .
	        AD<double> x1 = vars[x_start + t];
	        AD<double> y1 = vars[y_start + t];
	        AD<double> psi1 = vars[psi_start + t];
	        AD<double> v1 = vars[v_start + t];
	        AD<double> cte1 = vars[cte_start + t];
	        AD<double> epsi1 = vars[epsi_start + t];

	        // The state at time t.
	        AD<double> x0 = vars[x_start + t - 1];
	        AD<double> y0 = vars[y_start + t - 1];
	        AD<double> psi0 = vars[psi_start + t - 1];
	        AD<double> v0 = vars[v_start + t - 1];
	        AD<double> cte0 = vars[cte_start + t - 1];
	        AD<double> epsi0 = vars[epsi_start + t - 1];

	        AD<double> delta0 = vars[delta_start + t -1];
	        AD<double> a0 = vars[a_start + t - 1];

	      // Consideration for latency
	        // 100 ms latency  = 0.1 seconds
	        // dt = 0.05 ,time horizon = dt*N = 0.5 sec
	        // it means  for every 3rd  point out of N  , acceleration  = 1st point acceleration
	        if (t > 2)
	        {
	        	delta0 = vars[delta_start + t -3];
	        	a0 = vars[a_start + t - 3];
	        }

	        // Considering the actuation at time t.
	        // for 3 order polynomial  , y = a0 +a1*x+a2*x^2+a3*x^3
	        AD<double> f0 = coeffs[0] + coeffs[1] * x0+coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
	        // derivative of 3rd orde poly is a1 +2*a2*x +3*a3*x^2
	        AD<double> psides0 = CppAD::atan(coeffs[1]+2*coeffs[2]*x0+3*coeffs[3]*x0*x0);


	        // Model State Repsentation
	        // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
	        // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
	        // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
	        // v_[t+1] = v[t] + a[t] * dt
	        // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
	        // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

	        fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
	        fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
	        fg[1 + psi_start + t] = psi1 - (psi0 +v0 * delta0 / Lf * dt);
	        fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
	        fg[1 + cte_start + t] = cte1 - ((y0 - f0) + (v0 * CppAD::sin(epsi0) * dt));
	        fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
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
 /* n_vars is no. of independent variables
  * N time steps for each state
  *   (N-1)  values for each actuation
  */
  size_t n_vars = N*6 +(N-1)*2;

  // number of constraints
  size_t n_constraints = N*6;

  // Initialization of values of independent variables
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  //  Initial values of vars
  double x0 = state[0];
  double y0 = state[1];
  double psi0 = state[2];
  double v0 = state[3];
  double cte0 = state[4];
  double epsi0 = state[5];

  // Setting Initial values for vars
  vars[x_start] = x0;
  vars[y_start] =y0;
  vars[psi_start] = psi0;
  vars[v_start] = v0;
  vars[cte_start] = cte0;
  vars[epsi_start] = epsi0;
  //Setting all the non-actuator upper and lower limits
  for (int i = 0; i < delta_start; i++)
  {
      vars_lowerbound[i] = -1.0e19;
      vars_upperbound[i] = 1e19;
    }
	// Lower and upper limits  for steerign angle (-25,25) degrees
  for (int i=delta_start;i<a_start;i++)
  {
	  vars_lowerbound[i] = -25.0*M_PI/180;
	  vars_upperbound[i] = 25.0*M_PI/180;
  }
  // Lower and upper limits of acc (-1,1)
  for (int i = a_start; i < n_vars; i++)
  {
      vars_lowerbound[i] = -1.0;
      vars_upperbound[i] = 1.0;
    }

 //  Defining the lower and upper limits of constraints
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  // Setting the initial condition constraint lower and upper bound
  constraints_lowerbound[x_start] = x0;
    constraints_lowerbound[y_start] = y0;
    constraints_lowerbound[psi_start] = psi0;
    constraints_lowerbound[v_start] = v0;
    constraints_lowerbound[cte_start] = cte0;
    constraints_lowerbound[epsi_start] = epsi0;

    constraints_upperbound[x_start] = x0;
    constraints_upperbound[y_start] = y0;
    constraints_upperbound[psi_start] = psi0;
    constraints_upperbound[v_start] = v0;
    constraints_upperbound[cte_start] = cte0;
    constraints_upperbound[epsi_start] = epsi0;



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
  options += "Numeric max_cpu_time          5.0\n";

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
  std::cout << "delta: "<<solution.x[delta_start] <<std::endl;
  std::cout<<"a_start :"<< solution.x[a_start]<<std::endl;
  vector <double> output;
  output.push_back( solution.x[delta_start]);
  output.push_back( solution.x[a_start]);
  //Saving the predicted x and y values in output vector so that they can be visualized
  for (int j=2 ; j < N+1;j ++)
  {
	  output.push_back(solution.x[x_start+j-1])  ;
  }
  for (int j=2 ; j < (N+1) ;j ++)
  {
	  output.push_back(solution.x[y_start+j-1]);
  }
  return output;
  }

