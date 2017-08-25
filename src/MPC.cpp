#include "MPC.h"
#include "Eigen-3.3/Eigen/Core"
#include <limits>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;
using namespace std;

// Number of timesteps
size_t N = 10;

// Duration of each timestep
double dt = 0.1;

// Car's length from the front to center of gravity
const double Lf = 2.67;

// Target speed in m/s
double ref_v = 120.0 * 0.447;

// Variable start indices

// State: N variables
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;

// Errors: N variables
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;

// Actuators: N - 1 variables
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
  
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  
  // Constructor
  FG_eval(Eigen::VectorXd coeffs) {
    
    this->coeffs = coeffs;
    
  }
  
  /**
   Sets the initial cost, constraints, and variables

   @param fg cost and constraints
   @param vars state, error, and actuator variables
   */
  void operator()(ADvector &fg, const ADvector &vars) {
    
    // Computing the cost
    
    // The cost is stored at index 0
    fg[0] = 0.0;
    
    // Cost based on errors
    for (int t = 0; t < N; ++t) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2) * 100;
      fg[0] += CppAD::pow((vars[v_start + t] - ref_v), 2);
    }
    
    // Cost based on actuators: N - 1 variables
    // Minimizes the use of actuators
    for (int t = 0; t < N - 1; ++t) {
      fg[0] += CppAD::pow(vars[delta_start + t], 2) * 100;
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }
    
    // Cost based on sequential actuations: N - 1 variables - 1 timestep for t + 1
    // Mininmizes the actuation rate of change
    for (int t = 0; t < N - 2; ++t) {
      fg[0] += CppAD::pow((vars[delta_start + t + 1] - vars[delta_start + t]), 2) * 1000;
      fg[0] += CppAD::pow((vars[a_start + t + 1] - vars[a_start + t]), 2);
    }
    
    // Setting the initial state and error constraints
    // Index + 1 because the cost is stored at index 0
    
    // State
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    
    // Errors
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    
    for (int t = 1; t < N; ++t) {
      
      // Time t
      
      // State
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      
      // Error
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      
      // Actuation
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];
      
      // Desired y0 = f(x)
      // Used to calculate CTE
      AD<double> ydes0 = coeffs[3] * CppAD::pow(x0, 3) + coeffs[2] * CppAD::pow(x0, 2) + coeffs[1] * x0 + coeffs[0];
      
      // Desired psi = arctan(f'(x))
      // Used to calculate epsi
      AD<double> psides0 = CppAD::atan(3 * coeffs[3] * CppAD::pow(x0, 2) + 2 * coeffs[2] * x0 + coeffs[1]);
      
      // Time t + 1
      
      // State
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      
      // Error
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];
      
      // Setting the remaining state and error constraints at time t + 1
      
      // State
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + (v0 / Lf) * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      
      // Error
      fg[1 + cte_start + t] = cte1 - ((ydes0 - y0) + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + (v0 / Lf) * delta0 * dt);
      
    }
    
  }
  
}; // End FG_eval class


// MPC class definition

// Constructor
MPC::MPC() {}

// Destructor
MPC::~MPC() {}

/**
 Solves the optimization problem

 @param state car's coordinates (x, y), heading (psi),
              speed (v), and errors (cte, and epsi)
 @param coeffs coefficients of the polynomial fitted
               to the waypoints
 @return actuation commands and predicted path
 */
vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  // State variables
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  
  // Error variables
  double cte = state[4];
  double epsi = state[5];
  
  // Number of model variables
  // 6 state + error variables, 2 actuator variables
  size_t n_vars = 6 * N + 2 * (N - 1);
  
  // Number of model constraints
  size_t n_constraints = 6 * N;
  
  // Setting the variable values
  
  // All variables = 0 except for the initial state and error variables
  Dvector vars(n_vars);
  
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0.0;
  }
  
  // Initial state variables
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  
  // Initial error variables
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  
  // Setting the variable lower and upper bounds
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  // State and error variable lower and upper bounds
  for (int i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = - numeric_limits<double>::max();
    vars_upperbound[i] = numeric_limits<double>::max();
  }
  
  // Steering variable lower and upper bounds
  for (int i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -(25.0 / 180.0) * M_PI;
    vars_upperbound[i] = (25.0 / 180.0) * M_PI;
  }
  
  // Throttle variable lower and upper bounds
  for (int i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  
  // Setting the constraint lower and upper bounds
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  
  // All constraints = 0 except for the initial state and error constraints
  for (int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = constraints_upperbound[i] = 0.0;
  }
  
  // Initial state constraints
  constraints_lowerbound[x_start] = constraints_upperbound[x_start] = x;
  constraints_lowerbound[y_start] = constraints_upperbound[y_start] = y;
  constraints_lowerbound[psi_start] = constraints_upperbound[psi_start] = psi;
  constraints_lowerbound[v_start] = constraints_upperbound[v_start] = v;
  
  // Initial error constraints
  constraints_lowerbound[cte_start] = constraints_upperbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = constraints_upperbound[epsi_start] = epsi;
  
  // Initializing the FG evaluator object
  FG_eval fg_eval(coeffs);
  
  // IPOPT solver options
  string options;
  
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
  options += "Numeric max_cpu_time          0.5\n";

  // Vector to capture the solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // Solving the optimization problem
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound,
                                        constraints_lowerbound, constraints_upperbound, fg_eval, solution);

  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  cout << "Cost " << cost << endl;
  
  vector<double> result;
  
  // Steering value: Index 0
  result.push_back(solution.x[delta_start]);
  
  // Throttle value: Index 1
  double throttle;
  throttle = solution.x[a_start] * (M_PI / (fabs(solution.x[delta_start]) * 36.0 + M_PI));
  result.push_back(throttle);
  
  // X coordinates: Even indices
  // Y coordinates: Odd indices
  for (int t = 0; t < N; ++t) {
    result.push_back(solution.x[x_start + t]);
    result.push_back(solution.x[y_start + t]);
  }
  
  return result;
}
