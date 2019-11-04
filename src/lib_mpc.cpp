#include "lib/mpc/lib_mpc.h"

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + CONTROL_HORIZON;
size_t psi_start = y_start + CONTROL_HORIZON;
size_t v_start = psi_start + CONTROL_HORIZON;
size_t offset_start = v_start + CONTROL_HORIZON;
size_t epsi_start = offset_start + CONTROL_HORIZON;
size_t delta_start = epsi_start + CONTROL_HORIZON;
size_t a_start = delta_start + CONTROL_HORIZON - 1;

//Following the example from:
//https://www.coin-or.org/CppAD/Doc/ipopt_solve_get_started.cpp.htm
class FG_eval
{
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars)
  {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // ----------------------- Cost
    fg[0] = 0;

    // Reference State Cost
    // minimize
    // 1. cte,
    // 2. epsi (orientation diff between vehicle and ref trajectory)
    // 3. diff bw v and ref_v
    for (int t = 0; t < CONTROL_HORIZON; t++)
    {
      fg[0] += CppAD::pow(vars[offset_start + t] - REF_OFFSET, 2); //CppAD::exp(-t*0.1)*
      // fg[0] += CppAD::pow(vars[epsi_start + t] - REF_PSI_ERR, 2);
      // fg[0] += CppAD::pow(vars[v_start + t] - REF_V, 2);
    }

    // Minimize the use of actuators
    // for (int t = 0; t < CONTROL_HORIZON - 1; t++)
    // {
    //   fg[0] += 10*CppAD::pow(vars[delta_start + t], 2);
    //   fg[0] += 10*CppAD::pow(vars[a_start + t], 2);
    // }

    // Minimize the value gap between sequential actuations.
    // for (int t = 0; t < CONTROL_HORIZON - 2; t++)
    // {
    //   fg[0] += 130000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
    //   fg[0] += 100*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    // }

    // -----------------------  model constraints

    // initial constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + offset_start] = vars[offset_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // rest of the constraints
    for (int t = 1; t < CONTROL_HORIZON; t++)
    {
      // The state at time t+1 .
      CppAD::AD<double> x1 = vars[x_start + t];
      CppAD::AD<double> y1 = vars[y_start + t];
      CppAD::AD<double> psi1 = vars[psi_start + t];
      CppAD::AD<double> v1 = vars[v_start + t];
      CppAD::AD<double> cte1 = vars[offset_start + t];
      CppAD::AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      CppAD::AD<double> x0 = vars[x_start + t - 1];
      CppAD::AD<double> y0 = vars[y_start + t - 1];
      CppAD::AD<double> psi0 = vars[psi_start + t - 1];
      CppAD::AD<double> v0 = vars[v_start + t - 1];
      CppAD::AD<double> cte0 = vars[offset_start + t - 1];
      CppAD::AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      CppAD::AD<double> delta0 = vars[delta_start + t - 1];
      CppAD::AD<double> a0 = vars[a_start + t - 1];

      CppAD::AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
      CppAD::AD<double> f0prime = coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*x0*x0;

      CppAD::AD<double> psides0 = CppAD::atan(f0prime);

      // equations for the model:
      // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      // v_[t] = v[t-1] + a[t-1] * dt
      // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * DT);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * DT);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / LF * DT);
      fg[1 + v_start + t] = v1 - (v0 + a0 * DT);
      fg[1 + offset_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * DT));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) - v0 * delta0 / LF * DT);
    }
  }
};


Eigen::VectorXd Propagate(Eigen::VectorXd x ,Eigen::VectorXd u) {
  Eigen::VectorXd x_propagate(SIZE_STATE);
  x_propagate(0) = x(0) + x(3)*cos(x(2))*DT;
  x_propagate(1) = x(1) + x(3)*sin(x(2))*DT;
  x_propagate(2) = x(2) - x(3)/LF*u(0)*DT;
  x_propagate(3) = x(3) + u(1)*DT;
  x_propagate(4) = x(4);
  x_propagate(5) = x(5);
  return x_propagate;
};


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd Polyfit(Eigen::VectorXd x, Eigen::VectorXd y,int n) {
  assert(x.size() == y.size());
  assert(n >= 1 && n <= x.size() - 1);
  Eigen::MatrixXd A(x.size(), n + 1);

  for (int i = 0; i < x.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < x.size(); j++) {
    for (int i = 0; i < n; i++) {
      A(j, i + 1) = A(j, i) * x(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(y);
  return result;
}

std::vector<double> Solve_MPC(Eigen::VectorXd x, Eigen::VectorXd poly_coeffs){
  typedef CPPAD_TESTVECTOR(double) Dvector;
  bool ok = true;
  std::vector<double> x_pred_vals;
  std::vector<double> y_pred_vals;

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = SIZE_STATE*CONTROL_HORIZON + SIZE_INPUT*(CONTROL_HORIZON-1);
  // Set the number of constraints
  size_t n_constraints = SIZE_STATE*CONTROL_HORIZON;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++)
    vars[i] = 0;

  // set initial state
  vars[x_start] = x[0];
  vars[y_start] = x[1];
  vars[psi_start] = x[2];
  vars[v_start] = x[3];
  vars[offset_start] = x[4];
  vars[epsi_start] = x[5];

  // Set lower and upper limits for variables.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int i = delta_start; i < a_start; i++)
  {
    vars_lowerbound[i] = -0.836332;
    vars_upperbound[i] = 0.836332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++)
  {
    vars_lowerbound[i] = -0.1;
    vars_upperbound[i] = 0.1;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x[0];
  constraints_lowerbound[y_start] = x[1];
  constraints_lowerbound[psi_start] = x[2];
  constraints_lowerbound[v_start] = x[3];
  constraints_lowerbound[offset_start] = x[4];
  constraints_lowerbound[epsi_start] = x[5];

  constraints_upperbound[x_start] = x[0];
  constraints_upperbound[y_start] = x[1];
  constraints_upperbound[psi_start] = x[2];
  constraints_upperbound[v_start] = x[3];
  constraints_upperbound[offset_start] = x[4];
  constraints_upperbound[epsi_start] = x[5];

  //Following the example from:
  //https://www.coin-or.org/CppAD/Doc/ipopt_solve_get_started.cpp.htm
  // object that computes objective and constraints
  FG_eval fg_eval(poly_coeffs);
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
  std::cout << "cost = " << cost << std::endl;
  // set x_pred_vals and y_pred_vals for plotting
  x_pred_vals.clear();
  y_pred_vals.clear();
  for (int i = 1; i<CONTROL_HORIZON; ++i)
  {
    x_pred_vals.push_back(solution.x[x_start+i]);
    y_pred_vals.push_back(solution.x[y_start+i]);
  }

  //  Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {solution.x[delta_start], solution.x[a_start], cost};
}
