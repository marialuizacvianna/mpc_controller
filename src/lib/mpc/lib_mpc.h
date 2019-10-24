#ifndef LIB_MPC
#define LIB_MPC

#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"
#include <math.h>
#include <stdio.h>
#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <fstream>

#define NB_POINTS_POLYFIT   10
#define SIZE_STATE          6
#define SIZE_INPUT          2
#define LF                 1.0
#define DT                 0.01
#define CONTROL_HORIZON    10
#define T                  0.02

#define REF_V                            1.0
#define REF_OFFSET                       0.0
#define REF_PSI_ERR                      0.0


Eigen::VectorXd Propagate(Eigen::VectorXd,Eigen::VectorXd);
Eigen::VectorXd Polyfit(Eigen::VectorXd x, Eigen::VectorXd y,int n);
std::vector<double> Solve_MPC(Eigen::VectorXd state, Eigen::VectorXd coeffs);

#endif
