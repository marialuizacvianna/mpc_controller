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

#define SIZE_STATE          5
#define SIZE_INPUT          1
#define LF                 1.0
#define DT                 0.01
#define CONTROL_HORIZON    20
#define T                  44
#define LINE               0.0
#define RESULTS_FILE        0
#define RESULTS_MPC_FILE    0
#define RESULTS_NN_FILE     1
#define INPUTS_NN_FILE      0
#define REVERSE             0
#define V_CTE               3.0


Eigen::VectorXd Propagate(Eigen::VectorXd,double);
Eigen::VectorXd Polyfit(Eigen::VectorXd x, Eigen::VectorXd y,int n);
std::vector<double> Solve_MPC(Eigen::VectorXd state,Eigen::VectorXd ref_x_world,Eigen::VectorXd ref_y_world);
Eigen::VectorXd Polyfit(Eigen::VectorXd x, Eigen::VectorXd y,int n);
std::vector<std::string> split(const std::string &s, char delim);

#endif
