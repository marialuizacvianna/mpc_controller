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

#define NB_POINTS_POLYFIT   100
#define SIZE_STATE          5
#define SIZE_INPUT          1
#define LF                 1.0
#define DT                 0.01
#define CONTROL_HORIZON    20
#define T                  20
#define LINE               0.0
#define PATH               "line"
#define RESULTS_FILE        0
#define RESULTS_NN_FILE     0
#define REF_FILE            0

//intial conditions
#define X0                 0.0 //x
#define X1                 0.0 //y
#define X2                 0.0//yaw
#define X3                 3.0 // speed

#define REF_V                            10.0
#define REF_OFFSET                       0.0
#define REF_PSI_ERR                      0.0


Eigen::VectorXd Propagate(Eigen::VectorXd,Eigen::VectorXd);
Eigen::VectorXd Polyfit(Eigen::VectorXd x, Eigen::VectorXd y,int n);
std::vector<double> Solve_MPC(Eigen::VectorXd state,Eigen::VectorXd ref_x_world,Eigen::VectorXd ref_y_world,double ref_theta,double ref_theta_r);
Eigen::VectorXd Polyfit(Eigen::VectorXd x, Eigen::VectorXd y,int n);
std::vector<std::string> split(const std::string &s, char delim);

#endif
