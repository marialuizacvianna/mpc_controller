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
#define SIZE_STATE          4
#define SIZE_INPUT          2
#define LF                 1.0
#define DT                 0.01
#define CONTROL_HORIZON    20
#define T                  30
#define LINE               0.0

//intial conditions
#define X0                 0.0 //x
#define X1                 1.0 //y
#define X2                 -3.14//yaw
#define X3                 3.0 // speed

#define REF_V                            1.0
#define REF_OFFSET                       0.0
#define REF_PSI_ERR                      0.0


Eigen::VectorXd Propagate(Eigen::VectorXd,Eigen::VectorXd);
Eigen::VectorXd Polyfit(Eigen::VectorXd x, Eigen::VectorXd y,int n);
std::vector<double> Solve_MPC(Eigen::VectorXd state,std::vector<double> ref_x_world,std::vector<double> ref_y_world);


#endif
