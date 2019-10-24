/*
  Author : Maria Costa
  maria.costa@ensta-bretagne.org
  LIX - Ecole Polytechnique
  Date : 21/10/2019
*/

#include <iostream>
#include "lib/mpc/lib_mpc.h"
#include <vector>
#include <math.h>
#include "lib/io/lib_io.h"
#include <string.h>
#include <sstream>

int main(int argc, char *argv[]) {
  std::ofstream result_file;
  int referece_count = 0; // reference step
  Eigen::VectorXd ref_x(NB_POINTS_POLYFIT); //vector with reference x positions
  Eigen::VectorXd ref_y(NB_POINTS_POLYFIT); //vector with reference y positions
  Eigen::VectorXd x(SIZE_STATE);//x y psi v err_offset err_psi
  Eigen::VectorXd u(SIZE_INPUT);//steering angle rate  acceleration
  std::vector<double> result_mpc;
  int i = 0; //iterator
  float j = 0.0;
  std::chrono::time_point<std::chrono::system_clock> start;

  create_txt_file(&result_file);
  // result_file.open("reults/result_run.txt")

  for(i = 0; i < SIZE_STATE; i++){
    x(i) = 0.0; //vehicle position
  }

  x(1) = -1.0; //speed
  x(3) = 5.0; //speed

  for(i = 0; i < SIZE_INPUT; i++){
    u(i) = 0.0; //vehicle position
  }


  for(j = 0.0; j < T; j=j+DT){
    // draw_car(x);
    // draw_tank(x ,"blue" , 0.3);

    for(i = 0; i < NB_POINTS_POLYFIT; i++){
      ref_y(i) = 0.0;
      ref_x(i) = x(0) + (DT*(i+1));
    }
    referece_count += NB_POINTS_POLYFIT;

    x = Propagate(x,u);
    for(i = 0; i < NB_POINTS_POLYFIT; i++){
      ref_y(i) = (ref_y(i) - x(1))*cos(x(2)) - (ref_x(i) - x(0))*sin(x(2));
      ref_x(i) = (ref_x(i) - x(0))*cos(x(2)) + (ref_y(i) - x(1))*sin(x(2));
    }

    Eigen::VectorXd poly_coeffs = Polyfit(ref_x, ref_y, 3);
    //it  actualizes the function offset error
    x(4) = poly_coeffs(0);

    // it calculate the vehicle's orientation error
    // f(x) is the polynomial defining the reference trajectory
    // f'(x) = 3Ax^2 + 2Bx + C
    // double d_f_x = 3*poly_coeffs[3]*pow(px,2) + 2*poly_coeffs[2]*px + poly_coeffs[1];
    double d_f_x = poly_coeffs(1);
    x(5) = -atan(d_f_x);

    // solve mpc for state and reference trajectory
    // returns [steering_angle, acceleration]
    result_mpc = Solve_MPC(x,poly_coeffs);
    u(0) = result_mpc.at(0);
    u(1) = result_mpc.at(1);
  }

  return 0;
}
