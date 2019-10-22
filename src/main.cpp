/*
  Author : Maria Costa
  maria.costa@ensta-bretagne.org
  LIX - Ecole Polytechnique
  Date : 21/10/2019
*/

#include <iostream>
#include "lib_mpc.h"
#include <vector>
#include <math.h>

int main(int argc, char *argv[]) {
  int referece_count = 0; // reference step
  Eigen::VectorXd ref_x(NB_POINTS_POLYFIT); //vector with reference x positions
  Eigen::VectorXd ref_y(NB_POINTS_POLYFIT); //vector with reference y positions
  Eigen::VectorXd x(SIZE_STATE);//x y psi v err_offset err_psi
  Eigen::VectorXd u(SIZE_INPUT);//steering angle rate  acceleration

  int i,j = 0; //iterator

  for(i = 0; i < SIZE_STATE; i++){
    x(i) = 0.0; //vehicle position
  }

  for(i = 0; i < SIZE_INPUT; i++){
    u(i) = 0.0; //vehicle position
  }

  while(1){

    for(i = 0; i < NB_POINTS_POLYFIT; i++){
      ref_y(i) = 1.0;
      ref_x(i) = i + referece_count;
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
    double d_f_x = poly_coeffs[1];
    x(5) = -atan(d_f_x);

    // solve mpc for state and reference trajectory
    // returns [steering_angle, acceleration]
    Solve_MPC(x,poly_coeffs);


  }




  return 0;
}
