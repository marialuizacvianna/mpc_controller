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
  std::vector<double> result_x;
  std::vector<double> result_y;
  std::vector<double> result_psi;
  std::vector<double> result_cost;
  std::vector<double> ref_x_vector, ref_y_vector;
  std::ofstream result_file;

  Eigen::VectorXd ref_x(NB_POINTS_POLYFIT); //vector with reference x positions
  Eigen::VectorXd ref_y(NB_POINTS_POLYFIT); //vector with reference y positions

  Eigen::VectorXd x(SIZE_STATE);//x y psi v err_offset err_psi
  Eigen::VectorXd u(SIZE_INPUT);//steering angle rate  acceleration

  std::vector<double> result_mpc;

  double last_x = 0.0;

  int i = 0; //iterator
  float j = 0.0;//iterator

  std::chrono::time_point<std::chrono::system_clock> start;

  create_txt_file(&result_file);
  if(result_file.is_open())
    std::cout << "Created file for writing results \n";

  for(i = 0; i < SIZE_STATE; i++){
    x(i) = 0.0; //vehicle position
  }

  x(1) = X1; //y
  x(2) = X2; //theta
  x(3) = X3; //speed


  for(i = 0; i < SIZE_INPUT; i++){
    u(i) = 0.0;
  }

  std::cout << "Begin simulation \n";
  for(j = 0.0; j < T; j=j+DT){
    std::cout << "Time = " << j << std::endl;
    result_x.push_back(x(0));
    result_y.push_back(x(1));
    result_psi.push_back(x(2));
    write_results(&result_file,x,u);

    //line
    for(i = 0; i < NB_POINTS_POLYFIT; i++){
      ref_y(i) = LINE;
      ref_x(i) = last_x + (DT*(i + 1));
    }

    last_x = ref_x(NB_POINTS_POLYFIT - 1);

    x = Propagate(x,u);

    for(i = 0; i < NB_POINTS_POLYFIT; i++){
      ref_y(i) = (ref_y(i) - x(1))*cos(x(2)) - (ref_x(i) - x(0))*sin(x(2));
      ref_x(i) = (ref_x(i) - x(0))*cos(x(2)) + (ref_y(i) - x(1))*sin(x(2));
      ref_x_vector.push_back(ref_x(i));
      ref_y_vector.push_back(ref_y(i));
    }

    Eigen::VectorXd poly_coeffs = Polyfit(ref_x, ref_y, 3);

    //it  actualizes the function offset error
    x(4) = poly_coeffs(0);
    std::cout << "x(4) = " << x(4) << std::endl;

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
    result_cost.push_back(result_mpc.at(2));

  }
  std::cout << "Begin visualisation \n";
  glGetCosts(result_cost);
  glGetCoefficients(ref_x_vector,ref_y_vector);
  glGetResults(result_x,result_y,result_psi);
  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DOUBLE);
  glutInitWindowSize (2000, 1000);
  glutInitWindowPosition (0, 0);
  glutCreateWindow ("MPC Simulation");

  glutDisplayFunc (display);
  glutIdleFunc (display);
  glutReshapeFunc (reshape);

  initgLOptions();
  glutMainLoop ();

  std::cout << "End simulation \n";
  result_file.close();

  return 0;
}
