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
#include <thread>         // std::thread
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <algorithm>
#include <mutex>

// global variables
std::vector<double>result_x_global,result_y_global,result_psi_global;
double result_cost_global;
std::vector<double> ref_x_vector_global, ref_y_vector_global;
std::vector<double> ref_x_vector_robot_global, ref_y_vector_robot_global;


// a global instance of std::mutex to protect global variable
std::mutex myMutex;

void main_draw(){
  std::cout << "Begin visualisation \n";
  int argc;
  char *argv[100];
  std::lock_guard<std::mutex> guard(myMutex);
  glGetCosts(result_cost_global);
  glGetCoefficients(ref_x_vector_global,ref_y_vector_global,ref_x_vector_robot_global,ref_y_vector_robot_global);
  glGetResults(result_x_global,result_y_global,result_psi_global);
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

}
//
void main_MPC(){
  std::lock_guard<std::mutex> guard(myMutex);

  std::vector<double> result_x;
  std::vector<double> result_y;
  std::vector<double> result_psi;
  std::vector<double> result_cost;
  std::vector<double> ref_x_vector, ref_y_vector;
  std::vector<double> ref_x_vector_robot, ref_y_vector_robot;
  // std::ofstream result_file;

  Eigen::VectorXd ref_x(CONTROL_HORIZON); //vector with reference x positions
  Eigen::VectorXd ref_y(CONTROL_HORIZON); //vector with reference y positions

  Eigen::VectorXd ref_x_robot(CONTROL_HORIZON); //vector with reference x positions
  Eigen::VectorXd ref_y_robot(CONTROL_HORIZON); //vector with reference y positions

  Eigen::VectorXd x(SIZE_STATE);//x y psi v err_offset err_psi
  Eigen::VectorXd u(SIZE_INPUT);//steering angle rate  acceleration

  std::vector<double> result_mpc;

  double last_x = X0;

  int i = 0; //iterator
  float j = 0.0;//iterator

  x(0) = X0; //x
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
    result_x_global.push_back(x(0));
    result_y.push_back(x(1));
    result_y_global.push_back(x(1));
    result_psi.push_back(x(2));
    result_psi_global.push_back(x(2));

    //line
    for(i = 0; i < CONTROL_HORIZON; i++){
      if(x(0) >= last_x ){
        ref_x(i) = x(0) + (DT*(i + 5));
        ref_y(i) = cos(ref_x(i)) ;
        last_x = x(0);
      }
      else{
        ref_x(i) = last_x + (DT*(i + 1));
        ref_y(i) = cos(ref_x(i)) ;
      }

      ref_x_vector.push_back(ref_x(i));
      ref_x_vector_global.push_back(ref_x(i));
      ref_y_vector.push_back(ref_y(i));
      ref_y_vector_global.push_back(ref_y(i));
    }

    Eigen::VectorXd poly_coeffs = Polyfit(ref_x, ref_y,2);

    double ref_theta = -atan(poly_coeffs(1));

    x = Propagate(x,u);

    for(i = 0; i < CONTROL_HORIZON; i++){
      ref_y_robot(i) = (ref_y(i) - x(1))*cos(x(2)) - (ref_x(i) - x(0))*sin(x(2));
      ref_x_robot(i) = (ref_x(i) - x(0))*cos(x(2)) + (ref_y(i) - x(1))*sin(x(2));

      ref_x_vector_robot.push_back(ref_x_robot(i));
      ref_x_vector_robot_global.push_back(ref_x_robot(i));
      ref_y_vector_robot.push_back(ref_y_robot(i));
      ref_y_vector_robot_global.push_back(ref_y_robot(i));
    }

    // solve mpc for state and reference trajectory
    // returns [steering_angle, acceleration]
    result_mpc = Solve_MPC(x,ref_x,ref_y,ref_theta);
    u(0) = result_mpc.at(0);
    u(1) = result_mpc.at(1);
    // result_cost.push_back(result_mpc.at(2));
    // result_list.push_back(result_mpc.at(2));
    result_cost_global = result_mpc.at(2);
    // getchar();

  }
}




int main(int argc, char *argv[]) {
	std::thread t1(main_MPC);
	std::thread t2(main_draw);


	t1.join();
	t2.join();


	return 0;

}
