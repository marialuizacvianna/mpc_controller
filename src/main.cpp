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



// a global instance of std::mutex to protect global variable
// std::mutex myMutex;

// void main_draw(){
//   std::lock_guard<std::mutex> guard(myMutex);
//   if(result_x_global.size() == 0)
//     return;
//   std::cout << "Begin visualisation \n";
//   std::cout << "Time = " << j << std::endl;
//   int argc;
//   char *argv[100];
//   glInitMPC();
//   glGetCosts(result_cost_global);
//   glGetCoefficients(ref_x_vector_global,ref_y_vector_global,ref_x_vector_robot_global,ref_y_vector_robot_global);
//   glGetResults(result_x_global,result_y_global,result_psi_global);
//   glutInit (&argc, argv);
//   glutInitDisplayMode (GLUT_DOUBLE);
//   glutInitWindowSize (2000, 1000);
//   glutInitWindowPosition (0, 0);
//   glutCreateWindow ("MPC Simulation");
//   glutDisplayFunc (display);
//   glutIdleFunc (display);
//   glutReshapeFunc (reshape);
//   initgLOptions();
//   std::cout << "End simulation \n";
//
// }
//
// void main_MPC(){
//   std::lock_guard<std::mutex> guard(myMutex);
//
//   Eigen::VectorXd ref_x(CONTROL_HORIZON); //vector with reference x positions
//   Eigen::VectorXd ref_y(CONTROL_HORIZON); //vector with reference y positions
//
//   Eigen::VectorXd ref_x_robot(CONTROL_HORIZON); //vector with reference x positions
//   Eigen::VectorXd ref_y_robot(CONTROL_HORIZON); //vector with reference y positions
//
//   std::vector<double> result_mpc;
//
//   double last_x = X0;
//
//   int i = 0; //iterator
//
//   std::cout << "Begin simulation \n";
//   std::cout << "Time = " << j << std::endl;
//
//   result_x_global.push_back(x(0));
//   result_y_global.push_back(x(1));
//   result_psi_global.push_back(x(2));
//
//   //line
//   for(i = 0; i < CONTROL_HORIZON; i++){
//     if(x(0) >= last_x ){
//       ref_x(i) = x(0) + (DT*(i + 5));
//       ref_y(i) = cos(ref_x(i)) ;
//       last_x = x(0);
//     }
//
//     else{
//       ref_x(i) = last_x + (DT*(i + 1));
//       ref_y(i) = cos(ref_x(i)) ;
//     }
//
//     ref_x_vector_global.push_back(ref_x(i));
//     ref_y_vector_global.push_back(ref_y(i));
//   }
//
//   Eigen::VectorXd poly_coeffs = Polyfit(ref_x, ref_y,2);
//
//   double ref_theta = -atan(poly_coeffs(1));
//
//   x = Propagate(x,u);
//
//   for(i = 0; i < CONTROL_HORIZON; i++){
//     ref_y_robot(i) = (ref_y(i) - x(1))*cos(x(2)) - (ref_x(i) - x(0))*sin(x(2));
//     ref_x_robot(i) = (ref_x(i) - x(0))*cos(x(2)) + (ref_y(i) - x(1))*sin(x(2));
//
//     ref_x_vector_robot_global.push_back(ref_x_robot(i));
//     ref_y_vector_robot_global.push_back(ref_y_robot(i));
//   }
//
//   // solve mpc for state and reference trajectory
//   // returns [steering_angle, acceleration]
//   result_mpc = Solve_MPC(x,ref_x,ref_y,ref_theta);
//   u(0) = result_mpc.at(0);
//   u(1) = result_mpc.at(1);
//
//   result_cost_global = result_mpc.at(2);
//   // getchar();
// }



int main(int argc, char *argv[]) {
  glInitMPC();
  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DOUBLE);
  glutInitWindowSize (2000, 1000);
  glutInitWindowPosition (0, 0);
  glutCreateWindow ("MPC Simulation");
  glutDisplayFunc (display);
  glutIdleFunc (display);
  glutReshapeFunc (reshape);
  initgLOptions();
  glutMainLoop();
  std::cout << "End simulation \n";
	return 0;

}
