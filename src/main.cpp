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
  std::vector<double> result_off;
  std::vector<double> result_cost_0;
  std::vector<double> result_cost_1;
  std::vector<double> ref_x_vector, ref_y_vector;
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

  // create_txt_file(&result_file);
  // if(result_file.is_open())
  //   std::cout << "Created file for writing results \n";

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
    result_y.push_back(x(1));
    result_psi.push_back(x(2));

    // write_results(&result_file,x,u);

    //line
    for(i = 0; i < CONTROL_HORIZON; i++){
      if(x(0) >= last_x ){
        ref_x(i) = x(0) + (DT*(i + 5));
        ref_y(i) = LINE + sin(ref_x(i));
        last_x = x(0);
      }
      else{
        ref_x(i) = last_x + (DT*(i + 1));
        ref_y(i) = LINE + sin(ref_x(i));
      }

      ref_x_vector.push_back(ref_x(i));
      ref_y_vector.push_back(ref_y(i));
    }


    x = Propagate(x,u);

    // solve mpc for state and reference trajectory
    // returns [steering_angle, acceleration]
    result_mpc = Solve_MPC(x,ref_x_vector,ref_y_vector);
    u(0) = result_mpc.at(0);
    u(1) = result_mpc.at(1);
    result_cost.push_back(result_mpc.at(2));

    // getchar();

  }
  std::cout << "Begin visualisation \n";
  glGetCosts(result_cost);
  // glGetCoefficients(ref_x_vector,ref_y_vector);
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
  // result_file.close();

  return 0;
}
