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
#include <stdio.h>
#include <stdlib.h>
#include <time.h>       /* time */

int main(int argc, char *argv[]) {
  std::string path_to_follow;
  std::cout << "argv[1] == " << argv[1] << std::endl;
  std::cout << "argv[2] == " << argv[2] << std::endl;
  if(atoi(argv[1]) == 0)
    path_to_follow = "line";

  else if(atoi(argv[1]) == 1)
    path_to_follow = "sinus";

  else if(atoi(argv[1]) == 2)
    path_to_follow = "circle";

  else if(atoi(argv[1]) == 3)
    path_to_follow = "quad";

  int neurons =  atoi(argv[2]);

  double x1 =  atof(argv[3]);

  double x2 =  atof(argv[4]);

  double freq = atof(argv[5]);

  std::string file_name;
  file_name = "/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/0505_1600/result_line_new.txt";

  if(RESULTS_NN_FILE == 1){
    x1 = 3.0;
    x2 = 0.5;
    path_to_follow = "line";
  }

  std::cout << "restart x0 =  " << 0.0 << ", x1 =  " << x1 << "  , x2 = " <<x2 << ", freq = "<< freq << std::endl;

  srand (time(NULL));
  glInitMPC(X0,x1,x2,path_to_follow,neurons,freq,file_name);
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
  return 0;

}
