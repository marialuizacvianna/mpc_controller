/*
  Author : Maria Costa
  maria.costa@ensta-bretagne.org
  LIX - Ecole Polytechnique
*/

#include <iostream>
#include <vector>
#include <math.h>
#include <string.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "lib/traj/lib_trajectory.h"
#include "lib/mpc/lib_mpc.h"
#include "lib/io/lib_io.h"

int main(int argc, char *argv[]) {
  double x0 = 0.0;
  double y0 = 0.0;
  double theta0 = 0.0;
  int parse_help = 0;
  int reverse_traj = 0;

  if(argc > 1){
    if(strcmp(argv[1],"-h") == 0){
      std::cout <<"help:\n";
      return 0;
    }
    else{
      for(int i=1 ; i < argc; i = i + 1 + parse_help){
          if(strcmp(argv[i],"-x0") == 0){
            x0 = atof(argv[i+1]);
            parse_help = 1;
          }
          else if(strcmp(argv[i],"-y0")){
            y0 = atof(argv[i+1]);
            parse_help = 1;
          }
          else if(strcmp(argv[i],"-t0")){
            theta0 = atof(argv[i+1]);
            parse_help = 1;
          }
          else if(strcmp(argv[i],"-r")){
            reverse_traj = atoi(argv[i+1]);
            parse_help = 1;
          }
          else{
            parse_help = 0;
          }
      }
    }
  }


  srand (time(NULL));
  glInitMPC(x0,y0,theta0);
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
