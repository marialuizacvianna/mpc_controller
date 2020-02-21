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
  srand (time(NULL));
  glInitMPC(X0,X1,X2);
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
