#ifndef LIB_IO
#define LIB_IO

#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"
#include <math.h>
#include <vector>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <chrono>
#include <ctime>


void CalculateMPC();
void CalculateTrajectory();
//OpenGL functions
void printString(const char *str, double x, double y, double size);
void timer(int id);
void drawTank();
void drawReference();
void drawRobotReference();
void display();
void reshape (int w, int h);
void initgLOptions();
void glInitMPC(double x0,double y0,double theta0,std::string ,double);
//Translate and Rotate tank points matrix
Eigen::MatrixXd move_motif(Eigen::MatrixXd M, float x, float y,float psi);

#endif
