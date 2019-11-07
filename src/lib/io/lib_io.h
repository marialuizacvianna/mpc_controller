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

const std::string io_path = "../results/";

void glGetResults(std::vector<double> x, std::vector<double> y , std::vector<double> psi);
void glGetCosts(std::vector<double> costs);
void glGetCoefficients(std::vector<double> ref_x_vector,std::vector<double>  ref_y_vector,
  std::vector<double> ref_x_vector_robot,std::vector<double>  ref_y_vector_robot);
//OpenGL functions
void printString(const char *str, double x, double y, double size);
void timer(int id);
void drawTank();
void drawReference();
void drawRobotReference();
void display();
void reshape (int w, int h);
void initgLOptions();


//Translate and Rotate tank points matrix
Eigen::MatrixXd move_motif(Eigen::MatrixXd M, float x, float y,float psi);

//Text file output
void create_txt_file(std::ofstream * file);
void write_results(std::ofstream * file, Eigen::VectorXd x, Eigen::VectorXd u);


#endif
