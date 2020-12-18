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
#include "../scan/pose_2d.hpp"


const std::string io_path = "../results/";
std::vector<double> get_closest_point(Eigen::MatrixXd wall,std::vector<double> predicted_car_position);
void drawWall(Eigen::MatrixXd wall,int size,Eigen::VectorXd x);
std::vector<double>  update_scan(Eigen::VectorXd x) ;
Eigen::MatrixXf get_scan_as_cartesian(std::vector<double> msg);
int find_left_right_border(Eigen::MatrixXf points);
void drawCircleInput(float cx,float cy,float r,float num_segments, int count);
void glGetResults(std::vector<double> x, std::vector<double> y , std::vector<double> psi);
void glGetCosts(double cost);
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
void glInitMPC(double x0,double y0,double theta0,std::string );
void CalculateMPC();
void CalculateTrajectory();
// std::vector<double>  find_closest_point_map(std::vector<double>  x);
int find_closest_point_map(Eigen::VectorXd  x);
void init_map();
//Translate and Rotate tank points matrix
Eigen::MatrixXd move_motif(Eigen::MatrixXd M, float x, float y,float psi);

//Text file output
void create_txt_file(std::ofstream * file);
void write_results(std::ofstream * file, Eigen::VectorXd x, Eigen::VectorXd u);


#endif
