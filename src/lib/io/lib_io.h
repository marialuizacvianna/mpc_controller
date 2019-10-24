#ifndef LIB_IO
#define LIB_IO

#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"
#include <math.h>
#include <vector>
#include <chrono>
#include <ctime>
#include <fstream>

const std::string io_path = "../results/";

void draw_tank(Eigen::VectorXd x,char* color , float r);
void draw_car(Eigen::VectorXd x);
Eigen::MatrixXd move_motif(Eigen::MatrixXd M, float x, float y,float psi);

void create_txt_file(std::ofstream * file);
void write_results(std::ofstream * file, Eigen::VectorXd x, Eigen::VectorXd u);


#endif
