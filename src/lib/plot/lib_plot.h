#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"
#include <math.h>
#include <vector>

void draw_tank(Eigen::VectorXd x,char* color , float r);
Eigen::MatrixXd move_motif(Eigen::MatrixXd M, float x, float y,float psi);
