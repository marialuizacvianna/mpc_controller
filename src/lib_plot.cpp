#include "lib/Matplotlib/matplotlibcpp.h"
#include  "lib/plot/lib_plot.h"
#include <stdio.h>
namespace plt = matplotlibcpp;

void draw_tank(Eigen::VectorXd x ,char* color , float r = 0.3){
  Eigen::MatrixXd M(2, 15);

  M.row(0) << 0.3, -0.3,   0.0,    0.0,   -0.3 , -0.3 ,  0.0,    0.0,   -0.3,   0.3 ,0.0,    0.0,   0.9  , 0.9 ,  0.0;

  M.row(1) << -0.6 , -0.6 , -0.6 , -0.3 , -0.3  , 0.3 ,  0.3  , 0.6  , 0.6  , 0.6 ,  0.6 ,  0.3, 0.15 ,-0.15 ,-0.3;

  M = move_motif(M,x(0),x(1),x(2));

  std::vector<float> x_plot;
  std::vector<float> y_plot;

  for(int i = 0; i < 15; i ++){
    x_plot.push_back(M(0,i));
    y_plot.push_back(M(1,i));
  }

  plt::plot(x_plot, y_plot);

  plt::show();
}

Eigen::MatrixXd move_motif(Eigen::MatrixXd M, float x, float y,float psi){
  Eigen::MatrixXd M1 = Eigen::MatrixXd::Constant(1, M.cols(), 1.0);
  Eigen::MatrixXd M2(M.rows()+M1.rows(), M.cols());
  M2 << M, M1;
  Eigen::MatrixXd R(2,3);
  R.row(0) << cos(psi), -sin(psi), x;
  R.row(1) << sin(psi), cos(psi),  y ;
  return (R*M2);
}
