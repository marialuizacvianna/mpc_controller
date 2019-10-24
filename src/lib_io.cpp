#include "lib/Matplotlib/matplotlibcpp.h"
#include "lib/io/lib_io.h"
#include <stdio.h>
namespace plt = matplotlibcpp;

void draw_tank(Eigen::VectorXd x ,char* color , float r = 0.3){
  Eigen::MatrixXd M(2, 15);
  M.row(0) << 1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0;
  M.row(1) << -2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1;

  M = move_motif(M,x(0),x(1),x(2));

  std::vector<float> x_plot;
  std::vector<float> y_plot;

  for(int i = 0; i < 15; i ++){
    x_plot.push_back(M(0,i));
    y_plot.push_back(M(1,i));
  }

  plt::plot(x_plot, y_plot);

  plt::show();

  // plt.pause(0.01);
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

void create_txt_file(std::ofstream * file){
  std::vector<std::string> month_vec = {"Jan", "Feb", "Mar", "Apr", "May", "Jui", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
  std::chrono::time_point<std::chrono::system_clock> start;
  start = std::chrono::system_clock::now();
  std::time_t start_t = std::chrono::system_clock::to_time_t(start);
  std::string start_s = std::ctime(&start_t);

  std::string month = start_s.substr(4,3);
  std::string day = start_s.substr(8,2);
  std::string year = start_s.substr(20,4);
  std::string hour = start_s.substr(11,2);
  std::string mins = start_s.substr(14,2);
  std::string secs = start_s.substr(17,2);


  std::vector<std::string>::iterator it = std::find (month_vec.begin(), month_vec.end(), month);
  int month_nb = it - month_vec.begin() + 1;

  std::stringstream txt_title_aux;
  txt_title_aux<< io_path << month_nb << day << year<< hour << mins << secs << ".txt";
  std::string txt_title = txt_title_aux.str();
  file->open(txt_title);
}


void write_results(std::ofstream * file, Eigen::VectorXd x, Eigen::VectorXd u){
  *file << x(0) << "," << x(1) << "," << x(2) << "," << x(3) << "," << u(0) << "," << u(1) << "\n";
}
