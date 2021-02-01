#ifndef TRAJ_H
#define TRAJ_H

#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>

class Race{
  public:
    Race(std::string file_name,std::string out_file,std::string in_file, int c);
    int control_horizon;
    std::string traj_file_name;
    std::string out_file_name;
    std::string in_file_name;
    std::vector<double> traj_x;
    std::vector<double> traj_y;
    std::vector<double> out_contour_x;
    std::vector<double> out_contour_y;
    std::vector<double> in_contour_x;
    std::vector<double> in_contour_y;
    int find_closest_point_map(Eigen::VectorXd  x);
    void get_reference(Eigen::VectorXd x,std::vector<double>& ref_x_vector ,std::vector<double>& ref_y_vector,Eigen::VectorXd& ref_x_robot,Eigen::VectorXd& ref_y_robot);
};

class Circle{
  public:
    Circle(double x_center,double y_center, double ray,int c,double time_step);
    int control_horizon;
    double angle2follow;
    double cx;
    double cy;
    double circle_ray;
    double dt;
    void get_reference(Eigen::VectorXd x,std::vector<double>& ref_x_vector ,std::vector<double>& ref_y_vector,Eigen::VectorXd& ref_x_robot,Eigen::VectorXd& ref_y_robot);
    void reset();
};

class Sinus{
  public:
    Sinus(int c,double time_step,double freq);
    int control_horizon;
    double dt;
    void get_reference(Eigen::VectorXd x,std::vector<double>& ref_x_vector ,std::vector<double>& ref_y_vector,Eigen::VectorXd& ref_x_robot,Eigen::VectorXd& ref_y_robot);
    double f;
    double last_x;
    double last_y;
    void change_f(double new_value);
    void reset();
};


class Line{
  public:
    Line(int c,double time_step,double line_offset);
    int control_horizon;
    double dt;
    void get_reference(Eigen::VectorXd x,std::vector<double>& ref_x_vector ,std::vector<double>& ref_y_vector,Eigen::VectorXd& ref_x_robot,Eigen::VectorXd& ref_y_robot);
    double last_x;
    double offset;
    void reset();
};


#endif
