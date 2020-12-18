#include "lib/traj/lib_trajectory.h"
#include <stdio.h>
#include <iostream>

Race::Race(std::string file_name,std::string out_file,std::string in_file,int c){
  //open race file points
  std::ifstream traj_file;
  std::ifstream out_contour_file;
  std::ifstream in_contour_file;
  traj_file.open(file_name);//"../berlin_pts.txt"
  out_contour_file.open(out_file);
  in_contour_file.open(in_file);

  std::string line;
  std::vector<double> elems;
  std::stringstream ss(line);
  std::string number;
  size_t pos = 0;
  std::string delimiter = ",";
  std::string x,y;
  while (std::getline(traj_file, line)){
    pos = line.find(delimiter);
    x = line.substr(0, line.find(delimiter));
    line.erase(0, pos + delimiter.length());
    y = line.substr(0, line.find(delimiter));
    this->traj_x.push_back(std::stod(x));
    this->traj_y.push_back(std::stod(y));
  }

  traj_file.close();

  while (std::getline(out_contour_file, line)){
    pos = line.find(delimiter);
    x = line.substr(0, line.find(delimiter));
    line.erase(0, pos + delimiter.length());
    y = line.substr(0, line.find(delimiter));
    this->out_contour_x.push_back(std::stod(x));
    this->out_contour_y.push_back(std::stod(y));
  }

  std::reverse( this->out_contour_x.begin(), this->out_contour_x.end() );
  std::reverse( this->out_contour_y.begin(), this->out_contour_y.end() );

  out_contour_file.close();

  while (std::getline(in_contour_file, line)){
    pos = line.find(delimiter);
    x = line.substr(0, line.find(delimiter));
    line.erase(0, pos + delimiter.length());
    y = line.substr(0, line.find(delimiter));
    this->in_contour_x.push_back(std::stod(x));
    this->in_contour_y.push_back(std::stod(y));
  }

  in_contour_file.close();

  this->control_horizon = c;
}


int Race::find_closest_point_map(Eigen::VectorXd  x){
    int index;//in first and then out
    double x_r = x[0];
    double y_r = x[1];

    int result;

    double distance;
    double x_i;
    double y_i ;

    double smallest =10000;

    int index_smallest;

    for(int i=0; i < traj_x.size();i++){
      x_i = traj_x.at(i) - x_r;
      y_i = traj_y.at(i) - y_r;
      distance = sqrt((x_i*x_i) + (y_i*y_i));

      if(distance < smallest){
        smallest = distance;
        index_smallest = i;
      }
    }

    index = index_smallest;


    return index;

}

void Race::get_reference(Eigen::VectorXd x,std::vector<double>& ref_x_vector ,std::vector<double>& ref_y_vector,Eigen::VectorXd& ref_x_robot,Eigen::VectorXd& ref_y_robot){
  int plus_idx = 0;
  int nex_idx = 0;

  int point_idx = this->find_closest_point_map(x);

  for(int i = 0; i < this->control_horizon; i++){
    if(point_idx + plus_idx+ i < this->traj_x.size())
        nex_idx = point_idx + i + plus_idx ;
    else
        nex_idx =  (point_idx + plus_idx+ i) - this->traj_x.size();

    ref_x_vector.push_back(this->traj_x.at(nex_idx));
    ref_y_vector.push_back(this->traj_y.at(nex_idx));

    ref_y_robot(i) = (ref_y_vector[ref_y_vector.size() - 1] - x(1))*cos(x(2)) - (ref_x_vector[ref_x_vector.size() - 1] - x(0))*sin(x(2));
    ref_x_robot(i) = (ref_x_vector[(ref_x_vector.size() -1)] - x(0))*cos(x(2)) + (ref_y_vector[(ref_y_vector.size() - 1)] - x(1))*sin(x(2));

      // if (!REVERSE){
      //   if(point_idx + plus_idx+ i < traj_x.size())
      //     nex_idx = point_idx + i + plus_idx ;
      //   else
      //     nex_idx =  (point_idx + plus_idx+ i) - traj_x.size();
      // }
      // else{
        // if(point_idx - plus_idx - i >= 0)
        //   nex_idx = point_idx - plus_idx - i;
        // else
        //   nex_idx =  (point_idx - plus_idx - i) + traj_x.size();
      // }


  }
}

Circle::Circle(double x_center,double y_center, double ray,int c,double time_step){
  this->angle2follow = 0.0;
  this->cx = x_center;
  this->cy = y_center;
  this->circle_ray = this->angle2follow/3.14;
  this->control_horizon = c;
  this->dt = time_step;
}

void Circle::get_reference(Eigen::VectorXd x,std::vector<double>& ref_x_vector ,std::vector<double>& ref_y_vector,Eigen::VectorXd& ref_x_robot,Eigen::VectorXd& ref_y_robot){
  double angle2follow_i;
  double circle_ray_i;
  double ref_x,ref_y;
  this->circle_ray = this->angle2follow/3.14;

  for(int i = 0; i < this->control_horizon; i++){
      if(x(0) > (this->circle_ray*cos(this->angle2follow) + cx - 1.5) && x(0) < (this->circle_ray*cos(this->angle2follow) + cx + 1.5)
        && x(1) > (this->circle_ray*sin(this->angle2follow) + cy - 1.5) && x(1) < (circle_ray*sin(this->angle2follow) + cy + 1.5)){
        double angle2follow_it =this->angle2follow + 1.0*i*this->dt;
        this->circle_ray = angle2follow_it/3.14;
        ref_x = this->circle_ray*cos(angle2follow_it) + cx;
        ref_y = this->circle_ray*sin(angle2follow_it) + cy;
        // if(!REVERSE)
          this->angle2follow = this->angle2follow + 0.5*this->dt;
        // else
          // angle2follow = angle2follow - 0.5*this->dt;
      }
      else{
        double angle2follow_it = this->angle2follow + i*this->dt;
        ref_x = this->circle_ray*cos(angle2follow_it) + cx;
        ref_y = this->circle_ray*sin(angle2follow_it) + cy;
      }

      ref_x_vector.push_back(ref_x);
      ref_y_vector.push_back(ref_y);

      ref_y_robot(i) = (ref_y_vector[ref_y_vector.size() - 1] - x(1))*cos(x(2)) - (ref_x_vector[ref_x_vector.size() - 1] - x(0))*sin(x(2));
      ref_x_robot(i) = (ref_x_vector[(ref_x_vector.size() -1)] - x(0))*cos(x(2)) + (ref_y_vector[(ref_y_vector.size() - 1)] - x(1))*sin(x(2));
    }


}

void Circle::reset(){
  this->angle2follow = 0.0;
  this->circle_ray = this->angle2follow/3.14;
}


Sinus::Sinus(int c,double time_step,double freq){
  this->control_horizon = c;
  this->dt = time_step;
  this->f = freq;
  this->last_x = 0.0;
}

void Sinus::get_reference(Eigen::VectorXd x,std::vector<double>& ref_x_vector ,std::vector<double>& ref_y_vector,Eigen::VectorXd& ref_x_robot,Eigen::VectorXd& ref_y_robot){
  double const_aux = 0.0;
  double ref_v_x;
  double ref_x, ref_y;

  if( x(0) + const_aux > this->last_x){
     this->last_x = x(0);
  }
  ref_v_x = (x(3))/sqrt(1+(sin(this->f*(this->last_x +const_aux))*sin(this->f*(this->last_x +const_aux))));
  for(int i = 0; i < this->control_horizon; i++){
    ref_x = this->last_x +const_aux + ref_v_x*this->dt*i;
    ref_y = std::cos(this->f*ref_x);
    ref_v_x = (x(3))/sqrt(1+(sin(this->f*ref_x)*sin(this->f*ref_x)));

    ref_x_vector.push_back(ref_x);
    ref_y_vector.push_back(ref_y);

    ref_y_robot(i) = (ref_y_vector[ref_y_vector.size() - 1] - x(1))*cos(x(2)) - (ref_x_vector[ref_x_vector.size() - 1] - x(0))*sin(x(2));
    ref_x_robot(i) = (ref_x_vector[(ref_x_vector.size() -1)] - x(0))*cos(x(2)) + (ref_y_vector[(ref_y_vector.size() - 1)] - x(1))*sin(x(2));
  }
}

void Sinus::change_f(double new_value){
  this->f = new_value;
}

void Sinus::reset(){
  this->last_x = 0.0;
}


Line::Line(int c,double time_step,double line_offset){
  this->control_horizon = c;
  this->dt = time_step;
  this->last_x = 0.0;
  this->offset = line_offset;
}

void Line::get_reference(Eigen::VectorXd x,std::vector<double>& ref_x_vector ,std::vector<double>& ref_y_vector,Eigen::VectorXd& ref_x_robot,Eigen::VectorXd& ref_y_robot){
  if(x(0) > this->last_x){
    this->last_x = x(0);
  }
  for(int i = 0; i < this->control_horizon; i++){
    ref_x_vector.push_back(this->last_x + 0.5+x(3)*i*this->dt);
    ref_y_vector.push_back(this->offset);

    ref_y_robot(i) = (ref_y_vector[ref_y_vector.size() - 1] - x(1))*cos(x(2)) - (ref_x_vector[ref_x_vector.size() - 1] - x(0))*sin(x(2));
    ref_x_robot(i) = (ref_x_vector[(ref_x_vector.size() -1)] - x(0))*cos(x(2)) + (ref_y_vector[(ref_y_vector.size() - 1)] - x(1))*sin(x(2));
  }
}

void Line::reset(){
  this->last_x = 0.0;
}
