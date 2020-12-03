#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "lib/Matplotlib/matplotlibcpp.h"
#include "lib/io/lib_io.h"
#include "lib/mpc/lib_mpc.h"
#include <stdio.h>
#include <fstream>
#include <sstream>
#include "lib/nn/lib_nn.h"
#include "lib/scan/scan_simulator_2d.hpp"
#include "lib/scan/pose_2d.hpp"

// #include <boost/algorithm/string.hpp>
// Create an output filestream object
const double PI = 3.141592653;
std::ofstream results_file;
std::ofstream dataset_file;
std::ofstream dataset_file_nn;
std::ofstream result_file_mpc;
std::ofstream inputs_file_nn;
// std::ofstream save_map_file;
std::ifstream traj_file;
std::ifstream out_contour_file;
std::ifstream in_contour_file;

double time_mpc = 0.0;
double time_nn = 0.0;
int count_time = 0;
const double pi = std::acos(-1);

namespace plt = matplotlibcpp;

double ti = 0.0, tf = T;
int time_step_counter = 0, pause_flag = 0;
int ms_per_frame = 5;
double trans_lastx;
double trans_corr = -1;

std::vector<double> result_x;
std::vector<double> result_y;

std::vector<double> result_x_nn;
std::vector<double> result_y_nn;

std::vector<double> result_psi;
std::vector<double> result_psi_nn;
double result_cost;

int point_idx;
int point_idx_nn;

std::vector<double> ref_x_vector_w;
std::vector<double> ref_y_vector_w;
std::vector<double> ref_x_vector_nn;
std::vector<double> ref_y_vector_nn;


Eigen::VectorXd x(SIZE_STATE);//x y psi v err_offset err_psi
Eigen::VectorXd x_nn(SIZE_STATE);//x y psi v err_offset err_psi
Eigen::VectorXd u(SIZE_INPUT);//steering angle rate  acceleration
Eigen::VectorXd u_nn(SIZE_INPUT);//steering angle rate  acceleration
Eigen::VectorXd ref_x(CONTROL_HORIZON); //vector with reference x positions
Eigen::VectorXd ref_y(CONTROL_HORIZON); //vector with reference y positions
Eigen::VectorXd ref_x_nn(CONTROL_HORIZON);//vector with reference x positions
Eigen::VectorXd ref_y_nn(CONTROL_HORIZON); //vector with reference y positions


Eigen::VectorXd ref_x_robot(CONTROL_HORIZON); //vector with reference x positions
Eigen::VectorXd ref_y_robot(CONTROL_HORIZON); //vector with reference y positions
Eigen::VectorXd ref_theta_robot(CONTROL_HORIZON); //vector with reference theta positions
Eigen::VectorXd ref_x_robot_nn(CONTROL_HORIZON); //vector with reference x positions
Eigen::VectorXd ref_y_robot_nn(CONTROL_HORIZON); //vector with reference y positions
Eigen::VectorXd ref_theta_robot_nn(CONTROL_HORIZON); //vector with reference theta positions

NN_CONTROLLER neural_controller;
std::vector<double> out_contour_x;
std::vector<double> out_contour_y;
std::vector<double> in_contour_x;
std::vector<double> in_contour_y;


std::vector<double> traj_x;
std::vector<double> traj_y;
std::vector<double> traj_v;
std::vector<double> traj_yaw;

double last_x,last_y,last_x_nn;

double angle2follow;
double angle2follow_nn;

std::string trained_nn_file;

int no_of_neurons;

std::string path;

double y_idx = 100.0;
double theta_idx = 0.0;
double freq = 1.0;
int next_neuron = 100;
int next_inputs = 40;
int configuration = 1;

int map_init = 0;
racecar_simulator::Pose2D origin;
double map_resolution;
double free_thresh;
int map_height;
int map_width;
std::vector<double> map;

std::vector<double> current_scan;
std::vector<double> current_scan_nn;
std::vector<double> scan_angles;
std::vector<double> cosines;
std::vector<double> car_distances;
double width = 0.28;
double scan_ang_incr;
double scan_distance_to_base_link = 0.275;
int scan_beams = 1080;
double scan_fov = 4.7;
double scan_std_dev = 0.0;
double angle_max = scan_fov/ 2.0;
double angle_min = -scan_fov / 2.0;
double angle_inc = scan_fov / scan_beams;
double usable_laser_range = 3.83972;
racecar_simulator::ScanSimulator2D scan_simulator;
double margin_relative = 0.0;
int last_split = 0;
int last_split_nn = 0;

int find_left_right_border(Eigen::MatrixXf points){
    int margin = (int)(points.rows() * margin_relative);

    int coeff = 1;
    int value = 0;
    Eigen::MatrixXf points_left(points.rows()-1-(2*margin),2);
    Eigen::MatrixXf points_right(points.rows()-1-(2*margin),2);
    Eigen::MatrixXf relative(points.rows()-1-(2*margin),2);

    points_left(0,0) = points(margin + 1,0);
    points_left(0,1) = points(margin + 1,1);

    points_right(0,0) = points(margin,0);
    points_right(0,1) = points(margin,1);

    int left_count = 1;
    int right_count = 1;

    for(int i = margin + 1; i < points.rows() -margin; i ++){
      if((i <= points.rows() -1 -margin)   && (i > margin + 1)){
        points_left(left_count,0) = points(i,0);
        points_left(left_count,1) = points(i,1);
        left_count = left_count + 1;
      }

      if((i <= points.rows() -1 -margin - 1)){
        points_right(right_count,0) = points(i,0);
        points_right(right_count,1) = points(i,1);
        right_count = right_count + 1;
      }
    }
    relative = points_left - points_right;

    auto distances = relative.rowwise().norm();
    double max_dist = distances.maxCoeff();

    int max_notFound = 1;
    int index = 0;
    int count_iter = 0;

    while(max_notFound){
      if(distances(count_iter) == max_dist){
        index = count_iter;
        max_notFound = 0;
      }
      count_iter ++ ;
    }

    // std::cout <<"max_dist = " << max_dist<<std::endl;
    // std::cout <<"index = " << index << std::endl;
    // std::cout << "points.rows() = "<< points.rows() <<std::endl;
    if (index ==  0 || index == points.rows() - 1 ){
      coeff = -1;
      value = index;
    }
    else{
      value = margin + 1 + index;
    }
    return coeff*(value);

  }


Eigen::MatrixXf get_scan_as_cartesian(std::vector<double> msg){
    std::vector<double>  ranges = msg;
    std::vector<double>  ranges_aux;
    std::vector<double>  angles_aux;

    int skip_left,skip_right = 0;

    double laser_range = angle_max - angle_min;
    double increment = laser_range/(ranges.size()-1);
    std::vector<double>  angles;

    for(int i = 0; i < ranges.size(); i++){
      angles.push_back(angle_min + (i*increment));
    }


    if(usable_laser_range < laser_range){
      skip_left = (int)((( -angle_min - (usable_laser_range) / 2) / laser_range)  * ranges.size());
      skip_right = (int)((( angle_max - (usable_laser_range) / 2) / laser_range) * ranges.size() );
      angles.erase(angles.begin(),angles.begin()+skip_left);
      angles.erase(angles.end() - skip_right -1 ,angles.end());

      ranges.erase(ranges.begin(),ranges.begin()+skip_left);
      ranges.erase(ranges.end() - skip_right -1,ranges.end());
    }


    for(int i = 0; i < ranges.size(); i++){
      if(ranges.at(i) != std::numeric_limits<double>::infinity() && ranges.at(i) < 5.0){
        ranges_aux.push_back(ranges.at(i));
        angles_aux.push_back(angles.at(i));
      }
    }
    ranges = ranges_aux;
    angles = angles_aux;
    Eigen::MatrixXf points(ranges.size(), 2);

    for(int i = 0; i < ranges.size(); i++){
      points(i,0) = -sin(angles.at(i))*ranges.at(i);
      points(i,1) = cos(angles.at(i))*ranges.at(i);
    }

    return points;

  }



std::vector<double>  update_scan(Eigen::VectorXd x) {
    // std::cout<<"begin update\n";
    // std::cout<<"x[0] = " <<x[0]<<"\n";
    // std::cout<<"x[1] = " <<x[1]<<"\n";
    // std::cout<<"x[2] = " <<x[2]<<"\n";
    racecar_simulator::Pose2D scan_pose;
    scan_pose.x = x[0] + scan_distance_to_base_link*std::cos(x[2]);
    scan_pose.y = x[1] + scan_distance_to_base_link*std::sin(x[2]);
    scan_pose.theta = x[2];
    std::vector<double> scan = scan_simulator.scan(scan_pose);
    // for (int i =0; i< scan.size(); i++)
    //     std::cout <<"scan = "<< scan.at(i) <<std::endl;
    // std::cout<<"end update\n";
    return scan;
}

void init_map(){
  std::string map_path = "/home/maria/f110_ws/src/f1tenth_lix_ifac_2020/maps/berlin.yaml";
  std::string map_img_path = "/home/maria/f110_ws/src/f1tenth_lix_ifac_2020/maps/berlin.png";
  cv::Mat img = imread(map_img_path, cv::IMREAD_GRAYSCALE );
  cv::flip(img, img, 0);
  map_height = img.rows;
  map_width = img.cols;
  std::ifstream inFile;
  double x;
  inFile.open("/home/maria/berlin-map.txt");
  while (inFile >> x) {
        map.push_back(x);
  }

  inFile.close();
  // std::cout <<"map_height = "<<map_height<<std::endl;
  // std::cout <<"map_width = "<<map_width<<std::endl;
  free_thresh = 0.6;
  map_resolution = 0.05;
  origin.x =  -11.606540;
  origin.y =  -26.520793;
  origin.theta = 0.000000;
  map_init = 1;
  // std::cout <<"len(map)= "<<map.size()<<std::endl;



  // colorMat = colorMat.reshape (1, colorMat.rows * colorMat.cols);
  // map = (1. - img/255.).flatten();//.tolist();

  // std::cout <<"map.size = "<<map.size<<std::endl;


}

void glInitMPC(double x0,double x1, double x2, std::string path_option,int neurons,double freq_choice, std::string file_name){
  no_of_neurons = neurons;
  std::cout <<"no_of_neurons = "<<no_of_neurons<<std::endl;
  freq = freq_choice;
  x(0) = x0; //fx
  x(1) = x1; //y
  x(2) = x2; //theta
  x(3) = X3; //speed
  path = path_option;

  x_nn(0) = x0; //x
  x_nn(1) = x1; //y
  x_nn(2) = x2; //theta
  x_nn(3) = X3; //speed
  init_map();

  if(REVERSE){
    angle2follow = 20.0;
    angle2follow_nn = 20.0;
  }
  else{
    angle2follow = 0.0;
    angle2follow_nn = 0.0;
  }

  scan_simulator = racecar_simulator::ScanSimulator2D(scan_beams, scan_fov, scan_std_dev);
  scan_simulator.set_map(map,map_height,map_width,map_resolution,origin,free_thresh);
  current_scan = std::vector<double>(scan_beams);
  current_scan_nn = std::vector<double>(scan_beams);
  scan_angles = std::vector<double>(scan_beams);
  scan_ang_incr = scan_simulator.get_angle_increment();

  // precomp consines and distance from lidar to edge of car
    cosines = std::vector<double>(scan_beams);
    car_distances = std::vector<double>(scan_beams);
    double dist_to_sides = width/2.0;
    double dist_to_front = (0.3302 - 0.17145);
    double dist_to_back = scan_distance_to_base_link;

    for (int i=0; i<scan_beams; i++) {
        double angle = -scan_fov/2.0 + i*scan_ang_incr;
        scan_angles[i] = angle;
        cosines[i] = std::cos(angle);
        if (angle > 0) {
            if (angle < PI/2.0) {
                // between 0 and pi/2
                double to_side = dist_to_sides/std::sin(angle);
                double to_front = dist_to_front/std::cos(angle);
                car_distances[i] = std::min(to_side, to_front);
            } else {
                // between pi/2 and pi
                double to_side = dist_to_sides/std::cos(angle-PI/2.0);
                double to_back = dist_to_back/std::sin(angle-PI/2.0);
                car_distances[i] = std::min(to_side, to_back);
            }
        } else {
            if (angle > -PI/2.0) {
                // between 0 and -pi/2
                double to_side = dist_to_sides/std::sin(-angle);
                double to_front = dist_to_front/std::cos(-angle);
                car_distances[i] = std::min(to_side, to_front);
            } else {
                // between -pi/2 and -pi
                double to_side = dist_to_sides/std::cos(-angle-PI/2.0);
                double to_back = dist_to_back/std::sin(-angle-PI/2.0);
                car_distances[i] = std::min(to_side, to_back);
            }
        }
    }

  current_scan = update_scan(x);
  current_scan_nn = update_scan(x_nn);
  // Eigen::MatrixXf points;
  // points = get_scan_as_cartesian(current_scan);
  if (no_of_neurons == 0 ){
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/3 neurons/training_minimum_cost/1903_1019/trained_nn_3";
  }

  else if (no_of_neurons == 3 ){
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/3 neurons/training_minimum_cost/0104_1700/trained_nn_3";//line
  }

  else if (no_of_neurons == 5 ){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/5 neurons/training_minimum_cost/0904_1200/trained_nn_5";//line
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/5 neurons/training_minimum_cost/2704_1414/trained_nn_5";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/5 neurons/training_minimum_cost/2704_2244/trained_nn_5";//line,sinus and spiral
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/5 neurons/training_minimum_cost/1406_1500/trained_nn_5";//lines 3 inp
  }

  else if (no_of_neurons == 10){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/10 neurons/training_minimum_cost/0904_1200/trained_nn_10";//line
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/10 neurons/training_minimum_cost/2704_1414/trained_nn_10";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/10 neurons/training_minimum_cost/2704_2244/trained_nn_10";//line,sinus and spiral
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/10 neurons/training_minimum_cost/1905_1730/trained_nn_10";//line,sinus and spiral tanh 9 couches
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/10 neurons/training_minimum_cost/2705_1550/trained_nn_10";//line,sinus and spiral relu 9 couches
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/10 neurons/training_minimum_cost/1406_1500/trained_nn_10";//lines 3 inp
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_3inp_1hiddlay_10neurons_dataset2_activationf0";//keras 3 inp
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_21inp_1hiddlay_10neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_20neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_40inp_1hiddlay_10neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_3hiddlay_30neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/30_10/training_final_cost_40inp_3hiddlay_30neurons_dataset0_activationf0";
    trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/30_10/training_final_cost_40inp_3hiddlay_30neurons_dataset2_activationf2";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/30_10/training_final_cost_40inp_3hiddlay_30neurons_dataset1_activationf0";
  }

  else if (no_of_neurons == 15){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/15 neurons/training_minimum_cost/0904_1200/trained_nn_15";//line
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/15 neurons/training_minimum_cost/2704_1414/trained_nn_15";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/15 neurons/training_minimum_cost/2704_2244/trained_nn_15";//line,sinus and spiral
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/15 neurons/training_minimum_cost/1406_1500/trained_nn_15";//lines 3 inp
    trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_30neurons_dataset2_activationf0";
  }

  else if (no_of_neurons == 20){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/20 neurons/training_minimum_cost/1504_1500/trained_nn_20";//line
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/20 neurons/training_minimum_cost/2704_1414/trained_nn_20";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/20 neurons/training_minimum_cost/2704_2244/trained_nn_20";//line,sinus and spiral
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/20 neurons/training_minimum_cost/1406_1500/trained_nn_20";//lines 3 inp


    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_40inp_1hiddlay_20neurons_dataset2_activationf0";//keras
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_21inp_1hiddlay_20neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_3inp_1hiddlay_20neurons_dataset2_activationf0";


    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_20neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_40neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/16_10/training_final_cost_40inp_3hiddlay_20neurons_dataset2_activationf0";
    trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_3hiddlay_60neurons_dataset2_activationf0";
  }

  else if (no_of_neurons == 25){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/25 neurons/training_minimum_cost/1504_1500/trained_nn_25";//line
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/25 neurons/training_minimum_cost/2704_1414/trained_nn_25";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/25 neurons/training_minimum_cost/2704_2244/trained_nn_25";//line,sinus and spiral
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/25 neurons/training_minimum_cost/1406_1500/trained_nn_25";//lines 3 inp
    trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_50neurons_dataset2_activationf0";
  }

  else if (no_of_neurons == 30){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/30 neurons/training_minimum_cost/1504_1500/trained_nn_30";//line
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/30 neurons/training_minimum_cost/2704_1414/trained_nn_30";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/30 neurons/training_minimum_cost/2704_2244/trained_nn_30";//line,sinus and spiral
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/30 neurons/training_minimum_cost/1406_1500/trained_nn_30";//lines 3 inp


    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_40inp_1hiddlay_30neurons_dataset2_activationf0";//keras
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_3inp_1hiddlay_30neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_21inp_1hiddlay_30neurons_dataset2_activationf0";

    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_30neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/16_10/training_final_cost_40inp_3hiddlay_30neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_60neurons_dataset2_activationf0";
    trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_3hiddlay_90neurons_dataset2_activationf0";
  }

  else if (no_of_neurons == 35){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/35 neurons/training_minimum_cost/1504_1500/trained_nn_35";//line
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/35 neurons/training_minimum_cost/2704_1414/trained_nn_35";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/35 neurons/training_minimum_cost/2704_2244/trained_nn_35";//line,sinus and spiral
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/35 neurons/training_minimum_cost/1406_1500/trained_nn_35";//lines 3 inp
    trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_70neurons_dataset2_activationf0";
  }

  else if (no_of_neurons == 40){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/40 neurons/training_minimum_cost/1504_1500/trained_nn_40";//line
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/40 neurons/training_minimum_cost/2704_1414/trained_nn_40";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/40 neurons/training_minimum_cost/2704_2244/trained_nn_40";//line,sinus and spiral
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/40 neurons/training_minimum_cost/1406_1500/trained_nn_40";//lines 3 inp


    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_3inp_1hiddlay_40neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_21inp_1hiddlay_40neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_40inp_1hiddlay_40neurons_dataset2_activationf0";


    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_80neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/16_10/training_final_cost_40inp_3hiddlay_40neurons_dataset2_activationf0";
    trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_3hiddlay_120neurons_dataset2_activationf0";
  }

  else if (no_of_neurons == 45){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/45 neurons/training_minimum_cost/1504_1500/trained_nn_45";//line
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/45 neurons/training_minimum_cost/2704_1414/trained_nn_45";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/45 neurons/training_minimum_cost/2704_2244/trained_nn_45";//line,sinus and spiral
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/45 neurons/training_minimum_cost/1406_1500/trained_nn_45";//lines 3 inp
    trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_90neurons_dataset2_activationf0";
  }

  else if (no_of_neurons == 50){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/50 neurons/training_minimum_cost/1504_1500/trained_nn_50";//line relu
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/2704_1414/trained_nn_50";//line and sinus relu
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/2704_2244/trained_nn_50";//line,sinus and spiral relu
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/0505_1600/trained_nn_50";//line,sinus sigmoid
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/1105_1400/trained_nn_50";//line,sinus  tanh
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/1805_0808/trained_nn_50";//line,sinus and spiral sigmoid
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/1805_1215/trained_nn_50";//line,sinus and spiral tanh
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/1406_1500/trained_nn_50";//lines 3 inp
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/08_10/training_final_cost_40inp_1hiddlay_50neurons_dataset2_activationf0";//with keras 150 epochs


    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_3inp_1hiddlay_50neurons_dataset2_activationf0";//with keras 10000 epochs3 inputs
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_21inp_1hiddlay_50neurons_dataset2_activationf0";
    trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_40inp_1hiddlay_50neurons_dataset2_activationf0";

    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_100neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_50neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/16_10/training_final_cost_40inp_3hiddlay_50neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_3hiddlay_150neurons_dataset2_activationf0";
  }

  else if (no_of_neurons == 55){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/55 neurons/training_minimum_cost/1504_1500/trained_nn_55";//line
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/55 neurons/training_minimum_cost/2704_1414/trained_nn_55";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/55 neurons/training_minimum_cost/2704_2244/trained_nn_55";//line,sinus and spiral
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/55 neurons/training_minimum_cost/1406_1500/trained_nn_55";//lines 3 inp
  }

  else if (no_of_neurons == 60){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/60 neurons/training_minimum_cost/2304_0930/trained_nn_60";//line
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/60 neurons/training_minimum_cost/2704_1414/trained_nn_60";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/60 neurons/training_minimum_cost/2704_2244/trained_nn_60";//line,sinus and spiral
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/60 neurons/training_minimum_cost/1406_1500/trained_nn_60";//lines 3 inp
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_21inp_1hiddlay_60neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_3inp_1hiddlay_60neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_40inp_1hiddlay_60neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_60neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_2hiddlay_120neurons_dataset2_activationf0";
    trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_3hiddlay_180neurons_dataset2_activationf0";
  }

  else if (no_of_neurons == 65){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/65 neurons/training_minimum_cost/2304_0930/trained_nn_65";//line
  }

  else if (no_of_neurons == 70){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/70 neurons/training_minimum_cost/2304_0930/trained_nn_70";//line
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_21inp_1hiddlay_70neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_3inp_1hiddlay_70neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_40inp_1hiddlay_70neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_70neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_2hiddlay_140neurons_dataset2_activationf0";
    trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_3hiddlay_210neurons_dataset2_activationf0";
  }


  else if (no_of_neurons == 75){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/75 neurons/training_minimum_cost/2304_0930/trained_nn_75";
  }

  else if (no_of_neurons == 80){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/80 neurons/training_minimum_cost/2304_0930/trained_nn_80";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_3inp_1hiddlay_80neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_21inp_1hiddlay_80neurons_dataset2_activationf0";
      // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_40inp_1hiddlay_80neurons_dataset2_activationf0";
      // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_80neurons_dataset2_activationf0";
      // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_2hiddlay_160neurons_dataset2_activationf0";
      trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_3hiddlay_240neurons_dataset2_activationf0";
  }


  else if (no_of_neurons == 85){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/85 neurons/training_minimum_cost/2304_0930/trained_nn_85";
  }

  else if (no_of_neurons == 90){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/90 neurons/training_minimum_cost/2304_0930/trained_nn_90";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_3inp_1hiddlay_90neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_21inp_1hiddlay_90neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_40inp_1hiddlay_90neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_90neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_2hiddlay_180neurons_dataset2_activationf0";
    trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_3hiddlay_270neurons_dataset2_activationf0";
  }

  else if (no_of_neurons == 95){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/95 neurons/training_minimum_cost/2304_0930/trained_nn_95";
  }

  else if (no_of_neurons == 100){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/100 neurons/training_minimum_cost/2304_0930/trained_nn_100";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_3inp_1hiddlay_100neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_21inp_1hiddlay_100neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/09_10/training_final_cost_40inp_1hiddlay_100neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/18_10/training_final_cost_40inp_2hiddlay_100neurons_dataset2_activationf0";
    // trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_2hiddlay_200neurons_dataset2_activationf0";
    trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/26_10/training_final_cost_40inp_3hiddlay_300neurons_dataset2_activationf0";
  }



  else if (no_of_neurons == 105){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/105 neurons/training_minimum_cost/2304_0930/trained_nn_105";
  }

  else if (no_of_neurons == 110){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/110 neurons/training_minimum_cost/2304_0930/trained_nn_110";
  }

  else if (no_of_neurons == 115){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/115 neurons/training_minimum_cost/2304_0930/trained_nn_115";
  }

  else if (no_of_neurons == 120){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/120 neurons/training_minimum_cost/2304_0930/trained_nn_120";
  }

  else if (no_of_neurons == 125){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/125 neurons/training_minimum_cost/2304_0930/trained_nn_125";
  }

  else if (no_of_neurons == 130){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/130 neurons/training_minimum_cost/2304_0930/trained_nn_130";
  }

  else if (no_of_neurons == 135){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/135 neurons/training_minimum_cost/2304_0930/trained_nn_135";
  }

  else if (no_of_neurons == 140){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/140 neurons/training_minimum_cost/2304_0930/trained_nn_140";
  }

  else if (no_of_neurons == 145){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/145 neurons/training_minimum_cost/2304_0930/trained_nn_145";
  }

  else if (no_of_neurons == 150){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/150 neurons/training_minimum_cost/2304_0930/trained_nn_150";
  }

  else if (no_of_neurons == 155){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/155 neurons/training_minimum_cost/2304_0930/trained_nn_155";
  }

  else if (no_of_neurons == 400){
    // trained_nn_file = "/home/maria/Documents/DDPG_1/trained_nn/neurocar2111-v0-20000_trainednn";
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar2-v0-6800_trainednn";
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0212-v4-1800_trainednn";
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0212-v6-1800_trainednn";
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0212-v7-28000_trainednn";
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0212-v8-600_trainednn"; //first with 0.8 -  the best one- plus index 20
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0212-v9-21000_trainednn"; //igual a v8 mas com mais tempo de treino - nao funciona bem
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0212-v8-1000_trainednn";
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0212-v8-12000_trainednn";
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0312-v0-2000_trainednn"; //plus idx 10
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0312-v0-3000_trainednn";
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0312-v1-2800_trainednn"; //using eric reward plus idx 10
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0312-v2-4400_trainednn"; //using eric reward with sigmoid plus idx20
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0312-v3-1400_trainednn"; //using ancient reward with sigmoid
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0312-v3-13200_trainednn"; //using ancient reward with sigmoid
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0312-v4-600_trainednn"; //using ancient reward with sigmoid plus_idx = 0
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0312-v5-2400_trainednn"; //v5 = v4 plus_idx = 0
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0312-v6-600_trainednn"; //2 inputs  plus_idx = 10
    // trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0312-v6-1000_trainednn"; //2 inputs best result plus_idx = 10
    trained_nn_file = "/home/maria/Documents/neurocar_ddpg/trained_nn/neurocar0312-v6-1000_trainednn"; //2 inputs best result plus_idx = 10

  }



  time_step_counter = 0;
  pause_flag = 0;
  trans_corr = -1;

  for(int i = 0; i < SIZE_INPUT; i++){
    u(i) = 0.0;
    u_nn(i) = 0.0;
  }
  // std::cout <<"u begin all= "<<u<<std::endl;
  //  <<"u_nn begin all= "<<u_nn<<std::endl;
  last_x = x0;
  last_x_nn = x0;

  //angle2follow = 0.0;
  // angle2follow_nn = 0.0;
  // Send column names to the stream
  if(RESULTS_FILE == 1 && !dataset_file.is_open()){
    dataset_file.open("/home/maria/Documents/MPC/mpc_controller/dataset/dataset_04_lines_sinus_spiral.txt", std::ios_base::app);
  }

  if (RESULTS_MPC_FILE== 1){
    result_file_mpc.open("/home/maria/Documents/MPC/mpc_controller/spiral_mpc.txt");
  }

  if(RESULTS_NN_FILE == 1){
    dataset_file_nn.open(file_name, std::ios_base::app);
  }

  if(INPUTS_NN_FILE == 1){
    inputs_file_nn.open("../inputs_nn/line_tanh.txt");
  }

  // save_map_file.open("../berlin_pts.txt");
  traj_file.open("../berlin_pts.txt");
  out_contour_file.open("../out_contour_berlin.txt");
  in_contour_file.open("../in_contour_berlin.txt");

  std::string line;

  while (std::getline(out_contour_file, line)){
    std::vector<double> elems;
    std::stringstream ss(line);
    std::string number;
    while(std::getline(ss, number, ',')) {
        elems.push_back(std::stod(number));
    }
    out_contour_x.push_back(elems[0]);
    out_contour_y.push_back(elems[1]);

  }
  std::reverse( out_contour_x.begin(), out_contour_x.end() );
  std::reverse( out_contour_y.begin(), out_contour_y.end() );
  while (std::getline(in_contour_file, line)){
    std::vector<double> elems;
    std::stringstream ss(line);
    std::string number;
    while(std::getline(ss, number, ',')) {
        elems.push_back(std::stod(number));
    }
    in_contour_x.push_back(elems[0]);
    in_contour_y.push_back(elems[1]);

  }
  // std::reverse( in_contour_x.begin(), in_contour_x.end() );
  // std::reverse( in_contour_y.begin(), in_contour_y.end() );
  in_contour_file.close();
  out_contour_file.close();
  while (std::getline(traj_file, line)){
    // std::vector<std::string> results;
    // std::replace(line.begin(), line.end(), ':', ' ');  // replace ':' by ' '
    // std::vector<std::string> array;
    // stringstream ss(str);
    // int temp;
    // while (ss >> temp)
    //   array.push_back(temp); // done! now array={102,330,3133,76531,451,000,12,44412}
    // boost::algorithm::split(results, line, boost::is_any_of(","));
    // traj_x.push_back(std::stod (results[0],&sz));
    // traj_y.push_back(std::stod (results[1],&sz));
    // traj_v.push_back(std::stod (results[4],&sz));
    // traj_yaw.push_back(std::stod (results[2],&sz));
    std::vector<double> elems;
    std::stringstream ss(line);
    std::string number;
    while(std::getline(ss, number, ',')) {
        elems.push_back(std::stod(number));
    }
    traj_x.push_back(elems[0]);
    traj_y.push_back(elems[1]);

  }

  traj_file.close();
  neural_controller.initialize_nn(trained_nn_file);

}

// std::vector<int>  find_closest_point_map(Eigen::VectorXd  x){
//     std::vector<int> indexes(2);//in first and then out
//     double x_r = x[0];
//     double y_r = x[1];
//     //  <<"x = "<<x_r <<" , "<<y_r<<std::endl;
//     int result;
//
//     double distance_in ;
//     double distance_out ;
//     double x_i;
//     double y_i ;
//
//     double smallest_in =10000;
//     double smallest_out =10000;
//     int index_smallest_in,index_smallest_out;
//
//     for(int i=0; i < in_contour_x.size();i++){
//       x_i = in_contour_x.at(i) - x_r;
//       y_i = in_contour_y.at(i) - y_r;
//       distance_in = sqrt((x_i*x_i) + (y_i*y_i));
//
//       if(distance_in < smallest_in){
//         smallest_in = distance_in;
//         index_smallest_in = i;
//       }
//     }
//
//     for(int i=0; i < out_contour_x.size();i++){
//       x_i = out_contour_x.at(i) - x_r;
//       y_i = out_contour_y.at(i) - y_r;
//       distance_out = sqrt((x_i*x_i) + (y_i*y_i));
//
//       if(distance_out < smallest_out){
//         smallest_out = distance_out;
//         index_smallest_out = i;
//       }
//     }
//
//     // std::vector<double> target_point(2);
//     //
//     indexes.at(0) = index_smallest_in;
//     indexes.at(1) = index_smallest_out;
//
//
//     return indexes;
//
//
// }

int find_closest_point_map(Eigen::VectorXd  x){
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

std::vector<double> get_closest_point(Eigen::MatrixXd wall,std::vector<double> predicted_car_position){
    std::vector<double> result(2);
    double distance ;
    double x;
    double y ;
    double smallest =10000;
    int index_smallest;

    for(int i=0; i < wall.rows();i++){
      x = wall(i,0) - predicted_car_position.at(0);
      y = wall(i,1) - predicted_car_position.at(1);
      distance = sqrt((x*x) + (y*y));
      if(distance < smallest){
        smallest = distance;
        index_smallest = i;
      }
    }

    result.at(0) = wall(index_smallest,0);
    result.at(1) = wall(index_smallest,1);
    return result;

  }

void CalculateTrajectory(){
  if (path == "race"){
    int plus_idx = 0;
    int nex_idx = 0;

    point_idx = find_closest_point_map(x);

    for(int i = 0; i < CONTROL_HORIZON; i++){
      if (!REVERSE){
        if(point_idx + plus_idx+ i < traj_x.size())
          nex_idx = point_idx + i + plus_idx ;
        else
          nex_idx =  (point_idx + plus_idx+ i) - traj_x.size();
      }
      else{
        if(point_idx - plus_idx - i >= 0)
          nex_idx = point_idx - plus_idx - i;
        else
          nex_idx =  (point_idx - plus_idx - i) + traj_x.size();
      }

      ref_x(i) = traj_x.at(nex_idx);
      ref_y(i) = traj_y.at(nex_idx);


      ref_x_vector_w.push_back(ref_x(i));
      ref_y_vector_w.push_back(ref_y(i));
    }

    point_idx_nn = find_closest_point_map(x_nn);
    plus_idx = 10;
    for(int i = 0; i < CONTROL_HORIZON; i++){
      if (!REVERSE){
        if(point_idx_nn + plus_idx+ i < traj_x.size())
          nex_idx = point_idx_nn + i + plus_idx ;
        else
          nex_idx =  (point_idx_nn + plus_idx+ i) - traj_x.size();
      }
      else{
        if(point_idx_nn - plus_idx - i >= 0)
          nex_idx = point_idx_nn - plus_idx - i;
        else
          nex_idx =  (point_idx_nn - plus_idx - i) + traj_x.size();
      }

      ref_x_nn(i) = traj_x.at(nex_idx);
      ref_y_nn(i) = traj_y.at(nex_idx);

      // ref_x_nn(i) = ref_x(i);
      // ref_y_nn(i) = ref_y(i);

      ref_x_vector_nn.push_back(ref_x_nn(i));
      ref_y_vector_nn.push_back(ref_y_nn(i));
    }
  }

  else if(path == "quad"){
    double aux_quad;
    double aux_ref_x;
    double aux_center;
    if(x(0) < 5.0){
      aux_ref_x = 5.0;
      aux_center = 10;
    }
    else{
      aux_center = (int)round(x(0)/10.0) * 10;
      aux_ref_x = x(0);
    }

    for(int i = 0; i < CONTROL_HORIZON; i++){
      if( x(0) + 0.5>= last_x){
        ref_x(i) = aux_ref_x + 0.5 + i*DT;
        aux_quad =  ref_x(i) - aux_center;
        ref_y(i) = -0.2*(aux_quad*aux_quad)  + 5;
        last_x = aux_ref_x + 0.5;
      }
      else{
        ref_x(i) = last_x +  i*DT;
        aux_quad =  ref_x(i) - aux_center;
        ref_y(i) = -0.2*(aux_quad*aux_quad)  + 5;
      }

      ref_x_vector_w.push_back(ref_x(i));
      ref_y_vector_w.push_back(ref_y(i));

    }

    if(x_nn(0) < 5.0){
      aux_ref_x = 5.0;
      aux_center = 10;
    }
    else{
      aux_center = (int)round(x_nn(0)/10.0) * 10;
      aux_ref_x = x_nn(0);
    }

    for(int i = 0; i < CONTROL_HORIZON; i++){
      if( x_nn(0) + 0.5>= last_x_nn){
        ref_x_nn(i) = aux_ref_x + 0.5 + i*DT;
        aux_quad =  ref_x_nn(i) - aux_center;
        ref_y_nn(i) = -0.2*(aux_quad*aux_quad)  + 5;
        last_x_nn = aux_ref_x + 0.5;
      }
      else{
        ref_x_nn(i) = last_x_nn +  i*DT;
        aux_quad =  ref_x_nn(i) - aux_center;
        ref_y_nn(i) = -0.2*(aux_quad*aux_quad)  + 5;
      }

      ref_x_vector_nn.push_back(ref_x_nn(i));
      ref_y_vector_nn.push_back(ref_y_nn(i));

    }
  }

  else if(path == "circle"){
    //circle
    double cx = 0.0;
    double cy = 0.0;
    double circle_ray = angle2follow/3.14;
    double circle_ray_nn = angle2follow_nn/3.14;

    for(int i = 0; i < CONTROL_HORIZON; i++){
      if(x(0) > (circle_ray*cos(angle2follow) + cx - 1.5) && x(0) < (circle_ray*cos(angle2follow) + cx + 1.5)
        && x(1) > (circle_ray*sin(angle2follow) + cy - 1.5) && x(1) < (circle_ray*sin(angle2follow) + cy + 1.5)){
        double angle2follow_it = angle2follow + 1.0*i*DT;
        circle_ray = angle2follow_it/3.14;
        ref_x(i) = circle_ray*cos(angle2follow_it) + cx;
        ref_y(i) = circle_ray*sin(angle2follow_it) + cy;
        if(!REVERSE)
          angle2follow = angle2follow + 0.5*DT;
        else
          angle2follow = angle2follow - 0.5*DT;
      }
      else{
        double angle2follow_it = angle2follow + i*DT;
        ref_x(i) = circle_ray*cos(angle2follow_it) + cx;
        ref_y(i) = circle_ray*sin(angle2follow_it) + cy;
      }

      ref_x_vector_w.push_back(ref_x(i));
      ref_y_vector_w.push_back(ref_y(i));
    }

    for(int i = 0; i < CONTROL_HORIZON; i++){
      if(x_nn(0) > (circle_ray_nn*cos(angle2follow_nn) + cx - 1.5) && x_nn(0) < (circle_ray_nn*cos(angle2follow_nn) + cx + 1.5)
        && x_nn(1) > (circle_ray_nn*sin(angle2follow_nn) + cy - 1.5) && x_nn(1) < (circle_ray_nn*sin(angle2follow_nn) + cy + 1.5)){
        double angle2follow_it = angle2follow_nn + 1.0*i*DT;
        circle_ray_nn = angle2follow_it/3.14;
        ref_x_nn(i) = circle_ray_nn*cos(angle2follow_it) + cx;
        ref_y_nn(i) = circle_ray_nn*sin(angle2follow_it) + cy;
        if(!REVERSE)
          angle2follow_nn = angle2follow_nn + 0.5*DT;
        else
          angle2follow_nn = angle2follow_nn - 0.5*DT;
      }
      else{
        double angle2follow_it = angle2follow_nn + i*DT;
        ref_x_nn(i) = circle_ray_nn*cos(angle2follow_it) + cx;
        ref_y_nn(i) = circle_ray_nn*sin(angle2follow_it) + cy;
      }

      ref_x_vector_nn.push_back(ref_x_nn(i));
      ref_y_vector_nn.push_back(ref_y_nn(i));
    }

    // ref_x = ref_x_nn;
    // ref_y = ref_y_nn;
  }

  else if(path == "sinus"){
      double const_aux;
      double const_nn;

      // if(freq == 2.0){
      //   const_aux = x(3)*cos(x(2))*DT;
      //   const_nn = x_nn(3)*cos(x_nn(2))*DT;
      // }
      // else{
        const_aux = 0.0;
        const_nn = 0.5;
      // }
      double ref_v_x = (x(3))/sqrt(1+(sin(freq*(x(0) +const_aux))*sin(freq*(x(0) +const_aux))));
      for(int i = 0; i < CONTROL_HORIZON; i++){
        if( x(0) + const_aux > last_x){
          ref_x(i) = x(0) +const_aux + ref_v_x*DT*i;
          ref_y(i) = LINE + std::cos(freq*ref_x(i));
          last_x = x(0) + const_aux  ;
          ref_v_x = (x(3))/sqrt(1+(sin(freq*ref_x(i))*sin(freq*ref_x(i))));
        }
        else{
          ref_x(i) = x(0) +const_aux + ref_v_x*DT*i;
          ref_y(i) = LINE + cos(freq*ref_x(i));
          ref_v_x = (x(3))/sqrt(1+(sin(freq*ref_x(i))*sin(freq*ref_x(i))));
        }

        ref_x_vector_w.push_back(ref_x(i));
        ref_y_vector_w.push_back(ref_y(i));
      }

    double ref_v_x_nn = (x_nn(3))/sqrt(1+(sin(freq*(x_nn(0) +const_aux))*sin(freq*(x_nn(0) +const_aux))));
    // double ref_v_x_nn = 1.0;
    for(int i = 0; i < CONTROL_HORIZON; i++){
      if( x_nn(0) + const_nn > last_x_nn){
        ref_x_nn(i) = x_nn(0)  + const_nn  + ref_v_x_nn*i*DT;
        ref_y_nn(i) = LINE + cos(freq*ref_x_nn(i));
        last_x_nn = x_nn(0) + const_nn ;
        ref_v_x_nn = (x_nn(3))/sqrt(1+(sin(freq*ref_x_nn(i))*sin(freq*ref_x_nn(i))));
      }
      else{
        ref_x_nn(i) = last_x_nn + ref_v_x_nn*i*DT;
        ref_y_nn(i) = LINE + cos(freq*ref_x_nn(i));
        ref_v_x_nn = (x_nn(3))/sqrt(1+(sin(freq*ref_x_nn(i))*sin(freq*ref_x_nn(i))));
      }

      ref_x_vector_nn.push_back(ref_x_nn(i));
      ref_y_vector_nn.push_back(ref_y_nn(i));
    }

  }

  else if(path == "line"){
    for(int i = 0; i < CONTROL_HORIZON; i++){
      if(x(0)+0.5 > last_x){
        ref_x(i) = x(0) +  0.5+x(3)*i*DT;
        ref_y(i) = LINE;
        last_x = x(0)+0.5;
      }
      else{
        ref_x(i) = last_x   + i*DT;
        ref_y(i) = LINE;
      }

      ref_x_vector_w.push_back(ref_x(i));
      ref_y_vector_w.push_back(ref_y(i));
    }

    for(int i = 0; i < CONTROL_HORIZON; i++){
      if(x_nn(0) + 0.5> last_x_nn){
        ref_x_nn(i) = x_nn(0) + 0.5 +x_nn(3)*i*DT;
        ref_y_nn(i) = LINE;
        last_x_nn = x_nn(0) + 0.5;
      }
      else{
        ref_x_nn(i) = last_x_nn + i*DT;
        ref_y_nn(i) = LINE;
      }

      ref_x_vector_nn.push_back(ref_x_nn(i));
      ref_y_vector_nn.push_back(ref_y_nn(i));

    }
  }
}
void CalculateMPC(){
  // NN_CONTROLLER neural_controller(trained_nn_file);
  result_x.push_back(x(0));
  result_y.push_back(x(1));
  result_x_nn.push_back(x_nn(0));
  result_y_nn.push_back(x_nn(1));
  result_psi.push_back(x(2));
  result_psi_nn.push_back(x_nn(2));
  // std::cout <<"begin CalculateTrajectory\n"<<std::endl;
  CalculateTrajectory();
  // std::cout <<"end CalculateTrajectory\n"<<std::endl;
  for(int i = 0; i < CONTROL_HORIZON; i++){
    ref_y_robot(i) = (ref_y(i) - x(1))*cos(x(2)) - (ref_x(i) - x(0))*sin(x(2));
    ref_x_robot(i) = (ref_x(i) - x(0))*cos(x(2)) + (ref_y(i) - x(1))*sin(x(2));
  }

  for(int i = 0; i < CONTROL_HORIZON; i++){
    ref_y_robot_nn(i) = (ref_y_nn(i) - x_nn(1))*cos(x_nn(2)) - (ref_x_nn(i) - x_nn(0))*sin(x_nn(2));
    ref_x_robot_nn(i) = (ref_x_nn(i) - x_nn(0))*cos(x_nn(2)) + (ref_y_nn(i) - x_nn(1))*sin(x_nn(2));
    // std::cout <<"ref_x_nn(i) = " <<ref_x_nn(i)<<" ,ref_y_nn(i) = " << ref_y_nn(i) <<std::endl;
  }
  // std::cout <<"end change repere\n"<<std::endl;
  Eigen::VectorXd poly_coeffs_robot = Polyfit(ref_x_robot, ref_y_robot,3);
  double ref_offset_robot_poly = poly_coeffs_robot(0);
  double ref_theta_robot_poly = -atan(poly_coeffs_robot(1));


  Eigen::VectorXd poly_coeffs_robot_nn = Polyfit(ref_x_robot_nn, ref_y_robot_nn,3);
  double ref_offset_robot_nn = poly_coeffs_robot_nn(0);
  double ref_theta_robot_nn = -atan(poly_coeffs_robot_nn(1));
  // std::cout <<"poly_coeffs_robot_nn(1) = "<<poly_coeffs_robot_nn(1)<<std::endl;
  if(std::isnan(ref_theta_robot_nn ) ){
    if((ref_x_robot_nn(CONTROL_HORIZON - 1) - ref_x_robot_nn(0)) != 0.0 )
      ref_theta_robot_nn = atan((ref_y_robot_nn(CONTROL_HORIZON - 1) - ref_y_robot_nn(0))/(ref_x_robot_nn(CONTROL_HORIZON - 1) - ref_x_robot_nn(0)));
    else
      ref_theta_robot_nn = pi/2.0;
  }
  Eigen::VectorXd poly_coeffs = Polyfit(ref_x, ref_y,3);
  double ref_offset_poly = poly_coeffs(0);
  double ref_theta_world_poly = -atan(poly_coeffs(1));

  Eigen::VectorXd poly_coeffs_nn = Polyfit(ref_x_nn, ref_y_nn,3);
  double ref_offset_nn = poly_coeffs_nn(0);
  double ref_theta_nn = -atan(poly_coeffs_nn(1));
  // std::cout <<"ref_theta_robot_poly  = "<< ref_theta_robot_poly << "  ,x(2)  = "<< x(2)<<std::endl;
  x = Propagate(x,u);
  if (no_of_neurons > 0){
    x_nn = Propagate(x_nn,u_nn);
  }
  // std::cout <<"end propagate\n"<<std::endl;
  current_scan = update_scan(x);
  current_scan_nn = update_scan(x_nn);
  // std::cout <<"end update_scan\n"<<std::endl;
  std::vector<double> result_mpc;
  std::vector<double> result_mpc_nn;
  clock_t begin_time_mpc = std::clock();
  result_mpc = Solve_MPC(x,ref_x,ref_y,0.0,0.0);
  clock_t end_time_mpc = std::clock();
  // std::cout <<"mpc time = " << float( end_time_mpc - begin_time_mpc ) / CLOCKS_PER_SEC <<std::endl;;
  time_mpc = time_mpc + double( end_time_mpc - begin_time_mpc ) / CLOCKS_PER_SEC;
  result_mpc_nn = Solve_MPC(x_nn,ref_x_nn,ref_y_nn,0.0,0.0);
  // std::cout <<"end Solve_MPC\n"<<std::endl;
  // inputs 40
  // int count_idx = 0;
  // Eigen::VectorXd inputs_nn (2*CONTROL_HORIZON);
  // for(int i = 0; i < (2*CONTROL_HORIZON)-1;  i = i +2){
  //   inputs_nn(i) = ref_x_robot_nn(count_idx);
  //   inputs_nn(i+1) = ref_y_robot_nn(count_idx);
  //   if(INPUTS_NN_FILE == 1){
  //     inputs_file_nn << std::to_string(inputs_nn(i));
  //     inputs_file_nn << ",";
  //     inputs_file_nn << std::to_string(inputs_nn(i+1));
  //     inputs_file_nn << ",";
  //   }
  //   count_idx = count_idx +1;
  // }

  //
  // save_map_file << std::to_string(x(0));
  // save_map_file << ",";
  // save_map_file << std::to_string(x(1));
  // save_map_file << "\n";


  //inputs 21
  // Eigen::VectorXd inputs_nn (CONTROL_HORIZON + 1);
  // for(int i = 0; i < CONTROL_HORIZON;  i = i + 1){
  //   inputs_nn(i) = ref_y_robot_nn(i);
  // }
  // inputs_nn(CONTROL_HORIZON) = ref_theta_robot_nn;

  // inputs 3
  // Eigen::VectorXd inputs_nn (3);
  // inputs_nn(0) = ref_x_robot_nn(0);
  // inputs_nn(1) = ref_y_robot_nn(0);
  // inputs_nn(2) = x(2) - ref_theta_robot_nn;

  // inputs 2
  Eigen::VectorXd inputs_nn (2);
  inputs_nn(0) = ref_x_robot_nn(0);
  inputs_nn(1) = ref_y_robot_nn(0);
  // inputs_nn(2) = x(2) - ref_theta_robot_nn;

  // std::cout<<"inputs_nn(0) = "<<inputs_nn(0)<<std::endl;
  // std::cout<<"inputs_nn(1) = "<<inputs_nn(1)<<std::endl;
  // std::cout<<"inputs_nn(2) = "<<inputs_nn(2)<<std::endl;

  u(0) =result_mpc.at(0);

  if (no_of_neurons == 0){
    u_nn(0) = 0.0;
  }
  else{
    // double output = neural_controller.Solve_NN(inputs_nn);
    // double slope = (2.0 + 2.0) / (0.836332 + 0.836332);
    // u_nn(0) = -2.0 + slope * (output + 0.836332);
    // std::cout <<"inputs_nn= "<<inputs_nn<<std::endl;
    clock_t begin_time_nn = std::clock();
    u_nn(0) = neural_controller.Solve_NN(inputs_nn);
    clock_t end_time_nn = std::clock();
    // std::cout <<"nn time = " << float( end_time_nn - begin_time_nn ) / CLOCKS_PER_SEC <<std::endl;
    time_nn = time_nn + double( end_time_nn - begin_time_nn ) / CLOCKS_PER_SEC;
    // std::cout <<"end Solve_NN\n"<<std::endl;
    // std::cout <<"u_nn(0)= "<<u_nn(0)<<std::endl;
    // std::cout<< "mpc = "<<  u(0) << ",  output = "<< output<< ",  u_nn(0) = " << u_nn(0) << std::endl;
  }
  count_time = count_time +1;
  // std::cout <<" count_time = "<<count_time<<std::endl;
  // std::cout <<" time_nn = "<<time_nn<<std::endl;
  // std::cout <<" time_mpc = "<<time_mpc<<std::endl;

  if(INPUTS_NN_FILE == 1){
    inputs_file_nn << u_nn(0);
    inputs_file_nn << "\n";
  }
  result_cost = result_mpc.at(1);

  if(RESULTS_FILE == 1){
      for(int i = 0; i<CONTROL_HORIZON; i ++){
        dataset_file << std::to_string(ref_x_robot(i));
        dataset_file << ", ";
        dataset_file << std::to_string(ref_y_robot(i));
        dataset_file << ", ";
      }
      // for(int i = 0; i<CONTROL_HORIZON; i ++){
      //   dataset_file << std::to_string(ref_y_robot(i));
      //   dataset_file << ", ";
      // }
      dataset_file << std::to_string(ref_theta_robot_poly);
      dataset_file << ", ";
      dataset_file << std::to_string(ref_theta_world_poly);
      dataset_file << ", ";
      dataset_file << std::to_string(ref_offset_robot_poly);
      dataset_file << ", ";
      dataset_file << std::to_string(ref_offset_poly);
      dataset_file << ", ";
      dataset_file << std::to_string(u(0));
      dataset_file << "\n";
  }

  if(RESULTS_MPC_FILE == 1){
      result_file_mpc << std::to_string(x(0));
      result_file_mpc << ",";
      result_file_mpc << std::to_string(x(1));
      result_file_mpc << ",";
      result_file_mpc << std::to_string(x(2));
      result_file_mpc << ",";
      result_file_mpc << std::to_string(u(0));
      result_file_mpc << "\n";
  }

  if(RESULTS_NN_FILE == 1){
      dataset_file_nn << std::to_string(point_idx_nn);
      dataset_file_nn << ",";
      dataset_file_nn << std::to_string(x_nn(0));
      dataset_file_nn << ",";
      dataset_file_nn << std::to_string(x_nn(1));
      dataset_file_nn << ",";
      dataset_file_nn << std::to_string(x_nn(2));
      dataset_file_nn << ",";
      for(int i = 0; i<CONTROL_HORIZON; i ++){
        dataset_file_nn << std::to_string(ref_x_nn(i));
        dataset_file_nn << ", ";
        dataset_file_nn << std::to_string(ref_y_nn(i));
        dataset_file_nn << ", ";
      }
      dataset_file_nn << std::to_string(u_nn(0));
      dataset_file_nn << ",";
      dataset_file_nn << std::to_string(u(0));
      dataset_file_nn << "\n";
  }

}



void printString(const char *str, double x, double y, double size) {
   glPushMatrix();
   glTranslatef(x,y,0);
   glScalef(size/153.0,size/153.0,1.0);
   int lineCt = 0;
   int len = strlen(str);
   for (int i = 0; i < len; i++) {
      if (str[i] == '\n') {
         lineCt++;
         glPopMatrix();
         glPushMatrix();
         glTranslatef(x,y-size*1.15*lineCt,0);
         glScalef(size/153.0,size/153.0,1.0);
      }
      else {
         glutStrokeCharacter(GLUT_STROKE_ROMAN,str[i]);
         //glutStrokeCharacter(GLUT_BITMAP_TIMES_ROMAN_24,str[i]);
      }
   }
   glPopMatrix();
}

void timer(int id) {
    if(pause_flag == 0){
        time_step_counter++;
    }
    glutPostRedisplay();
}


void drawTank() {
    double time = time_step_counter*DT + ti;
    double size_vector = result_x.size();
    double size_vector_nn = result_x_nn.size();
    // Tank
    Eigen::MatrixXd M(2, 15);
    M.row(0) << 1/5.0,-1/5.0,0,0,-1/5.0,-1/5.0,0,0,-1/5.0,1/5.0,0,0,3/5.0,3/5.0,0;
    M.row(1) << -2/5.0,-2/5.0,-2/5.0,-1/5.0,-1/5.0,1/5.0,1/5.0,2/5.0,2/5.0,2/5.0,2/5.0,1/5.0,0.5/5.0,-0.5/5.0,-1/5.0;
    M = move_motif(M,result_x.at(size_vector - 1),result_y.at(size_vector - 1),result_psi.at(size_vector - 1));


    Eigen::MatrixXd M_nn(2, 15);
    M_nn.row(0) << 1/5.0,-1/5.0,0,0,-1/5.0,-1/5.0,0,0,-1/5.0,1/5.0,0,0,3/5.0,3/5.0,0;
    M_nn.row(1) << -2/5.0,-2/5.0,-2/5.0,-1/5.0,-1/5.0,1/5.0,1/5.0,2/5.0,2/5.0,2/5.0,2/5.0,1/5.0,0.5/5.0,-0.5/5.0,-1/5.0;
    M_nn = move_motif(M_nn,result_x_nn.at(size_vector_nn - 1),result_y_nn.at(size_vector_nn - 1),result_psi_nn.at(size_vector_nn - 1));


    glColor3f(1.0, 0.0, 1.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i < 15; i ++){
      glVertex3d(M(0,i),  M(1,i),  0.0);
    }
    glEnd();

    glColor3f(1.0, 0.0, 1.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i <= size_vector - 1; i ++)
      glVertex3d(result_x.at(i),  result_y.at(i),  0.0);
    glEnd();


    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i < 15; i ++){
      glVertex3d(M_nn(0,i),  M_nn(1,i),  0.0);
    }
    glEnd();


    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i <= size_vector - 1; i ++)
      glVertex3d(result_x_nn.at(i),  result_y_nn.at(i),  0.0);
    glEnd();

    // drawCircleInput(0.5,0.5,0.2,15,1);

    // Time string
    char *mystr = (char*)malloc(100*sizeof(char));
    sprintf(mystr,"Time %4.2f : %4.2f : %4.2f , Cost : %4.2f\n",
     ti,time,tf,result_cost);


    if(trans_corr > -1){
      printString(mystr, 2+ (trans_corr*35), 5, 0.5);
    }
    else{
      printString(mystr, 2, 8, 0.5);
    }


    free(mystr);
}

void drawCircleInput(float cx,float cy,float r,float num_segments, int count){
  glBegin(GL_LINE_LOOP);
  for(int ii = 0; ii < num_segments; ii++)
  {
      float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);//get the current angle

      float x = r * cosf(theta);//calculate the x component
      float y = r * sinf(theta);//calculate the y component

      glVertex2f(x + cx, y + cy);//output vertex

  }
  glEnd();

  if (count < 40){
    drawCircleInput( cx, cy - 0.8, r, num_segments, count + 1);
  }
}
void drawRobotReference() {
    // Reference line

    double size_vector_ref = ref_x_vector_w.size();
    double size_vector_ref_nn = ref_x_vector_nn.size();
    double size_vector_result = result_x_nn.size();

    // glLineWidth(10.0);
    // glColor3f(255,255,255);
    // glBegin(GL_LINE_STRIP);
    //
    //
    // for (int i=0; i < CONTROL_HORIZON; i ++){
    //     glVertex2f(ref_x_robot(i), ref_y_robot(i));
    // }
    // glEnd();

    glLineWidth(10.0);
    glColor3f(255,20,147);
    glBegin(GL_LINE_STRIP);


    for (int i=0; i < CONTROL_HORIZON; i ++){
        glVertex2f(ref_x(i), ref_y(i));
    }
    glEnd();

    glLineWidth(10.0);
    glColor3f(0,0,255);
    glBegin(GL_LINE_STRIP);


    for (int i=0; i < CONTROL_HORIZON; i ++){
        glVertex2f(ref_x_nn(i), ref_y_nn(i));
    }
    glEnd();

    // glLineWidth(2.5);
    // glColor3f(1.0, 1.0, 0.0);
    // glBegin(GL_LINE_STRIP);
    // glVertex2f(result_x_nn.at(size_vector_result - 1), result_y_nn.at(size_vector_result - 1));
    // glVertex2f(result_x_nn.at(size_vector_result - 1) + 1.0*cos(result_psi_nn.at(size_vector_result - 1)), result_y_nn.at(size_vector_result - 1) + 1.0*sin(result_psi_nn.at(size_vector_result - 1)));
    //
    // glEnd();
    //
    //
    // glLineWidth(2.5);
    // glColor3f(1.0, 1.0, 0.0);
    // glBegin(GL_LINE_STRIP);
    // glVertex2f(result_x_nn.at(size_vector_result - 1), result_y_nn.at(size_vector_result - 1));
    // glVertex2f(result_x_nn.at(size_vector_result - 1) + 1.0*cos(result_psi_nn.at(size_vector_result - 1) + 1.5708), result_y_nn.at(size_vector_result - 1) + 1.0*sin(result_psi_nn.at(size_vector_result - 1)+ 1.5708));
    //
    // glEnd();


    // glLineWidth(2.5);
    // glColor3f(0.0, 1.0, 0.0);
    // glBegin(GL_LINE_STRIP);
    // glVertex2f(result_x.at(size_vector_result - 1), result_y.at(size_vector_result - 1));
    // glVertex2f(result_x.at(size_vector_result - 1) + 1.0*cos(result_psi.at(size_vector_result - 1)), result_y.at(size_vector_result - 1) + 1.0*sin(result_psi.at(size_vector_result - 1)));
    //
    // glEnd();
    //
    //
    // glLineWidth(2.5);
    // glColor3f(0.0, 1.0, 0.0);
    // glBegin(GL_LINE_STRIP);
    // glVertex2f(result_x.at(size_vector_result - 1), result_y.at(size_vector_result - 1));
    // glVertex2f(result_x.at(size_vector_result - 1) + 1.0*cos(result_psi.at(size_vector_result - 1) + 1.5708), result_y.at(size_vector_result - 1) + 1.0*sin(result_psi.at(size_vector_result - 1)+ 1.5708));
    //
    // glEnd();


}

void drawReference() {
    // Reference line
    // glLineWidth(0.5);
    // glColor3f(1.0, 0.0, 0.0);
    // glBegin(GL_LINES);
    // for(int i = 0; i < ref_x_vector_w.size(); i++){
    //   glVertex2f(ref_x_vector_w.at(i),ref_y_vector_w.at(i));
    // }
    //
    // glEnd();

    glLineWidth(5.0);
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_POINTS);
    glVertex2f(-0.234908,-3.9336);
    glVertex2f(-0.177147,-3.949818);
    glVertex2f(-0.205924,-3.94134);
    glEnd();

    glLineWidth(0.5);
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);
    for(int i = 0; i < in_contour_x.size(); i++){
      glVertex2f(in_contour_x.at(i),in_contour_y.at(i));
    }

    glEnd();


    glLineWidth(0.5);
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);
    for(int i = 0; i < out_contour_x.size(); i++){
      glVertex2f(out_contour_x.at(i),out_contour_y.at(i));
    }

    glEnd();
}

// void drawWall(Eigen::MatrixXd wall,int size,Eigen::VectorXd x) {
//     double x_wall;
//     double y_wall;
//     glLineWidth(0.5);
//     glColor3f(1.0, 0.0, 0.0);
//     glBegin(GL_LINES);
//
//     for(int i = 0; i <size; i++){
//       x_wall = ((wall(i,1)+ 0.275)*std::cos(x(2)) + (wall(i,0))*std::sin(x(2))) + x(0);
//       y_wall = -((wall(i,0))*std::cos(x(2)) - (wall(i,1)+ 0.265)*std::sin(x(2))) + x(1);
//       std::cout<<"wall(i,0),wall(i,1) = "<<x_wall<<","<<y_wall<<std::endl;
//       glVertex2f(x_wall,y_wall );
//     }
//
//     glEnd();
// }


void display() {
    srand (time(NULL));
    double x0 ;
    double x1 ;
    double x2 ;
    double deltax;
    double time = time_step_counter*DT + ti;
    // std::cout <<"time = "<<time<<std::endl;
    double size_vector;
    std::string file_name;
    if(time >= tf){
      inputs_file_nn.close();
      if(RESULTS_NN_FILE == 1){
        if(next_neuron == 100){
          glutDestroyWindow(1);
          if(RESULTS_NN_FILE == 1)
            dataset_file_nn.close();
        }

        else{
          dataset_file_nn.close();
          file_name = "/home/maria/Documents/nn_mpc/src/keras_result_files/18_10/result_race_";//3inp_1hiddlay_10neurons_dataset2_activationf0.txt";

          // if (next_neuron == 100){
          //   if (next_inputs == 3){
          //     next_inputs = 21;
          //   }
          //   else{//next_inputs == 21
          //     next_inputs = 40;
          //   }
          // }
          //
          // else{
            next_neuron = next_neuron + 10;
            next_inputs = next_inputs;
          // }

          file_name = file_name + std::to_string(next_inputs) + "inp_2hiddlay_"+std::to_string(next_neuron)+"neurons_dataset2_activationf0.txt";
          time = 0.0;
          time_step_counter = 0.0;

          x0 = 0.0;
          x1 = 0.0;
          x2 = 0.0;
          freq = 1.0;
          path = "race";
          std::cout << "restart x0 =  " << x0 << ", x1 =  " << x1 << "  , x2 = " <<x2 << ", freq = "<< freq << ", path =  " << path << std::endl;

          glClearColor(1.0f, 1.0f, 1.0f, 1.0f );
          glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
          glInitMPC(x0,x1, x2,path,next_neuron,freq,file_name);
        }
      }

      else if(RESULTS_FILE == 1){
        if(y_idx > 20.0){
         glutDestroyWindow(1);
         if(RESULTS_FILE == 1)
           results_file.close();
        }

       else{
         time = 0.0;
         time_step_counter = 0.0;

         if(theta_idx == 2){
           theta_idx = 0;
           y_idx = y_idx + 1.0;
           if(freq == 1.0)
             freq = 0.5;
           else
             freq = 1.0;
         }
         else{
           if(freq == 1.0)
             freq = 0.5;
           else
             freq = 1.0;
           theta_idx = theta_idx + 1;
         }

         x0 = 0.0;
         x1 = y_idx;
         x2 = theta_idx*1.5708;

         std::cout << "restart x0 =  " << x0 << ", x1 =  " << x1 << "  , x2 = " <<x2 << ", freq = "<< freq << std::endl;

         path = "circle";
         glClearColor(1.0f, 1.0f, 1.0f, 1.0f );
         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
         glInitMPC(x0,x1, x2,path,no_of_neurons,freq,file_name);
       }
      }

      else{
        glutDestroyWindow(1);
      }

      return;
    }

    //Run MPC
    CalculateMPC();
    size_vector  = result_x.size();

    //Initialize window parameters
    glutTimerFunc(ms_per_frame,timer,1);
    glClearColor (1.0,1.0,1.0,1.0);
    glClear (GL_COLOR_BUFFER_BIT);
    glLoadIdentity();

    if(trans_corr == -1){
      trans_lastx = result_x.at(size_vector - 1);
      trans_corr = 0;
    }
    else{
        deltax = result_x.at(size_vector - 1) - trans_lastx;
        if (deltax >= 35 ){
            trans_corr ++ ;
            trans_lastx = result_x.at(size_vector - 1);
        }
    }

    glTranslatef(-15 - (trans_corr*35),10,-35);
    glColor3f(1,1,1);


    //Visualize MPC results
    drawTank();
    drawRobotReference();
    drawReference();
    glutSwapBuffers();
    // getchar();


}

void reshape (int w, int h) {
    glViewport (0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective (60, (GLfloat)w / (GLfloat)h, 0.1, 100.0);
    glMatrixMode (GL_MODELVIEW);
}


/* opengl options */
void initgLOptions() {
   glEnable(GL_BLEND);
   glEnable(GL_LINE_SMOOTH);
   glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
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
