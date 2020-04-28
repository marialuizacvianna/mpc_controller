#include "lib/Matplotlib/matplotlibcpp.h"
#include "lib/io/lib_io.h"
#include "lib/mpc/lib_mpc.h"
#include <stdio.h>
#include <fstream>
#include <sstream>
#include "lib/nn/lib_nn.h"

// Create an output filestream object
std::ofstream results_file;
std::ofstream dataset_file;
std::ofstream dataset_file_nn;

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

double last_x,last_y,last_x_nn,angle2follow,angle2follow_nn;

std::string trained_nn_file;

int no_of_neurons;

std::string path;

double y_idx = 40.0;
double theta_idx = 0.0;
double freq = 1.0;
int next_neuron = 5;
int configuration = 1;


void glInitMPC(double x0,double x1, double x2, std::string path_option,int neurons,double freq_choice, std::string file_name){
  no_of_neurons = neurons;
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

  if (no_of_neurons == 0 ){
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/3 neurons/training_minimum_cost/1903_1019/trained_nn_3";
  }

  else if (no_of_neurons == 3 ){
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/3 neurons/training_minimum_cost/0104_1700/trained_nn_3";//line
  }

  else if (no_of_neurons == 5 ){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/5 neurons/training_minimum_cost/0904_1200/trained_nn_5";//line
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/5 neurons/training_minimum_cost/2704_1414/trained_nn_5";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/5 neurons/training_minimum_cost/2704_2244/trained_nn_5";//line,sinus and spiral
  }

  else if (no_of_neurons == 10){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/10 neurons/training_minimum_cost/0904_1200/trained_nn_10";//line
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/10 neurons/training_minimum_cost/2704_1414/trained_nn_10";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/10 neurons/training_minimum_cost/2704_2244/trained_nn_10";//line,sinus and spiral
  }

  else if (no_of_neurons == 15){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/15 neurons/training_minimum_cost/0904_1200/trained_nn_15";//line
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/15 neurons/training_minimum_cost/2704_1414/trained_nn_15";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/15 neurons/training_minimum_cost/2704_2244/trained_nn_15";//line,sinus and spiral
  }

  else if (no_of_neurons == 20){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/20 neurons/training_minimum_cost/1504_1500/trained_nn_20";//line
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/20 neurons/training_minimum_cost/2704_1414/trained_nn_20";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/20 neurons/training_minimum_cost/2704_2244/trained_nn_20";//line,sinus and spiral
  }

  else if (no_of_neurons == 25){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/25 neurons/training_minimum_cost/1504_1500/trained_nn_25";//line
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/25 neurons/training_minimum_cost/2704_1414/trained_nn_25";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/25 neurons/training_minimum_cost/2704_2244/trained_nn_25";//line,sinus and spiral
  }

  else if (no_of_neurons == 30){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/30 neurons/training_minimum_cost/1504_1500/trained_nn_30";//line
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/30 neurons/training_minimum_cost/2704_1414/trained_nn_30";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/30 neurons/training_minimum_cost/2704_2244/trained_nn_30";//line,sinus and spiral
  }

  else if (no_of_neurons == 35){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/35 neurons/training_minimum_cost/1504_1500/trained_nn_35";//line
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/35 neurons/training_minimum_cost/2704_1414/trained_nn_35";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/35 neurons/training_minimum_cost/2704_2244/trained_nn_35";//line,sinus and spiral
  }

  else if (no_of_neurons == 40){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/40 neurons/training_minimum_cost/1504_1500/trained_nn_40";//line
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/40 neurons/training_minimum_cost/2704_1414/trained_nn_40";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/40 neurons/training_minimum_cost/2704_2244/trained_nn_40";//line,sinus and spiral
  }

  else if (no_of_neurons == 45){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/45 neurons/training_minimum_cost/1504_1500/trained_nn_45";//line
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/45 neurons/training_minimum_cost/2704_1414/trained_nn_45";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/45 neurons/training_minimum_cost/2704_2244/trained_nn_45";//line,sinus and spiral
  }

  else if (no_of_neurons == 50){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/50 neurons/training_minimum_cost/1504_1500/trained_nn_50";//line
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/2704_1414/trained_nn_50";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/2704_2244/trained_nn_50";//line,sinus and spiral
  }

  else if (no_of_neurons == 55){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/55 neurons/training_minimum_cost/1504_1500/trained_nn_55";//line
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/55 neurons/training_minimum_cost/2704_1414/trained_nn_55";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/55 neurons/training_minimum_cost/2704_2244/trained_nn_55";//line,sinus and spiral
  }

  else if (no_of_neurons == 60){
    // trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/60 neurons/training_minimum_cost/2304_0930/trained_nn_60";//line
    trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/60 neurons/training_minimum_cost/2704_1414/trained_nn_60";//line and sinus
    // trained_nn_file = "/home/maria/Documents/nn_mpc/trained_nn/60 neurons/training_minimum_cost/2704_2244/trained_nn_60";//line,sinus and spiral
  }

  else if (no_of_neurons == 65){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/65 neurons/training_minimum_cost/2304_0930/trained_nn_65";//line
  }

  else if (no_of_neurons == 70){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/70 neurons/training_minimum_cost/2304_0930/trained_nn_70";//line
  }


  else if (no_of_neurons == 75){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/75 neurons/training_minimum_cost/2304_0930/trained_nn_75";
  }

  else if (no_of_neurons == 80){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/80 neurons/training_minimum_cost/2304_0930/trained_nn_80";
  }

  else if (no_of_neurons == 85){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/85 neurons/training_minimum_cost/2304_0930/trained_nn_85";
  }

  else if (no_of_neurons == 90){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/90 neurons/training_minimum_cost/2304_0930/trained_nn_90";
  }

  else if (no_of_neurons == 95){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/95 neurons/training_minimum_cost/2304_0930/trained_nn_95";
  }

  else if (no_of_neurons == 100){
    trained_nn_file = "/home/maria/Documents/Neural Network MPC/nn_mpc/trained_nn/100 neurons/training_minimum_cost/2304_0930/trained_nn_100";
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

  time_step_counter = 0;
  pause_flag = 0;
  trans_corr = -1;

  for(int i = 0; i < SIZE_INPUT; i++){
    u(i) = 0.0;
    u_nn(i) = 0.0;
  }

  last_x = x0;
  last_x_nn = x0;

  angle2follow = 0.0;
  angle2follow_nn = 0.0;
  // Send column names to the stream
  if(RESULTS_FILE == 1 && !dataset_file.is_open()){
    dataset_file.open("/home/maria/Documents/MPC/mpc_controller/dataset/dataset_03_spiral.txt", std::ios_base::app);
  }

  if(RESULTS_NN_FILE == 1){
    dataset_file_nn.open(file_name, std::ios_base::app);
  }

}
void CalculateTrajectory(){
  if(path == "quad"){
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
      if(x(0) > (circle_ray*cos(angle2follow) + cx - 1.0) && x(0) < (circle_ray*cos(angle2follow) + cx + 1.0)
        && x(1) > (circle_ray*sin(angle2follow) + cy - 1.0) && x(1) < (circle_ray*sin(angle2follow) + cy + 1.0)){
        double angle2follow_it = angle2follow + i*DT;
        circle_ray = angle2follow_it/3.14;
        ref_x(i) = circle_ray*cos(angle2follow_it) + cx;
        ref_y(i) = circle_ray*sin(angle2follow_it) + cy;
        angle2follow = angle2follow + 0.5*DT;
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
      if(x_nn(0) > (circle_ray_nn*cos(angle2follow_nn) + cx - 1.0) && x_nn(0) < (circle_ray_nn*cos(angle2follow_nn) + cx + 1.0)
        && x_nn(1) > (circle_ray_nn*sin(angle2follow_nn) + cy - 1.0) && x_nn(1) < (circle_ray_nn*sin(angle2follow_nn) + cy + 1.0)){
        double angle2follow_it = angle2follow_nn + i*DT;
        circle_ray_nn = angle2follow_it/3.14;
        ref_x_nn(i) = circle_ray_nn*cos(angle2follow_it) + cx;
        ref_y_nn(i) = circle_ray_nn*sin(angle2follow_it) + cy;
        angle2follow_nn = angle2follow_nn + 0.5*DT;
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

      if(freq == 2.0){
        const_aux = x(3)*cos(x(2))*DT;
        const_nn = x_nn(3)*cos(x_nn(2))*DT;
      }
      else{
        const_aux = 0.5;
        const_nn = 0.5;
      }

      for(int i = 0; i < CONTROL_HORIZON; i++){
        if( x(0) + const_aux > last_x){
          ref_x(i) = x(0) +const_aux ;
          ref_y(i) = LINE + std::cos(freq*ref_x(i));
          last_x = x(0) + const_aux  ;
        }
        else{
          ref_x(i) = last_x + i*DT;
          ref_y(i) = LINE + cos(freq*ref_x(i));
        }

        ref_x_vector_w.push_back(ref_x(i));
        ref_y_vector_w.push_back(ref_y(i));
      }

    for(int i = 0; i < CONTROL_HORIZON; i++){
      if( x_nn(0) + const_nn > last_x_nn){
        ref_x_nn(i) = x_nn(0)  + const_nn  + i*DT;
        ref_y_nn(i) = LINE + cos(freq*ref_x_nn(i));
        last_x_nn = x_nn(0) + const_nn ;
      }
      else{
        ref_x_nn(i) = last_x_nn + i*DT;
        ref_y_nn(i) = LINE + cos(freq*ref_x_nn(i));
      }

      ref_x_vector_nn.push_back(ref_x_nn(i));
      ref_y_vector_nn.push_back(ref_y_nn(i));
    }
    //
    // ref_x = ref_x_nn;
    // ref_y = ref_y_nn;
    // ref_x_nn = ref_x;
    // ref_y_nn = ref_y;
  }

  else if(path == "line"){
    for(int i = 0; i < CONTROL_HORIZON; i++){
      if(x(0) + 0.5> last_x){
        ref_x(i) = x(0) + 0.5 + i*DT;
        ref_y(i) = LINE;
        last_x = x(0)+ 0.5;
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
        ref_x_nn(i) = x_nn(0) + 0.5 + i*DT;
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
  NN_CONTROLLER neural_controller(trained_nn_file);
  result_x.push_back(x(0));
  result_y.push_back(x(1));
  result_x_nn.push_back(x_nn(0));
  result_y_nn.push_back(x_nn(1));
  result_psi.push_back(x(2));
  result_psi_nn.push_back(x_nn(2));

  CalculateTrajectory();

  for(int i = 0; i < CONTROL_HORIZON; i++){
    ref_y_robot(i) = (ref_y(i) - x(1))*cos(x(2)) - (ref_x(i) - x(0))*sin(x(2));
    ref_x_robot(i) = (ref_x(i) - x(0))*cos(x(2)) + (ref_y(i) - x(1))*sin(x(2));
  }

  for(int i = 0; i < CONTROL_HORIZON; i++){
    ref_y_robot_nn(i) = (ref_y_nn(i) - x_nn(1))*cos(x_nn(2)) - (ref_x_nn(i) - x_nn(0))*sin(x_nn(2));
    ref_x_robot_nn(i) = (ref_x_nn(i) - x_nn(0))*cos(x_nn(2)) + (ref_y_nn(i) - x_nn(1))*sin(x_nn(2));
  }

  Eigen::VectorXd poly_coeffs_robot = Polyfit(ref_x_robot, ref_y_robot,3);
  double ref_offset_robot_poly = poly_coeffs_robot(0);
  double ref_theta_robot_poly = -atan(poly_coeffs_robot(1));


  Eigen::VectorXd poly_coeffs_robot_nn = Polyfit(ref_x_robot_nn, ref_y_robot_nn,3);
  double ref_offset_robot_nn = poly_coeffs_robot_nn(0);
  double ref_theta_robot_nn = -atan(poly_coeffs_robot_nn(1));

  Eigen::VectorXd poly_coeffs = Polyfit(ref_x, ref_y,3);
  double ref_offset_poly = poly_coeffs(0);
  double ref_theta_world_poly = -atan(poly_coeffs(1));

  Eigen::VectorXd poly_coeffs_nn = Polyfit(ref_x_nn, ref_y_nn,3);
  double ref_offset_nn = poly_coeffs_nn(0);
  double ref_theta_nn = -atan(poly_coeffs_nn(1));

  x = Propagate(x,u);
  if (no_of_neurons > 0){
    x_nn = Propagate(x_nn,u_nn);
  }

  std::vector<double> result_mpc;
  result_mpc = Solve_MPC(x,ref_x,ref_y,0.0,0.0);

  //inputs 40
  int count_idx = 0;
  Eigen::VectorXd inputs_nn (40);
  for(int i = 0; i < 40-1;  i = i +2){
    inputs_nn(i) = ref_x_robot_nn(count_idx);
    inputs_nn(i+1) = ref_y_robot_nn(count_idx);
    count_idx = count_idx +1;
  }
  //inputs 21
  // Eigen::VectorXd inputs_nn (21);
  // for(int i = 0; i < 20;  i = i + 1){
  //   inputs_nn(i) = ref_y_robot_nn(i);
  // }
  // inputs_nn(20) = ref_theta_robot_nn;

  u(0) = result_mpc.at(0);

  if (no_of_neurons == 0){
    u_nn(0) = 0.0;
  }
  else{
    // double output = neural_controller.Solve_NN(inputs_nn);
    // double slope = (2.0 + 2.0) / (0.836332 + 0.836332);
    // u_nn(0) = -2.0 + slope * (output + 0.836332);
    u_nn(0) = neural_controller.Solve_NN(inputs_nn);
    // std::cout<< "mpc = "<<  u(0) << ",  output = "<< output<< ",  u_nn(0) = " << u_nn(0) << std::endl;
  }

  result_cost = result_mpc.at(1);

  if(RESULTS_FILE == 1){
      for(int i = 0; i<CONTROL_HORIZON; i ++){
        dataset_file << std::to_string(ref_x_robot(i));
        dataset_file << ", ";
        dataset_file << std::to_string(ref_y_robot(i));
        dataset_file << ", ";
      }
      for(int i = 0; i<CONTROL_HORIZON; i ++){
        dataset_file << std::to_string(ref_y_robot(i));
        dataset_file << ", ";
      }
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

  if(RESULTS_NN_FILE == 1){
      dataset_file_nn << std::to_string(ref_x_nn(0));
      dataset_file_nn << ",";
      dataset_file_nn << std::to_string(ref_y_nn(0));
      dataset_file_nn << ",";
      dataset_file_nn << std::to_string(x_nn(0));
      dataset_file_nn << ",";
      dataset_file_nn << std::to_string(x_nn(1));
      dataset_file_nn << ",";
      dataset_file_nn << std::to_string(ref_x(0));
      dataset_file_nn << ",";
      dataset_file_nn << std::to_string(ref_y(0));
      dataset_file_nn << ",";
      dataset_file_nn << std::to_string(x(0));
      dataset_file_nn << ",";
      dataset_file_nn << std::to_string(x(1));
      dataset_file_nn << ",";
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


    glColor3f(0.0, 1.0, 0.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i <= size_vector - 1; i ++)
      glVertex3d(result_x.at(i),  result_y.at(i),  0.0);
    glEnd();


    glColor3f(1.0, 1.0, 0.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i <= size_vector - 1; i ++)
      glVertex3d(result_x_nn.at(i),  result_y_nn.at(i),  0.0);
    glEnd();

    // Time string
    char *mystr = (char*)malloc(100*sizeof(char));
    sprintf(mystr,"Time %4.2f : %4.2f : %4.2f , Cost : %4.2f\n",
     ti,time,tf,result_cost);


    if(trans_corr > -1){
      printString(mystr, 2+ (trans_corr*35), 8, 0.5);
    }
    else{
      printString(mystr, 2, 8, 0.5);
    }


    free(mystr);
}

void drawRobotReference() {
    // Reference line

    double size_vector_ref = ref_x_vector_w.size();
    double size_vector_ref_nn = ref_x_vector_nn.size();
    double size_vector_result = result_x_nn.size();

    glLineWidth(10.0);
    glColor3f(255,255,255);
    glBegin(GL_LINE_STRIP);


    for (int i=0; i < CONTROL_HORIZON; i ++){
        glVertex2f(ref_x_robot(i), ref_y_robot(i));
    }
    glEnd();

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
        glVertex2f(ref_x_vector_nn.at((size_vector_ref_nn - CONTROL_HORIZON) + i), ref_y_vector_nn.at((size_vector_ref_nn - CONTROL_HORIZON) + i));
    }
    glEnd();

    glLineWidth(2.5);
    glColor3f(1.0, 1.0, 0.0);
    glBegin(GL_LINE_STRIP);
    glVertex2f(result_x_nn.at(size_vector_result - 1), result_y_nn.at(size_vector_result - 1));
    glVertex2f(result_x_nn.at(size_vector_result - 1) + 1.0*cos(result_psi_nn.at(size_vector_result - 1)), result_y_nn.at(size_vector_result - 1) + 1.0*sin(result_psi_nn.at(size_vector_result - 1)));

    glEnd();


    glLineWidth(2.5);
    glColor3f(1.0, 1.0, 0.0);
    glBegin(GL_LINE_STRIP);
    glVertex2f(result_x_nn.at(size_vector_result - 1), result_y_nn.at(size_vector_result - 1));
    glVertex2f(result_x_nn.at(size_vector_result - 1) + 1.0*cos(result_psi_nn.at(size_vector_result - 1) + 1.5708), result_y_nn.at(size_vector_result - 1) + 1.0*sin(result_psi_nn.at(size_vector_result - 1)+ 1.5708));

    glEnd();


    glLineWidth(2.5);
    glColor3f(0.0, 1.0, 0.0);
    glBegin(GL_LINE_STRIP);
    glVertex2f(result_x.at(size_vector_result - 1), result_y.at(size_vector_result - 1));
    glVertex2f(result_x.at(size_vector_result - 1) + 1.0*cos(result_psi.at(size_vector_result - 1)), result_y.at(size_vector_result - 1) + 1.0*sin(result_psi.at(size_vector_result - 1)));

    glEnd();


    glLineWidth(2.5);
    glColor3f(0.0, 1.0, 0.0);
    glBegin(GL_LINE_STRIP);
    glVertex2f(result_x.at(size_vector_result - 1), result_y.at(size_vector_result - 1));
    glVertex2f(result_x.at(size_vector_result - 1) + 1.0*cos(result_psi.at(size_vector_result - 1) + 1.5708), result_y.at(size_vector_result - 1) + 1.0*sin(result_psi.at(size_vector_result - 1)+ 1.5708));

    glEnd();


}

void drawReference() {
    // Reference line
    glLineWidth(0.5);
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i < ref_x_vector_w.size(); i++){
      glVertex2f(ref_x_vector_w.at(i),ref_y_vector_w.at(i));
    }

    glEnd();
}


void display() {
    srand (time(NULL));
    double x0 ;
    double x1 ;
    double x2 ;
    double deltax;
    double time = time_step_counter*DT + ti;
    double size_vector;
    std::string file_name;
    if(time >= tf){
      if(RESULTS_NN_FILE == 1){
        if(next_neuron ==  60 && configuration == 4){
          glutDestroyWindow(1);
          if(RESULTS_NN_FILE == 1)
            dataset_file_nn.close();
        }

        else{
          dataset_file_nn.close();
          file_name = "/home/maria/Documents/nn_mpc/trained_nn/"; //55 neurons/training_minimum_cost/1504_1500/result_circle.txt";

          if (configuration == 4){
            next_neuron = next_neuron +  5;
            configuration = 1;
          }

          else{
            configuration = configuration + 1;
          }

          file_name = file_name + std::to_string(next_neuron) + " neurons/training_minimum_cost/2704_1414/result_";
          time = 0.0;
          time_step_counter = 0.0;

          x0 = 0.0;

          if(configuration == 1){
            x1 = 3.0;
            x2 = 0.5;
            path = "line";
            file_name = file_name + "line.txt";
          }

          if(configuration == 2){
            x1 = 3.0;
            x2 = 2.7;
            freq = 0.5;
            path = "sinus";
            file_name = file_name + "sinus.txt";
          }

          if(configuration == 3){
            x1 = 3.0;
            x2 = 0.0;
            freq = 1.0;
            path = "sinus";
            file_name = file_name + "sinus_2.txt";
          }

          if(configuration == 4){
            x1 = 0.0;
            x2 = 0.0;
            path = "circle";
            file_name = file_name + "circle.txt";
          }

          std::cout << "restart x0 =  " << x0 << ", x1 =  " << x1 << "  , x2 = " <<x2 << ", freq = "<< freq << ", path =  " << path << std::endl;

          glClearColor(0.0f, 0.0f, 0.0f, 1.0f );
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

         glClearColor(0.0f, 0.0f, 0.0f, 1.0f );
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
    glClearColor (0.0,0.0,0.0,1.0);
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

    glTranslatef(-15 - (trans_corr*35),0,-20);
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
