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
#include "lib/traj/lib_trajectory.h"

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

std::vector<double> ref_x_vector_w;
std::vector<double> ref_y_vector_w;
std::vector<double> ref_x_vector_nn;
std::vector<double> ref_y_vector_nn;


Eigen::VectorXd x(SIZE_STATE);//x y psi v err_offset err_psi
Eigen::VectorXd x_nn(SIZE_STATE);//x y psi v err_offset err_psi
double u;
double u_nn;
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
int count_step = 0;
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
int public_choice_net = 0;
int plus_idx_nn = 0;

Race race_track("../berlin_pts.txt","../out_contour_berlin.txt","../in_contour_berlin.txt",CONTROL_HORIZON);
Circle circle_mpc(0.0,0.0,0.0,CONTROL_HORIZON,DT);
Circle circle_nn(0.0,0.0,0.0,CONTROL_HORIZON,DT);
Sinus sinus_mpc(CONTROL_HORIZON,DT,0.5);
Sinus sinus_nn(CONTROL_HORIZON,DT,0.5);
Line line_mpc(CONTROL_HORIZON,DT,0.0);
Line line_nn(CONTROL_HORIZON,DT,0.0);

void glInitMPC(double x0,double y0, double theta0){
  //initiate MPC robot
  x(0) = x0; //x
  x(1) = y0; //y
  x(2) = theta0; //theta
  x(3) = V_CTE; //speed
  //initiate NN robot
  x_nn(0) = x0; //x
  x_nn(1) = y0; //y
  x_nn(2) = theta0; //theta
  x_nn(3) = V_CTE; //speed

  u =  0.0; //initialize mpc control command
  u_nn = 0.0; //initialize nn control command

  //init trajectories
  path = "race";
  // Line line_track();
  // Race race_track("../berlin_pts.txt","../out_contour_berlin.txt","../in_contour_berlin.txt");
  //init race_trajectory

  //load nn controller
  trained_nn_file = "/home/maria/Documents/nn_mpc/src/keras_trained_nn/30_10/training_final_cost_40inp_3hiddlay_30neurons_dataset2_activationf2";
  neural_controller.initialize_nn(trained_nn_file);

  //useful parameters
  time_step_counter = 0;
  trans_corr = -1;


  // if(RESULTS_FILE == 1 && !dataset_file.is_open()){
  //   dataset_file.open("/home/maria/Documents/MPC/mpc_controller/dataset/dataset_04_lines_sinus_spiral.txt", std::ios_base::app);
  // }
  //
  // if (RESULTS_MPC_FILE== 1){
  //   result_file_mpc.open("/home/maria/Documents/MPC/mpc_controller/spiral_mpc.txt");
  // }
  //
  // if(RESULTS_NN_FILE == 1){
  //   std::cout <<"inside\n";
  //   dataset_file_nn.open(file_name, std::ios_base::app);
  // }
  //
  // if(INPUTS_NN_FILE == 1){
  //   inputs_file_nn.open("../inputs_nn/line_tanh.txt");
  // }

}

void CalculateTrajectory(){
  if (path == "race"){
    race_track.get_reference(x,ref_x_vector_w,ref_y_vector_w,ref_x_robot,ref_y_robot);
    race_track.get_reference(x_nn,ref_x_vector_nn,ref_y_vector_nn,ref_x_robot_nn,ref_y_robot_nn);
  }

  else if(path == "circle"){
    circle_mpc.get_reference(x,ref_x_vector_w,ref_y_vector_w,ref_x_robot,ref_y_robot);
    circle_nn.get_reference(x_nn,ref_x_vector_nn,ref_y_vector_nn,ref_x_robot_nn,ref_y_robot_nn);
  }

  else if(path == "sinus"){
    sinus_mpc.get_reference(x,ref_x_vector_w,ref_y_vector_w,ref_x_robot,ref_y_robot);
    sinus_nn.get_reference(x_nn,ref_x_vector_nn,ref_y_vector_nn,ref_x_robot_nn,ref_y_robot_nn);
  }

  else if(path == "line"){
    line_mpc.get_reference(x,ref_x_vector_w,ref_y_vector_w,ref_x_robot,ref_y_robot);
    line_nn.get_reference(x_nn,ref_x_vector_nn,ref_y_vector_nn,ref_x_robot_nn,ref_y_robot_nn);
  }

}

void CalculateMPC(){
  //keep track of visited positions
  result_x.push_back(x(0));
  result_y.push_back(x(1));
  result_psi.push_back(x(2));
  result_x_nn.push_back(x_nn(0));
  result_y_nn.push_back(x_nn(1));
  result_psi_nn.push_back(x_nn(2));

  CalculateTrajectory();

  x = Propagate(x,u);
  x_nn = Propagate(x_nn,u_nn);

  std::vector<double> result_mpc;
  std::vector<double> result_mpc_nn;
  clock_t begin_time_mpc = std::clock();
  int index_iteration = 0;
  if(ref_x_vector_w.size() > CONTROL_HORIZON){
    index_iteration = ref_x_vector_w.size() - CONTROL_HORIZON;
  }
  for(int i = 0; i < CONTROL_HORIZON ; i++){
    ref_x[i] = ref_x_vector_w.at(index_iteration + i);
    ref_y[i] = ref_y_vector_w.at(index_iteration + i);
  }
  result_mpc = Solve_MPC(x,ref_x,ref_y);
  clock_t end_time_mpc = std::clock();
  time_mpc = time_mpc + double( end_time_mpc - begin_time_mpc )/ CLOCKS_PER_SEC;
  result_mpc_nn = Solve_MPC(x_nn,ref_x_nn,ref_y_nn);


  // inputs 40
  int count_idx = 0;
  Eigen::VectorXd inputs_nn (2*CONTROL_HORIZON);
  for(int i = 0; i < (2*CONTROL_HORIZON)-1;  i = i +2){
    inputs_nn(i) = ref_x_robot_nn(count_idx);
    inputs_nn(i+1) = ref_y_robot_nn(count_idx);
    // if(INPUTS_NN_FILE == 1){
    //   inputs_file_nn << std::to_string(inputs_nn(i));
    //   inputs_file_nn << ",";
    //   inputs_file_nn << std::to_string(inputs_nn(i+1));
    //   inputs_file_nn << ",";
    // }
    count_idx = count_idx +1;
  }


  u =result_mpc.at(0);

  clock_t begin_time_nn = std::clock();
  u_nn = neural_controller.Solve_NN(inputs_nn);
  clock_t end_time_nn = std::clock();
  time_nn = time_nn + double( end_time_nn - begin_time_nn ) / CLOCKS_PER_SEC;


  // if(INPUTS_NN_FILE == 1){
  //   inputs_file_nn << u_nn(0);
  //   inputs_file_nn << "\n";
  // }
  // result_cost = result_mpc.at(1);
  //
  // if(RESULTS_FILE == 1){
  //     for(int i = 0; i<CONTROL_HORIZON; i ++){
  //       dataset_file << std::to_string(ref_x_robot(i));
  //       dataset_file << ", ";
  //       dataset_file << std::to_string(ref_y_robot(i));
  //       dataset_file << ", ";
  //     }
  //     // for(int i = 0; i<CONTROL_HORIZON; i ++){
  //     //   dataset_file << std::to_string(ref_y_robot(i));
  //     //   dataset_file << ", ";
  //     // }
  //     dataset_file << std::to_string(ref_theta_robot_poly);
  //     dataset_file << ", ";
  //     dataset_file << std::to_string(ref_theta_world_poly);
  //     dataset_file << ", ";
  //     dataset_file << std::to_string(ref_offset_robot_poly);
  //     dataset_file << ", ";
  //     dataset_file << std::to_string(ref_offset_poly);
  //     dataset_file << ", ";
  //     dataset_file << std::to_string(u(0));
  //     dataset_file << "\n";
  // }
  //
  // if(RESULTS_MPC_FILE == 1){
  //     result_file_mpc << std::to_string(x(0));
  //     result_file_mpc << ",";
  //     result_file_mpc << std::to_string(x(1));
  //     result_file_mpc << ",";
  //     result_file_mpc << std::to_string(x(2));
  //     result_file_mpc << ",";
  //     result_file_mpc << std::to_string(u(0));
  //     result_file_mpc << "\n";
  // }
  //
  // if(RESULTS_NN_FILE == 1){
  //     dataset_file_nn << std::to_string(point_idx_nn);
  //     dataset_file_nn << ",";
  //     dataset_file_nn << std::to_string(x_nn(0));
  //     dataset_file_nn << ",";
  //     dataset_file_nn << std::to_string(x_nn(1));
  //     dataset_file_nn << ",";
  //     dataset_file_nn << std::to_string(x_nn(2));
  //     dataset_file_nn << ",";
  //     for(int i = 0; i<CONTROL_HORIZON; i ++){
  //       dataset_file_nn << std::to_string(ref_x_nn(i));
  //       dataset_file_nn << ", ";
  //       dataset_file_nn << std::to_string(ref_y_nn(i));
  //       dataset_file_nn << ", ";
  //     }
  //     dataset_file_nn << std::to_string(u_nn(0));
  //     dataset_file_nn << ",";
  //     dataset_file_nn << std::to_string(u(0));
  //     dataset_file_nn << "\n";
  // }

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

void drawRobotReference() {
    // Reference line

    double size_vector_ref = ref_x_vector_w.size();
    double size_vector_ref_nn = ref_x_vector_nn.size();
    double size_vector_result = result_x_nn.size();


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

}

void drawReference() {

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
      // inputs_file_nn.close();
      // if(RESULTS_NN_FILE == 1){
      //   if(next_neuron == 100){
      //     glutDestroyWindow(1);
      //     if(RESULTS_NN_FILE == 1)
      //       dataset_file_nn.close();
      //   }
      //
      //   else{
      //     dataset_file_nn.close();
      //     file_name = "/home/maria/Documents/nn_mpc/src/keras_result_files/18_10/result_race_";//3inp_1hiddlay_10neurons_dataset2_activationf0.txt";
      //
      //       next_neuron = next_neuron + 10;
      //       next_inputs = next_inputs;
      //     // }
      //
      //     file_name = file_name + std::to_string(next_inputs) + "inp_2hiddlay_"+std::to_string(next_neuron)+"neurons_dataset2_activationf0.txt";
      //     time = 0.0;
      //     time_step_counter = 0.0;
      //
      //     x0 = 0.0;
      //     x1 = 0.0;
      //     x2 = 0.0;
      //     freq = 1.0;
      //     path = "race";
      //
      //
      //     glClearColor(1.0f, 1.0f, 1.0f, 1.0f );
      //     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      //     glInitMPC(x0,x1, x2,path,next_neuron,freq,file_name,0);
      //   }
      // }

      // else if(RESULTS_FILE == 1){
      //   if(y_idx > 20.0){
      //    glutDestroyWindow(1);
      //    if(RESULTS_FILE == 1)
      //      results_file.close();
      //   }
      //
      //  else{
      //    time = 0.0;
      //    time_step_counter = 0.0;
      //
      //    if(theta_idx == 2){
      //      theta_idx = 0;
      //      y_idx = y_idx + 1.0;
      //      if(freq == 1.0)
      //        freq = 0.5;
      //      else
      //        freq = 1.0;
      //    }
      //    else{
      //      if(freq == 1.0)
      //        freq = 0.5;
      //      else
      //        freq = 1.0;
      //      theta_idx = theta_idx + 1;
      //    }
      //
      //    x0 = 0.0;
      //    x1 = y_idx;
      //    x2 = theta_idx*1.5708;
      //
      //    // std::cout << "restart x0 =  " << x0 << ", x1 =  " << x1 << "  , x2 = " <<x2 << ", freq = "<< freq << std::endl;
      //
      //    path = "circle";
      //    glClearColor(1.0f, 1.0f, 1.0f, 1.0f );
      //    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      //    glInitMPC(x0,x1, x2,path,no_of_neurons,freq,file_name,0);
      //  }
      // }
      //
      // else{
      //   glutDestroyWindow(1);
      // }

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
