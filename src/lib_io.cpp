#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "lib/Matplotlib/matplotlibcpp.h"
#include "lib/io/lib_io.h"
#include "lib/mpc/lib_mpc.h"
#include <stdio.h>
#include <sstream>
#include "lib/nn/lib_nn.h"
#include "lib/traj/lib_trajectory.h"

double time_mpc = 0.0;
double time_nn = 0.0;

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
Eigen::VectorXd ref_x_robot_nn(CONTROL_HORIZON); //vector with reference x positions
Eigen::VectorXd ref_y_robot_nn(CONTROL_HORIZON); //vector with reference y positions

NN_CONTROLLER neural_controller;

std::string trained_nn_file;

std::string path;

Race race_track("../berlin_pts.txt","../out_contour_berlin.txt","../in_contour_berlin.txt",CONTROL_HORIZON);
Circle circle_mpc(0.0,0.0,0.0,CONTROL_HORIZON,DT);
Circle circle_nn(0.0,0.0,0.0,CONTROL_HORIZON,DT);
Sinus sinus_mpc(CONTROL_HORIZON,DT,0.5);
Sinus sinus_nn(CONTROL_HORIZON,DT,0.5);
Line line_mpc(CONTROL_HORIZON,DT,0.0);
Line line_nn(CONTROL_HORIZON,DT,0.0);

void glInitMPC(double x0,double y0, double theta0, std::string  path_choice,double freq){
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
  path = path_choice;
  sinus_mpc.change_f(freq);
  sinus_nn.change_f(freq);

  //load nn controller
  trained_nn_file = "../trained_nn/training_final_cost_40inp_3hiddlay_30neurons_dataset2_activationf2";
  neural_controller.initialize_nn(trained_nn_file);

  //useful parameters
  time_step_counter = 0;
  trans_corr = -1;

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

  std::vector<double> result_mpc;
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

  // inputs 40
  int count_idx = 0;
  Eigen::VectorXd inputs_nn (2*CONTROL_HORIZON);
  for(int i = 0; i < (2*CONTROL_HORIZON)-1;  i = i +2){
    inputs_nn(i) = ref_x_robot_nn(count_idx);
    inputs_nn(i+1) = ref_y_robot_nn(count_idx);
    count_idx = count_idx +1;
  }


  u =result_mpc.at(0);

  clock_t begin_time_nn = std::clock();
  u_nn = neural_controller.Solve_NN(inputs_nn);
  clock_t end_time_nn = std::clock();
  time_nn = time_nn + double( end_time_nn - begin_time_nn ) / CLOCKS_PER_SEC;

  x = Propagate(x,u);
  x_nn = Propagate(x_nn,u_nn);

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
     ti,time,tf);


    if(trans_corr > -1){
      printString(mystr, 2+ (trans_corr*35), 5, 0.5);
    }
    else{
      printString(mystr, 2, 8, 0.5);
    }


    free(mystr);
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

void drawRobotReference() {
    // Reference line

    double size_vector_ref = ref_x_vector_w.size();
    double size_vector_ref_nn = ref_x_vector_nn.size();
    double size_vector_result = result_x_nn.size();


    glLineWidth(10.0);
    glColor3f(0,255,0);
    glBegin(GL_LINE_STRIP);

    int begin = 0;
    if(ref_x_vector_w.size()>CONTROL_HORIZON)
      begin = ref_x_vector_w.size()-CONTROL_HORIZON;

    for (int i=begin; i < CONTROL_HORIZON + begin; i ++){
        glVertex2f(ref_x_vector_w.at(i), ref_y_vector_w.at(i));
    }
    glEnd();

    glLineWidth(10.0);
    glColor3f(0,0,255);
    glBegin(GL_LINE_STRIP);


    for (int i=begin; i < CONTROL_HORIZON + begin; i ++){
        glVertex2f(ref_x_vector_nn.at(i), ref_y_vector_nn.at(i));
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
    if(path == "race"){
      glBegin(GL_LINES);
      for(int i = 0; i < race_track.in_contour_x.size(); i++){
        glVertex2f(race_track.in_contour_x.at(i),race_track.in_contour_y.at(i));
      }

      glEnd();

      glBegin(GL_LINES);
      for(int i = 0; i < race_track.out_contour_x.size(); i++){
        glVertex2f(race_track.out_contour_x.at(i),race_track.out_contour_y.at(i));
      }

      glEnd();
    }

}

void display() {
    srand (time(NULL));
    double time = time_step_counter*DT + ti;
    double size_vector;
    double deltax;

    if(time >= tf){
      glutDestroyWindow(1);
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
