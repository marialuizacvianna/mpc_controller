#include "lib/Matplotlib/matplotlibcpp.h"
#include "lib/io/lib_io.h"
#include "lib/mpc/lib_mpc.h"
#include <stdio.h>
#include <fstream>
#include <sstream>

// Create an output filestream object
std::ofstream results_file;
std::ifstream ref_file;
std::ifstream infile("/home/maria/Documents/nn_mpc/results_run");
std::ofstream dataset_file;

namespace plt = matplotlibcpp;

double ti = 0.0, tf = T;
int time_step_counter = 0, pause_flag = 0;
int ms_per_frame = 5;
double trans_lastx;
double trans_corr = -1;
int count_u = 0;
std::vector<double> steer_data;
std::vector<double> steer_out;

std::vector<double> result_x;
std::vector<double> result_y;
std::vector<double> result_psi;

double result_cost;
double count_ref_file;
std::vector<double> ref_file_vec;

std::vector<double> ref_x_vector_w;
std::vector<double> ref_y_vector_w;
std::vector<double> ref_x_vector_r;
std::vector<double> ref_y_vector_r;

Eigen::VectorXd x(SIZE_STATE);//x y psi v err_offset err_psi
Eigen::VectorXd u(SIZE_INPUT);//steering angle rate  acceleration

Eigen::VectorXd ref_x(CONTROL_HORIZON); //vector with reference x positions
Eigen::VectorXd ref_y(CONTROL_HORIZON); //vector with reference y positions

Eigen::VectorXd ref_x_robot(CONTROL_HORIZON); //vector with reference x positions
Eigen::VectorXd ref_y_robot(CONTROL_HORIZON); //vector with reference y positions

double last_x, last_ray,angle2follow;

void glInitMPC(){
  x(0) = X0; //x
  x(1) = X1; //y
  x(2) = X2; //theta
  x(3) = X3; //speed


  for(int i = 0; i < SIZE_INPUT; i++){
    u(i) = 0.0;
  }

  last_x = X0;
  last_ray = 1000;
  angle2follow = 0.0;
  count_ref_file = 0;

  if(REF_FILE){
    std::string ref_file_line;
    ref_file.open("/home/maria/Documents/MPC/mpc_controller/data/ref_files/quad_ref.txt");
    if (ref_file.is_open()){
      while(std::getline(ref_file,ref_file_line)){
        ref_file_vec.push_back(std::stod(ref_file_line));
      }
      ref_file.close();
    }
    std::cout << " REF_FILE == 1\n";
    std::cout << "size of ref_file_vec: " << ref_file_vec.size() << std::endl;
  }

  // Send column names to the stream
  if(RESULTS_FILE == 1){
    dataset_file.open("/home/maria/Documents/MPC/mpc_controller/dataset_MPC.txt", std::ios_base::app);
    // dataset_file << "x_r";
    // dataset_file << ",";
    // dataset_file << "y_r";
    // dataset_file << ",";
    // dataset_file << "t_r";
    // dataset_file << ",";
    // dataset_file << "x_d";
    // dataset_file << ",";
    // dataset_file << "y_d";
    // dataset_file << ",";
    // dataset_file << "t_d";
    // dataset_file << ",";
    // dataset_file << "steer";
    // dataset_file << ",";
    // dataset_file << "acc";
    // dataset_file << "\n";
  }

  std::string line;
  int y_parse = -1;
  int count_lines = 0;
  while (std::getline(infile, line))
  {
    if(y_parse == 0 && line != "y_data"){
      steer_out.push_back(stod(line));
    }

    if(y_parse == 1){
      steer_data.push_back(stod(line));
    }

    if(line == "y_out"){
      y_parse = 0;
    }

    if(line == "y_data"){
      y_parse = 1;
    }

  }
}

void calculate_MPC(){
  result_x.push_back(x(0));
  result_y.push_back(x(1));
  result_psi.push_back(x(2));

  if(REF_FILE){
    if(count_ref_file < ref_file_vec.size() + CONTROL_HORIZON){
      for(int i = 0; i < CONTROL_HORIZON; i++){
        if(x(0) > last_x){
          ref_x(i) = x(0) + 0.5 + i*DT;
          ref_y(i) = ref_file_vec.at(count_ref_file + i);
          last_x = x(0);
        }
        else{
          ref_x(i) = x(0) + 0.5 + i*DT;
          ref_y(i) = ref_file_vec.at(count_ref_file + i);
        }

        ref_x_vector_w.push_back(ref_x(i));
        ref_y_vector_w.push_back(ref_y(i));
      }
      count_ref_file = count_ref_file + CONTROL_HORIZON ;
    }
    else
      count_ref_file = 0;
  }
  else{
    if(PATH == "quad"){
      double aux_quad;
      double quad_coeff = 0.2;
      double aux_ref_x;
      double aux_center ;
      if(x(0) < 5.0){
        aux_ref_x = 5.0;
        aux_center = 10;
      }
      else{
        aux_center = (int)round(x(0)/10.0) * 10;
        aux_ref_x = x(0);
      }

      for(int i = 0; i < CONTROL_HORIZON; i++){
        if( x(0) >= last_x){
          ref_x(i) = aux_ref_x  + i*DT;
          aux_quad =  ref_x(i) - aux_center;
          ref_y(i) = -0.2*(aux_quad*aux_quad)  + 5;
          last_x = aux_ref_x;// + 0.5;
        }
        else{
          ref_x(i) = last_x +  i*DT;
          aux_quad =  ref_x(i) - aux_center;
          ref_y(i) = -0.2*(aux_quad*aux_quad)  + 5;
        }

        ref_x_vector_w.push_back(ref_x(i));
        ref_y_vector_w.push_back(ref_y(i));

      }
    }
    if(PATH == "circle"){
      //circle
      double cx = 0.0;
      double cy = 0.0;
      double circle_ray = 5.0;

      for(int i = 0; i < CONTROL_HORIZON; i++){

        if(x(0) > (circle_ray*cos(angle2follow) + cx - 1.0) && x(0) < (circle_ray*cos(angle2follow) + cx + 1.0)
          && x(1) > (circle_ray*sin(angle2follow) + cy - 1.0) && x(1) < (circle_ray*sin(angle2follow) + cy + 1.0)){
          double angle2follow_it = angle2follow + i*DT;
          ref_x(i) = circle_ray*cos(angle2follow_it) + cx;
          ref_y(i) = circle_ray*sin(angle2follow_it) + cy;
          angle2follow = angle2follow + 0.1*DT;
        }
        else{
          double angle2follow_it = angle2follow + i*DT;
          ref_x(i) = circle_ray*cos(angle2follow_it) + cx;
          ref_y(i) = circle_ray*sin(angle2follow_it) + cy;
        }

        ref_x_vector_w.push_back(ref_x(i));
        ref_y_vector_w.push_back(ref_y(i));
      }
    }

    else if(PATH == "sinus"){
      for(int i = 0; i < CONTROL_HORIZON; i++){
        if( x(0) + 0.5 > last_x){
          ref_x(i) = x(0)  + 0.5 + i*DT;
          ref_y(i) = LINE + cos(ref_x(i));
          last_x = x(0) + 0.5;
        }
        else{
          ref_x(i) = last_x + i*DT;
          ref_y(i) = LINE + cos(ref_x(i));
        }

        ref_x_vector_w.push_back(ref_x(i));
        ref_y_vector_w.push_back(ref_y(i));
      }
    }

    else if(PATH == "line"){
      for(int i = 0; i < CONTROL_HORIZON; i++){
        if(x(0) > last_x){
          ref_x(i) = x(0)  + 5.0 + i*DT;
          ref_y(i) = LINE;
          last_x = x(0);
        }
        else{
          ref_x(i) = last_x + 5.0 +  i*DT;
          ref_y(i) = LINE;
        }

        ref_x_vector_w.push_back(ref_x(i));
        ref_y_vector_w.push_back(ref_y(i));
      }
    }
  }



  Eigen::VectorXd poly_coeffs = Polyfit(ref_x, ref_y,3);

  double ref_theta = -atan(poly_coeffs(1));

  x = Propagate(x,u);

  for(int i = 0; i < CONTROL_HORIZON; i++){
    ref_y_robot(i) = (ref_y(i) - x(1))*cos(x(2)) - (ref_x(i) - x(0))*sin(x(2));
    ref_x_robot(i) = (ref_x(i) - x(0))*cos(x(2)) + (ref_y(i) - x(1))*sin(x(2));

    ref_x_vector_r.push_back(ref_x_robot(i));
    ref_y_vector_r.push_back(ref_y_robot(i));
  }

  Eigen::VectorXd poly_coeffs_robot = Polyfit(ref_x_robot, ref_y_robot,2);

  double ref_theta_robot = -atan(poly_coeffs_robot(1));

  std::vector<double> result_mpc;
  result_mpc = Solve_MPC(x,ref_x,ref_y,ref_theta,ref_theta_robot);

  u(0) = result_mpc.at(0);//steer_out.at(count_u);//result_mpc.at(0);//steer
  u(1) = result_mpc.at(1); //acc

  result_cost = result_mpc.at(2);

  if(RESULTS_FILE == 1){
    // results_file << x(0);
    // results_file << ",";
    // results_file << x(1);
    // results_file << ",";
    // results_file << x(2);
    // results_file << ",";
    // results_file << ref_x(0);
    // results_file << ",";
    // results_file << ref_y(0);
    // results_file << ",";
    // results_file << ref_theta;
    // results_file << ",";
    // results_file << u(0);
    // results_file << ",";
    // results_file << u(1);
    // results_file << "\n";
    dataset_file << std::to_string(x(0));
    dataset_file << ", ";
    dataset_file << std::to_string(x(1));
    dataset_file << ", ";
    dataset_file << std::to_string(x(2));
    dataset_file << ", ";
    dataset_file << std::to_string(ref_x(0));
    dataset_file << ", ";
    dataset_file << std::to_string(ref_y(0));
    dataset_file << ", ";
    dataset_file << std::to_string(ref_theta);
    dataset_file << ", ";
    dataset_file << std::to_string(u(0));
    dataset_file << ", ";
    dataset_file << std::to_string(u(1));
    dataset_file << "\n";
  }

  count_u ++;
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
    // Tank
    Eigen::MatrixXd M(2, 15);
    M.row(0) << 1/5.0,-1/5.0,0,0,-1/5.0,-1/5.0,0,0,-1/5.0,1/5.0,0,0,3/5.0,3/5.0,0;
    M.row(1) << -2/5.0,-2/5.0,-2/5.0,-1/5.0,-1/5.0,1/5.0,1/5.0,2/5.0,2/5.0,2/5.0,2/5.0,1/5.0,0.5/5.0,-0.5/5.0,-1/5.0;
    M = move_motif(M,result_x.at(size_vector - 1),result_y.at(size_vector - 1),result_psi.at(size_vector - 1));

    glColor3f(1.0, 0.0, 1.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i < 15; i ++){
      glVertex3d(M(0,i),  M(1,i),  0.0);
    }
    glEnd();

    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i <= size_vector - 1; i ++)
      glVertex3d(result_x.at(i),  result_y.at(i),  0.0);
    glEnd();

    // Time string
    char *mystr = (char*)malloc(100*sizeof(char));
    sprintf(mystr,"Time %4.2f : %4.2f : %4.2f  --  Cost = %4.2f\n",
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

    glLineWidth(2.5);
    glColor3f(1.0, 1.0, 0.0);
    glBegin(GL_LINE_STRIP);

    double size_vector_ref = ref_x_vector_w.size();

    for (int i=0; i < CONTROL_HORIZON; i ++){
      glVertex2f(ref_x_vector_r.at((size_vector_ref - CONTROL_HORIZON) + i),ref_y_vector_r.at((size_vector_ref - CONTROL_HORIZON) + i));
    }
    glEnd();

    glLineWidth(10.0);
    glColor3f(255,20,147);
    glBegin(GL_LINE_STRIP);


    for (int i=0; i < CONTROL_HORIZON; i ++){
        glVertex2f(ref_x_vector_w.at((size_vector_ref - CONTROL_HORIZON) + i), ref_y_vector_w.at((size_vector_ref - CONTROL_HORIZON) + i));
    }
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
    double deltax;
    double time = time_step_counter*DT + ti;
    double size_vector;

    if(time >= tf){
        glutDestroyWindow(1);
        if(RESULTS_FILE == 1)
          results_file.close();
        return;
    }

    //Run MPC
    calculate_MPC();
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
