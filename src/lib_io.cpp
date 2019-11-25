#include "lib/Matplotlib/matplotlibcpp.h"
#include "lib/io/lib_io.h"
#include "lib/mpc/lib_mpc.h"
#include <stdio.h>
#include <fstream>

// Create an output filestream object
std::ofstream results_file("/home/maria/Documents/data_MPC/result01.csv");

namespace plt = matplotlibcpp;

double ti = 0.0, tf = T;
int time_step_counter = 0, pause_flag = 0;
int ms_per_frame = 5;
double trans_lastx;
double trans_corr = -1;

std::vector<double> result_x;
std::vector<double> result_y;
std::vector<double> result_psi;

double result_cost;

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

  // Send column names to the stream
  results_file << "ref_theta";
  results_file << ",";
  results_file << "ref_offset";
  results_file << ",";
  results_file << "acc_out";
  results_file << ",";
  results_file << "steer_out";
  results_file << "\n";
}

void calculate_MPC(){
  result_x.push_back(x(0));
  result_y.push_back(x(1));
  result_psi.push_back(x(2));


  if(PATH == "circle"){
    //circle
    double cx = 5.0;
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
        ref_x(i) = x(0) + 0.5 + i*DT;
        ref_y(i) = LINE + sin(ref_x(i));
        last_x = x(0) + 0.5;
      }
      else{
        ref_x(i) = last_x + i*DT;
        ref_y(i) = LINE + sin(ref_x(i));
      }

      ref_x_vector_w.push_back(ref_x(i));
      ref_y_vector_w.push_back(ref_y(i));
    }
  }

  else if(PATH == "line"){
    for(int i = 0; i < CONTROL_HORIZON; i++){
      if(x(0) > last_x){
        ref_x(i) = x(0) + i*DT;
        ref_y(i) = LINE;
        last_x = x(0);
      }
      else{
        ref_x(i) = last_x + i*DT;
        ref_y(i) = LINE;
      }

      ref_x_vector_w.push_back(ref_x(i));
      ref_y_vector_w.push_back(ref_y(i));
    }
  }


  Eigen::VectorXd poly_coeffs = Polyfit(ref_x, ref_y,2);

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

  u(0) = result_mpc.at(0);//steer
  u(1) = result_mpc.at(1); //acc

  result_cost = result_mpc.at(2);


  results_file << ref_theta_robot;
  results_file << ",";
  results_file << poly_coeffs_robot(0);
  results_file << ",";
  results_file << u(1);
  results_file << ",";
  results_file << u(0);
  results_file << "\n";
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
  *file << x(0) << "," << x(1) << "," << x(2) << "," << x(3) << "," << x(4) << "," << x(5) << "," << u(0) << "," << u(1) << "\n";
}
