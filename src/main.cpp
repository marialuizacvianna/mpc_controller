/*
  Author : Maria Costa
  maria.costa@ensta-bretagne.org
  LIX - Ecole Polytechnique
  Date : 21/10/2019
*/

#include <iostream>
#include "lib/mpc/lib_mpc.h"
#include <vector>
#include <math.h>
#include "lib/io/lib_io.h"
#include <string.h>
#include <sstream>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

// pendulum parameters
int l = 10, N = 2500;
double ti = 0, tf = 20;
int time_step_counter = 0, pause_flag = 0;
int ms_per_frame = 5;
std::vector<double> result_x;
std::vector<double> result_y;
std::vector<double> result_psi;

void printString(const char *str, double x, double y, double size) {
   glPushMatrix();
   glTranslatef(x,y,0);
   glScalef(size/1000.0,size/1000.0,1.0);
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

void drawTank(double time, double theta, double v) {
    // Tank
    Eigen::MatrixXd M(2, 15);
    M.row(0) << 1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0;
    M.row(1) << -2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1;
    std::cout << "time_step_counter = " << time_step_counter << std::endl;
    M = move_motif(M,result_x.at(time_step_counter),result_y.at(time_step_counter),result_psi.at(time_step_counter));

    glColor3f(1.0, 0.0, 1.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i < 15; i ++){
      glVertex3d(M(0,i),  M(1,i),  0.0);
    }
    glEnd();

    // Time string
    char *mystr = (char*)malloc(100*sizeof(char));
    sprintf(mystr,"Time %4.2f : %4.2f : %4.2f\n", ti,time,tf);
    printString(mystr, -10, 5, 2);
    free(mystr);
}


void drawReference() {
    // Reference line
    glLineWidth(0.5);
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex2f(-100,0);
    glVertex2f( 100,0);
    glEnd();
}

void display(void) {
    glutTimerFunc(ms_per_frame,timer,1);
    glClearColor (0.0,0.0,0.0,1.0);
    glClear (GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    glTranslatef(0,0,-20);
    glColor3f(1,1,1);
    drawTank(0.0,0.0,0.0);
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

/* quit on escape key press and toggle start/stop of simulation with s key */
void keyboard(unsigned char key, int x, int y){
    switch (key) {
        case 27:
            exit(0);
            break;
        case 115:
            if(pause_flag == 0){
                pause_flag = 1;
            }
            else{
                pause_flag = 0;
            }
            break;
    }
}

/* opengl options */
void initgLOptions() {
   glEnable(GL_BLEND);
   glEnable(GL_LINE_SMOOTH);
   glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
}

int main(int argc, char *argv[]) {
  std::ofstream result_file;
  int referece_count = 0; // reference step
  Eigen::VectorXd ref_x(NB_POINTS_POLYFIT); //vector with reference x positions
  Eigen::VectorXd ref_y(NB_POINTS_POLYFIT); //vector with reference y positions
  Eigen::VectorXd x(SIZE_STATE);//x y psi v err_offset err_psi
  Eigen::VectorXd u(SIZE_INPUT);//steering angle rate  acceleration
  std::vector<double> result_mpc;
  int i = 0; //iterator
  float last_x = 0;
  float j = 0.0;//iterator
  std::chrono::time_point<std::chrono::system_clock> start;

  create_txt_file(&result_file);
  if(result_file.is_open())
    std::cout << "Created file for writing results \n";

  for(i = 0; i < SIZE_STATE; i++){
    x(i) = 0.0; //vehicle position
  }

  x(1) = 5.0; //y
  x(2) = -3.14; //theta
  x(3) = 5.0; //speed

  last_x = x(0);

  for(i = 0; i < SIZE_INPUT; i++){
    u(i) = 0.0;
  }

  std::cout << "Begin simulation \n";
  for(j = 0.0; j < T; j=j+DT){
    result_x.push_back(x(0));
    result_y.push_back(x(1));
    result_psi.push_back(x(2));
    write_results(&result_file,x,u);

    //line
    for(i = 0; i < NB_POINTS_POLYFIT; i++){
      ref_y(i) = 1.0;
      ref_x(i) = x(0) + (DT*(i+1));
    }


    referece_count += NB_POINTS_POLYFIT;

    x = Propagate(x,u);
    for(i = 0; i < NB_POINTS_POLYFIT; i++){
      ref_y(i) = (ref_y(i) - x(1))*cos(x(2)) - (ref_x(i) - x(0))*sin(x(2));
      ref_x(i) = (ref_x(i) - x(0))*cos(x(2)) + (ref_y(i) - x(1))*sin(x(2));
    }

    Eigen::VectorXd poly_coeffs = Polyfit(ref_x, ref_y, 3);
    //it  actualizes the function offset error
    x(4) = poly_coeffs(0);

    // it calculate the vehicle's orientation error
    // f(x) is the polynomial defining the reference trajectory
    // f'(x) = 3Ax^2 + 2Bx + C
    // double d_f_x = 3*poly_coeffs[3]*pow(px,2) + 2*poly_coeffs[2]*px + poly_coeffs[1];
    double d_f_x = poly_coeffs(1);
    x(5) = -atan(d_f_x);

    // solve mpc for state and reference trajectory
    // returns [steering_angle, acceleration]
    result_mpc = Solve_MPC(x,poly_coeffs);
    u(0) = result_mpc.at(0);
    u(1) = result_mpc.at(1);

  }

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DOUBLE);
  glutInitWindowSize (1200, 800);
  glutInitWindowPosition (0, 0);
  glutCreateWindow ("MPC Simulation");

  glutDisplayFunc (display);
  glutIdleFunc (display);
  glutReshapeFunc (reshape);
  glutKeyboardFunc (keyboard);

  initgLOptions();
  glutMainLoop ();

  std::cout << "End simulation \n";
  result_file.close();

  return 0;
}
