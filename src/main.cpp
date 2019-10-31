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
int time_step_counter = -1, pause_flag = 0;
int ms_per_frame = 5;

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
        if(time_step_counter >= N){
            time_step_counter = 0;
        }
    }
    glutPostRedisplay();
}

void drawPendulum(double time, double theta, double v) {
    double x1 = 0, y1 = 0;
    // Reference line
    glLineWidth(0.5);
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex2f(-100,y1);
    glVertex2f( 100,y1);
    glEnd();

    Eigen::MatrixXd M(2, 15);
    M.row(0) << 1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0;
    M.row(1) << -2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1;

    M = move_motif(M,0.0,0.0,0.0);

    glColor3f(1.0, 0.0, 1.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i < 15; i ++){
      glVertex3d(M(0,i),  M(1,i),  0.0);
    }
    glEnd();



    // The pendulum support anchor block
    // glColor3f(0.0, 0.0, 1.0);
    // glBegin(GL_QUADS);
    // glNormal3f( 0.0,  0.0,  1.0);
    // glVertex3d(-0.8,  0.0,  0.0);
    // glVertex3d( 0.8,  0.0,  0.0);
    // glVertex3d( 0.8,  0.8,  0.0);
    // glVertex3d(-0.8,  0.8,  0.0);
    // glEnd();



    // // The pendulum bob
    double pi = 3.1416;
    double x2, y2;
    // if(0 <= theta < pi/2){
    //         x2 = l*cos(pi/2-theta);
    //         y2 = -l*sin(pi/2-theta);
    // }
    // else if(pi/2 <= theta < pi){
    //         x2 = l*cos(theta - pi/2);
    //         y2 = l*sin(theta - pi/2);
    // }
    // else if(-pi/2 <= theta < 0){
    //         x2 = -l*cos(pi/2-abs(theta));
    //         y2 = -l*sin(pi/2-abs(theta));
    // }
    // else if(-pi <= theta < -pi/2){
    //         x2 = -l*cos(abs(theta) - pi/2);
    //         y2 = l*sin(abs(theta) - pi/2);
    // }
    // else{
    //     printf("invalid theta\n");
    // }

    // glColor3f(0.2, 0.65, 0.4);
    // glPushMatrix();
    // glTranslated(x2, y2, 0.0);
    // glutWireSphere(0.82, 50, 50);
    // glPopMatrix();

    // // thick line connecting the two
    // glLineWidth(200);
    // glColor3f(0.8, 0.25, 0.4);
    // glBegin(GL_LINES);
    // glVertex2f(x1,y1);
    // glVertex2f(x2,y2);
    // glEnd();

    char *mystr = (char*)malloc(100*sizeof(char));
    sprintf(mystr,"Time %4.2f : %4.2f : %4.2f\n", ti,time,tf);
    printString(mystr, -10, 5, 2);
    free(mystr);
}

void display(void) {
    glutTimerFunc(ms_per_frame,timer,1);
    glClearColor (0.0,0.0,0.0,1.0);
    glClear (GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    glTranslatef(0,0,-20);
    glColor3f(1,1,1);
    drawPendulum(0.0,0.0,0.0);
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
  //visualisation
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
    write_results(&result_file,x,u);

    //line
    for(i = 0; i < NB_POINTS_POLYFIT; i++){
      ref_y(i) = 1.0;
      ref_x(i) = x(0) + (DT*(i+1));
    }

    //sinus
    // for(i = 0; i < NB_POINTS_POLYFIT; i++){
    //   ref_x(i) = x(0) + (DT*(i+1));
    //   ref_y(i) = sin(0.1*ref_x(i));
    // }
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

  std::cout << "End simulation \n";
  result_file.close();

  return 0;
}
