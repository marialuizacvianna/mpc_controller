#include "lib/Matplotlib/matplotlibcpp.h"
#include "lib/io/lib_io.h"
#include "lib/mpc/lib_mpc.h"
#include <stdio.h>
namespace plt = matplotlibcpp;

double ti = 0.0, tf = T;
int time_step_counter = 0, pause_flag = 0;
int ms_per_frame = 5;
double trans_lastx;
double trans_corr = -1;


std::vector<double> result_x;
std::vector<double> result_y;
std::vector<double> result_psi;

std::vector<double> result_cost;

std::vector<double> ref_x_vector;
std::vector<double> ref_y_vector;

void glGetCoefficients(std::vector<double> ref_x_vector_input,std::vector<double>  ref_y_vector_input){
  ref_x_vector = ref_x_vector_input;
  ref_y_vector = ref_y_vector_input;
}

void glGetResults(std::vector<double> x, std::vector<double> y , std::vector<double> psi){
  result_x = x;
  result_y = y;
  result_psi = psi;
}

void glGetCosts(std::vector<double> costs){
  result_cost = costs;
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

    // Tank
    Eigen::MatrixXd M(2, 15);
    M.row(0) << 1/5.0,-1/5.0,0,0,-1/5.0,-1/5.0,0,0,-1/5.0,1/5.0,0,0,3/5.0,3/5.0,0;
    M.row(1) << -2/5.0,-2/5.0,-2/5.0,-1/5.0,-1/5.0,1/5.0,1/5.0,2/5.0,2/5.0,2/5.0,2/5.0,1/5.0,0.5/5.0,-0.5/5.0,-1/5.0;
    M = move_motif(M,result_x.at(time_step_counter),result_y.at(time_step_counter),result_psi.at(time_step_counter));

    glColor3f(1.0, 0.0, 1.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i < 15; i ++){
      glVertex3d(M(0,i),  M(1,i),  0.0);
    }
    glEnd();

    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i <= time_step_counter ; i ++)
      glVertex3d(result_x.at(i),  result_y.at(i),  0.0);
    glEnd();

    // Time string
    char *mystr = (char*)malloc(100*sizeof(char));
    sprintf(mystr,"Time %4.2f : %4.2f : %4.2f  --  Cost = %4.2f\n", ti,time,tf,result_cost.at(time_step_counter));

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
    Eigen::VectorXd ref_x(NB_POINTS_POLYFIT);
    Eigen::VectorXd ref_y(NB_POINTS_POLYFIT);

    glLineWidth(2.5);
    glColor3f(1.0, 1.0, 0.0);
    glBegin(GL_LINE_STRIP);

    for (int i=0; i < NB_POINTS_POLYFIT; i ++){
      ref_x(i) = ref_x_vector.at((time_step_counter*(NB_POINTS_POLYFIT)) + i);
      ref_y(i) = ref_y_vector.at((time_step_counter*(NB_POINTS_POLYFIT)) + i);
      glVertex2f(ref_x(i),ref_y(i));
    }
    glEnd();

    std::cout << "ref_x = " << ref_x << std::endl;
    std::cout << "ref_y = " << ref_y << std::endl;

    std::cout << "psi = " << result_psi.at(time_step_counter) << std::endl;

    getchar();

    // float begin_x = poly_coeffs(4);
    // float end_x = poly_coeffs(5);
    //
    // float line_begin = poly_coeffs(0) + poly_coeffs(1)*begin_x + poly_coeffs(2)*(begin_x*begin_x) + poly_coeffs(3)*(begin_x*begin_x*begin_x);
    // float line_end = poly_coeffs(0) + poly_coeffs(1)*end_x + poly_coeffs(2)*(end_x*end_x) + poly_coeffs(3)*(end_x*end_x*end_x);

}

void drawReference() {
    // Reference line
    glLineWidth(0.5);
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex2f(-1000,LINE);
    glVertex2f( 1000,LINE);
    glEnd();
}


void display() {
    double deltax;
    glutTimerFunc(ms_per_frame,timer,1);
    glClearColor (0.0,0.0,0.0,1.0);
    glClear (GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    if(trans_corr == -1){
      trans_lastx = result_x.at(time_step_counter);
      trans_corr = 0;
    }
    else{
        deltax = result_x.at(time_step_counter) - trans_lastx;
        if (deltax >= 35 ){
            trans_corr ++ ;
            trans_lastx = result_x.at(time_step_counter);
        }
    }

    glTranslatef(-15 - (trans_corr*35),0,-20);
    glColor3f(1,1,1);
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
