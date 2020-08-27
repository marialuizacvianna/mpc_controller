from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import gym
import gym_f110
import numpy as np

time_step_counter = 0.0
trans_corr = -1
last_x = 0
ti = 0.0
tf = 5.0
dt = 0.01


ms_per_frame = 5
w, h = 500,500

# ---Section 1---
def square():
    # We have to declare the points in this sequence: bottom left, bottom right, top right, top left
    glBegin(GL_QUADS) # Begin the sketch
    glVertex2f(100, 100) # Coordinates for the bottom left point
    glVertex2f(200, 100) # Coordinates for the bottom right point
    glVertex2f(200, 200) # Coordinates for the top right point
    glVertex2f(100, 200) # Coordinates for the top left point
    glEnd() # Mark the end of drawing


def iterate():
    glViewport(0, 0, 500,500)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glOrtho(0.0, 500, 0.0, 500, 0.0, 1.0)
    glMatrixMode (GL_MODELVIEW)
    glLoadIdentity()

# ---Section 2---

def showScreen():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) # Remove everything from screen (i.e. displays all white)
    glLoadIdentity() # Reset all graphic/shape's position
    iterate()
    glColor3f(1.0, 0.0, 3.0) # Set the color to pink
    square() # Draw a square using our function
    glutSwapBuffers()

def timer(id):
    glutPostRedisplay()


def glInitEnv(x):
    time_step_counter = 0
    trans_corr = -1
    last_x = x[0]

def move_motif(M,x,y,θ):
    M1=np.ones((1,len(M[1,:])))
    M2=np.vstack((M, M1))
    R = np.array([[np.cos(θ),-np.sin(θ),x], [np.sin(θ),np.cos(θ),y]])
    return(R @ M2)

def draw_tank(x,r=1):
    glColor3f(1.0, 0.0, 3.0) # Set the color to pink
    x=x.flatten()
    M = r*np.array([[1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0], [-2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1]])
    M=move_motif(M,x[0],x[1],x[2])
    glBegin(GL_LINE_STRIP)
    for i in range(15):
        glVertex2f(M[0,i],  M[1,i])
    glEnd()



def display():
    # time = time_step_counter*dt + ti
    # print("time = ", time)
    # if(time >= tf):
    #     if(do_training):
    #         x = np.zeros(5)
    #         time_step_counter = 0
    #         glClearColor(0.0, 0.0, 0.0, 1.0 )
    #         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    #         glInitMPC(x)
    #     else:
    #         glutDestroyWindow(1)

    #do yout thing here, call mpc, reinforcement learning
    # glutTimerFunc(ms_per_frame,timer,1)
    # glClearColor (0.0,0.0,0.0,1.0)
    # glClear (GL_COLOR_BUFFER_BIT)
    # glLoadIdentity()

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) # Remove everything from screen (i.e. displays all white)
    glLoadIdentity() # Reset all graphic/shape's position
    iterate()
    draw_tank(np.array([[100],[100],[1]]))
    glutSwapBuffers()




x = np.zeros(5)
glInitEnv(x)
glutInit()
glutInitDisplayMode(GLUT_RGBA) # Set the display mode to be colored
glutInitWindowSize(500, 500)   # Set the w and h of your window
glutInitWindowPosition(0, 0)   # Set the position at which this windows should appear
wind = glutCreateWindow("OpenGL Coding Practice") # Set a window title
glutDisplayFunc(display)
glutIdleFunc(display) # Keeps the window open
glutMainLoop()  # Keeps the above created window displaying/running in a loop
