from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import gym
import gym_f110
import numpy as np
import tensorflow.compat.v1  as tf

class F110_RL(object):
    def __init__(self,sess):
        self.time_step_counter = 0.0
        self.trans_corr = -1
        self.last_x = 0
        self.ti = 0.0
        self.tf = 5.0
        self.dt = 0.01

    def iterate(self):
        glViewport(0, 0, 500,500)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0.0, 500, 0.0, 500, 0.0, 1.0)
        glMatrixMode (GL_MODELVIEW)
        glLoadIdentity()

    def timer(self,id):
        glutPostRedisplay()


    def glInitEnv(self,x):
        time_step_counter = 0
        trans_corr = -1
        last_x = x[0]

    def move_motif(self,M,x,y,theta):
        M1=np.ones((1,len(M[1,:])))
        M2=np.vstack((M, M1))
        R = np.array([[np.cos(theta),-np.sin(theta),x], [np.sin(theta),np.cos(theta),y]])
        return(R @ M2)

    def draw_tank(self,x,r=1):
        glColor3f(1.0, 0.0, 3.0) # Set the color to pink
        x=x.flatten()
        M = r*np.array([[1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0], [-2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1]])
        M=self.move_motif(M,x[0],x[1],x[2])
        glBegin(GL_LINE_STRIP)
        for i in range(15):
            glVertex2f(M[0,i],  M[1,i])
        glEnd()



    def display(self):
        time = self.time_step_counter*self.dt + self.ti
        print("time = ", time)
        if(time >= self.tf):
            if(do_training):
                x = np.zeros(5)
                self.time_step_counter = 0
                glClearColor(0.0, 0.0, 0.0, 1.0 )
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
                glInitMPC(x)
            else:
                glutDestroyWindow(1)

        #do yout thing here, call mpc, reinforcement learning

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) # Remove everything from screen (i.e. displays all white)
        glLoadIdentity() # Reset all graphic/shape's position
        self.iterate()
        self.draw_tank(np.array([[100],[100],[1]]))
        glutSwapBuffers()

    def run(self):
        x = np.zeros(5)
        self.glInitEnv(x)
        glutInit()
        glutInitDisplayMode(GLUT_RGBA) # Set the display mode to be colored
        glutInitWindowSize(500, 500)   # Set the w and h of your window
        glutInitWindowPosition(0, 0)   # Set the position at which this windows should appear
        wind = glutCreateWindow("OpenGL Coding Practice") # Set a window title
        glutDisplayFunc(self.display)
        glutIdleFunc(self.display) # Keeps the window open
        glutMainLoop()  # Keeps the above created window displaying/running in a loop


if __name__ == '__main__':
    with tf.Session() as sess:
        f110_rl = F110_RL(sess)
        f110_rl.run()
