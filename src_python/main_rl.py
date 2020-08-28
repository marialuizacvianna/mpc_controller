from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import gym
import gym_f110
import numpy as np
import tensorflow.compat.v1  as tf
from ddpg_agent import DDPG_Agent

class F110_RL(object):
    def __init__(self,sess):
        #rl parameters
        self.sess = sess
        self.sample_batch_size = 1000
        self.highest_reward = -np.inf
        self.last_action = 0.0

        self.racecar_env = gym.make('f110-v0')

        initial_state = {'x':[0.0], 'y': [0.0], 'theta': [0.0]}
        self.obs, _, self.done= self.racecar_env.reset(initial_state)

        self.time_step_counter = 0.0
        self.trans_corr = -1
        self.last_x = 0
        self.ti = 0.0
        self.tf = 5.0
        self.racecar_env.dt = 0.01
        self.X = np.zeros(4) #x,y,theta,v

        self.angle2follow = 0.0

        self.CONTROL_HORIZON = 20

        self.ref_x = np.zeros(self.CONTROL_HORIZON)
        self.ref_y = np.zeros(self.CONTROL_HORIZON)

        self.ref_x_robot = np.zeros(self.CONTROL_HORIZON)
        self.ref_y_robot = np.zeros(self.CONTROL_HORIZON)

        self.ref_x_vector_map = []
        self.ref_y_vector_map = []

        self.path = 0 #line default
        self.line_offset = 0 #default

        self.sinus_freq = 1.0

        #rl parameters
        self.state_size = self.CONTROL_HORIZON*2
        self.action_size = 1
        self.buffer_max_size = 100000
        self.chunk_size = 20000
        self.always_explore = False
        self.agent = DDPG_Agent(self.sess,self.state_size, self.action_size, self.buffer_max_size, self.chunk_size ,self.racecar_env.max_steering_angle, always_explore=self.always_explore)

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
        return(R.dot(M2))

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
        time = self.time_step_counter*self.racecar_env.dt + self.ti
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
        self.run_rl()
        self.draw_tank(np.array([[100],[100],[1]]))
        glutSwapBuffers()


    def calculate_trajectory(self):
        if(self.path == 2):
            #circle
            cx = 0.0
            cy = 0.0
            circle_ray = self.angle2follow/3.14

            for i in range(self.CONTROL_HORIZON):
                bound_up_x = self.X[0] > (circle_ray*np.cos(self.angle2follow) + cx - 1.0)
                bound_down_x = self.X[0] < (circle_ray*np.cos(self.angle2follow) + cx + 1.0)
                bound_up_y = self.X[1] > (circle_ray*np.sin(self.angle2follow) + cy - 1.0)
                bound_down_y = self.X[1] < (circle_ray*np.sin(self.angle2follow) + cy + 1.0)
                if(bound_up_x and bound_down_x and bound_up_y and bound_down_y):
                    angle2follow_it = self.angle2follow + i*self.racecar_env.dt
                    circle_ray = angle2follow_it/3.14
                    self.ref_x[i] = circle_ray*np.cos(angle2follow_it) + cx
                    self.ref_y[i] = circle_ray*np.sin(angle2follow_it) + cy
                    self.angle2follow = angle2follow + 0.5*DT

                else:
                    angle2follow_it = self.angle2follow + i*DT
                    self.ref_x[i] = circle_ray*np.cos(angle2follow_it) + cx
                    self.ref_y[i] = circle_ray*np.sin(angle2follow_it) + cy


                self.ref_x_vector_map.append(self.ref_x[i])
                self.ref_y_vector_map.append(self.ref_y[i])

        elif(self.path == 1):
            #sinus
            ref_v_x = (self.X[3])/sqrt(1+(np.sin(self.sinus_freq*(self.X[0]))**2))

            for i in range(self.CONTROL_HORIZON):
                if(self.X[0] > self.last_x):
                    self.ref_x[i] = self.X[0] + ref_v_x*self.racecar_env.dt*i
                    self.ref_y[i] = self.line_offset + np.cos(self.sinus_freq*self.ref_x[i])
                    self.last_x = self.X[0]
                    ref_v_x = (self.X[3])/sqrt(1+(np.sin(self.sinus_freq*self.ref_x[i])**2))
                else:
                    self.ref_x[i] = self.X[0] + ref_v_x*self.racecar_env.dt*i
                    self.ref_y[i] = self.line_offset + np.cos(self.sinus_freq*self.ref_x[i])
                    ref_v_x = (self.X[3])/sqrt(1+(np.sin(self.sinus_freq*self.ref_x[i])**2))

                self.ref_x_vector_map.append(self.ref_x[i])
                self.ref_y_vector_map.append(self.ref_y[i])


        elif(self.path == 0):
            #line
            for i in range(self.CONTROL_HORIZON):
                if(self.X[0]+0.5 > self.last_x):
                    self.ref_x[i] = self.X[0] + 0.5 + self.X[3]*i*self.racecar_env.dt
                    self.ref_y[i] = self.line_offset
                    self.last_x = self.X[0] + 0.5

                else:
                    self.ref_x[i] = self.X[0] + 0.5 + self.X[3]*i*self.racecar_env.dt
                    self.ref_y[i] = self.line_offset

                self.ref_x_vector_map.append(self.ref_x[i])
                self.ref_y_vector_map.append(self.ref_y[i])



    def run_rl(self):
        self.calculate_trajectory()

        #change frame of reference points from map to the robot frame
        for i in range(self.CONTROL_HORIZON):
            self.ref_y_robot[i] = (self.ref_y[i] - self.X[1])*np.cos(self.X[2]) - (self.ref_x[i]  - self.X[0])*np.sin(self.X[2])
            self.ref_x_robot[i] =  (self.ref_x[i]  - self.X[0])*np.cos(self.X[2]) + (self.ref_y[i] - self.X[1])*np.sin(self.X[2])


        #todo

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
