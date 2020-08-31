from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import gym
import gym_f110
import numpy as np
import tensorflow.compat.v1  as tf
from ddpg_agent import DDPG_Agent
import time
import math
from utils import  Memory

from memory import ReplayBuffer

BUFFER_SIZE = 1000000
RANDOM_SEED = 25
LR_CYCLE = 100
# Number of episodes to be run
MAX_EPISODES = 2000


MINIBATCH_SIZE = 128

# Learning rates
MAX_ACTOR_LEARNING_RATE = 1e-3
MIN_ACTOR_LEARNING_RATE = 1e-4

MAX_CRITIC_LEARNING_RATE = 5e-4
MIN_CRITIC_LEARNING_RATE = 5e-4

# Max number of steps in each episode
MAX_EP_STEPS = 130


# Memory warmup
MEMORY_WARMUP = 1024

# Discount factor reflects the agents preference for short-term rewards over long-term rewards
GAMMA = 0.9

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
        self.last_action =0.0

        #mpc
        self.result_x = []
        self.result_y = []
        self.result_theta = []
        self.trans_corr = -1
        self.trans_lastx = 0
        self.ref_x_vector_w = []
        self.dt = self.racecar_env.dt
        self.X[3] = 3.0


    def iterate(self):
        glViewport(0, 0, 2000,1000)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(-100, 100, -50, 50, 0.0, 1.0)
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
        glLineWidth(0.5)
        glColor3f(1.0, 0.0, 3.0) # Set the color to pink
        x=x.flatten()
        M = r*np.array([[1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0], [-2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1]])
        M=self.move_motif(M,x[0],x[1],x[2])
        glBegin(GL_LINE_STRIP)
        for i in range(15):
            glVertex2f(M[0,i],  M[1,i])
        glEnd()



    def display(self):
        time_i = self.time_step_counter*self.dt + self.ti
        if(time_i >= self.tf):
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
        # self.run_rl()
        self.draw_tank(np.array([[self.X[0]],[self.X[1]],[self.X[2]]]))
        glutSwapBuffers()


    def _calculate_trajectory(self,X):
        state_map = []
        if(self.path == 2):
            #circle
            cx = 0.0
            cy = 0.0

            circle_ray = self.angle2follow/3.14

            for i in range(self.CONTROL_HORIZON):
                bound_up_x = X[0] > (circle_ray*np.cos(self.angle2follow) + cx - 1.0)
                bound_down_x = X[0] < (circle_ray*np.cos(self.angle2follow) + cx + 1.0)
                bound_up_y =X[1] > (circle_ray*np.sin(self.angle2follow) + cy - 1.0)
                bound_down_y = X[1] < (circle_ray*np.sin(self.angle2follow) + cy + 1.0)
                if(bound_up_x and bound_down_x and bound_up_y and bound_down_y):
                    angle2follow_it = self.angle2follow + i*self.dt
                    circle_ray = angle2follow_it/3.14
                    self.ref_x[i] = circle_ray*np.cos(angle2follow_it) + cx
                    self.ref_y[i] = circle_ray*np.sin(angle2follow_it) + cy
                    self.angle2follow = angle2follow + 0.5*self.dt

                else:
                    angle2follow_it = self.angle2follow + i*self.dt
                    self.ref_x[i] = circle_ray*np.cos(angle2follow_it) + cx
                    self.ref_y[i] = circle_ray*np.sin(angle2follow_it) + cy


                self.ref_x_vector_map.append(self.ref_x[i])
                self.ref_y_vector_map.append(self.ref_y[i])
                state_map.append(self.ref_x[i])
                state_map.append(self.ref_y[i])
        elif(self.path == 1):
            #sinus
            ref_v_x = (X[3])/sqrt(1+(np.sin(self.sinus_freq*(X[0]))**2))

            for i in range(self.CONTROL_HORIZON):
                if(X[0] > self.last_x):
                    self.ref_x[i] = X[0] + ref_v_x*self.dt*i
                    self.ref_y[i] = self.line_offset + np.cos(self.sinus_freq*self.ref_x[i])
                    self.last_x = X[0]
                    ref_v_x = (X[3])/sqrt(1+(np.sin(self.sinus_freq*self.ref_x[i])**2))
                else:
                    self.ref_x[i] = X[0] + ref_v_x*self.dt*i
                    self.ref_y[i] = self.line_offset + np.cos(self.sinus_freq*self.ref_x[i])
                    ref_v_x = (X[3])/sqrt(1+(np.sin(self.sinus_freq*self.ref_x[i])**2))

                self.ref_x_vector_map.append(self.ref_x[i])
                self.ref_y_vector_map.append(self.ref_y[i])
                state_map.append(self.ref_x[i])
                state_map.append(self.ref_y[i])

        elif(self.path == 0):
            #line
            for i in range(self.CONTROL_HORIZON):
                if(X[0]+0.5 > self.last_x):
                    self.ref_x[i] = X[0] + 0.5 + X[3]*i*self.dt
                    self.ref_y[i] = self.line_offset
                    self.last_x = X[0] + 0.5

                else:
                    self.ref_x[i] = X[0] + 0.5 + X[3]*i*self.dt
                    self.ref_y[i] = self.line_offset

                self.ref_x_vector_map.append(self.ref_x[i])
                self.ref_y_vector_map.append(self.ref_y[i])
                state_map.append(self.ref_x[i])
                state_map.append(self.ref_y[i])

        return state_map

    def _find_state(self,X):
        state_map = self._calculate_trajectory(X)
        state = state_map
        #change frame of reference points from map to the robot frame
        for i in range(0,self.CONTROL_HORIZON*2,2):
            state[i+1] = (state_map[i+1] - X[1])*math.cos(X[2]) - (state_map[i]  - X[0])*math.sin(X[2])
            state[i] =  (state_map[i]  - X[0])*math.cos(X[2]) + (state_map[i+1] - X[1])*math.sin(X[2])

        state =  np.reshape(state, (1,self.agent.actor.s_dim, 1))
        state_map =  np.reshape(state_map, (1,self.agent.actor.s_dim, 1))
        return state,state_map

    def _update_sim_state(self):
        self.X[0] = self.obs['poses_x'][0]
        self.X[1] = self.obs['poses_y'][0]
        self.X[2] = self.obs['poses_theta'][0]
        self.X[3] = self.obs['linear_vels'][0]


    def run_rl(self):
        self.sess.run(tf.global_variables_initializer())
        # Initialize target network weights
        self.agent.actor.update_target_network()
        self.agent.critic.update_target_network()

        total_time = time.time()
        steps = 0

        self.agent.actor.learning_rate = MAX_ACTOR_LEARNING_RATE
        self.agent.critic.learning_rate = MAX_CRITIC_LEARNING_RATE

        # Initialize replay buffER
        replay_buffer = ReplayBuffer(BUFFER_SIZE, RANDOM_SEED)

        try:
            save_interval = 0

            do_training = True

            while do_training:
                # update learning rates using cosine annealing
                # T_cur = i % LR_CYCLE
                # self.agent.actor.learning_rate = MIN_ACTOR_LEARNING_RATE +\
                #               0.5 * (MAX_ACTOR_LEARNING_RATE - MIN_ACTOR_LEARNING_RATE) * \
                #               (1 + np.cos(np.pi * T_cur / LR_CYCLE))
                #
                # self.agent.critic.learning_rate = MIN_CRITIC_LEARNING_RATE +\
                #               0.5 * (MAX_CRITIC_LEARNING_RATE - MIN_CRITIC_LEARNING_RATE) * \
                #               (1 + np.cos(np.pi * T_cur / LR_CYCLE))

                episode_time = time.time()

                initial_state = {'x':[np.random.uniform(1,4)], 'y': [0.0], 'theta': [np.random.uniform(0.4,1)]}
                self.obs, _, self.done  = self.racecar_env.reset(initial_state)
                ep_ave_max_q = 0
                done = False
                cumulated_reward = 0

                X = self.X

                next_X = X


                state,state_map = self._find_state(X)

                next_state = state

                episode_steps = 0


                while not done:
                    steps+=1
                    episode_steps+=1

                    a = self.agent.act(state)

                    action_speed = 3.0

                    action = {'ego_idx': 0, 'speed': [action_speed], 'steer': [a]}


                    self.obs, reward, done = self.racecar_env.step(action,state_map,self.last_action)
                    # c = readchar.readchar()
                    self._update_sim_state()

                    self.last_action = a

                    next_X[0] = self.X[0]
                    next_X[1] = self.X[1]
                    next_X[2] = self.X[2]
                    next_X[2] = self.X[3]
                    self.display()
                    next_state,state_map = self._find_state(next_X)

                    # memory.append(action,state[0], next_state[0], reward, done)
                    replay_buffer.add(np.reshape(state, (self.agent.actor.s_dim, 1)), np.reshape(a, (self.agent.actor.a_dim,)), reward,
                                  done, np.reshape(next_state, (self.agent.actor.s_dim, 1)))
                    X = next_X

                    state = next_state

                    cumulated_reward += reward



                    # Keep adding experience to the memory until
                    # there are at least minibatch size samples
                    save_interval+=1

                    if save_interval >= self.sample_batch_size:
                        save_interval = 0
                        s_batch, a_batch, r_batch, d_batch, s2_batch = replay_buffer.sample_batch(MINIBATCH_SIZE)

                        # Find target estimate to use for updating the Q-function

                        #  Predict_traget function determines Q-value of next state
                        target_q = self.agent.critic.predict_target(s2_batch, self.agent.actor.predict_target(s2_batch))

                        # Complete target estimate (R(t+1) + Q(s(t+1),a(t+1)))
                        y_i = []
                        for k in xrange(MINIBATCH_SIZE):
                            if d_batch[k]:
                                y_i.append(r_batch[k])
                            else:
                                y_i.append(r_batch[k] + GAMMA * target_q[k][0])

                        # Perform gradient descent to update critic

                        predicted_q_value, _ = self.agent.critic.train(s_batch, a_batch, np.reshape(y_i, (MINIBATCH_SIZE, 1)))



                        ep_ave_max_q += np.amax(predicted_q_value, axis = 0)

                        # Perform "Learning" phase by moving policy parameters in direction of deterministic policy gradient
                        a_outs = self.agent.actor.predict(s_batch)
                        grads = self.agent.critic.action_gradients(s_batch, a_outs)
                        self.agent.actor.train(s_batch, grads[0])

                        # Update target networks
                        self.agent.actor.update_target_network()
                        self.agent.critic.update_target_network()
                    # if save_interval >= self.sample_batch_size:
                    #     save_interval = 0
                    #     replay_time = time.time()
                    #     self.agent.replay(memory)
                    #     rospy.loginfo("Replay time {}".format(time.time()-replay_time))
                    #     if steps >= self.sample_batch_size*200:
                    #         do_training = False
                    #     memory = Memory()


                if self.highest_reward < cumulated_reward:
                    self.highest_reward = cumulated_reward

                if self.agent.exploration_rate > self.agent.exploration_min:
                    self.agent.exploration_rate *= self.agent.exploration_decay

                rospy.loginfo("total episode_steps {}, reward {}/{}".format(episode_steps, cumulated_reward, self.highest_reward))
                rospy.loginfo("Episode time {}, total {}".format(self.format_time(episode_time),
                                                                self.format_time(total_time)))
                rospy.loginfo("exploration_rate {}".format(self.agent.exploration_rate))
                rospy.loginfo("steps: {} / self.sample_batch_size*200 : {} ".format(steps,self.sample_batch_size*200))


        finally:
            self.racecar_env.close()

    def propagate(self,u):
      self.X[0] = self.X[0] + self.X[3]*math.cos(self.X[2])*self.dt;
      self.X[1] = self.X[1] + self.X[3]*math.sin(self.X[2])*self.dt;
      self.X[2] = self.X[2] + self.X[3]*math.sin(u)/self.racecar_env.l_f*self.dt;
      self.X[3] = self.X[3]

    def CalculateMPC(self):
        self.result_x.append(self.X[0])
        self.result_y.append(self.X[1])
        self.result_theta.append(self.X[2])

        state,state_map = self._find_state(self.X)
        u = 0.0
        self.propagate(u)

    def drawRobotReference(self):
        # Reference line in the robot reference frame
        glLineWidth(10.0)
        glColor3f(255,255,255)
        glBegin(GL_LINE_STRIP)

        for i in range(self.CONTROL_HORIZON):
            glVertex2f(self.ref_x_robot[i], self.ref_y_robot[i])

        glEnd()

        glLineWidth(10.0)
        glColor3f(255,20,147)
        glBegin(GL_LINE_STRIP)


        for i in range(self.CONTROL_HORIZON):
            glVertex2f(self.ref_x[i], self.ref_y[i])

        glEnd()



    def drawReference(self):
        # Reference line
        glLineWidth(0.5)
        glColor3f(1.0, 0.0, 0.0)
        glBegin(GL_LINE_STRIP)
        for   i in range(len(self.ref_x_vector_map)):
            glVertex2f(self.ref_x_vector_map[i],self.ref_y_vector_map[i])

        glEnd()

    def run_mpc(self):
        time_i = self.time_step_counter*self.dt + self.ti
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) # Remove everything from screen (i.e. displays all white)
        glLoadIdentity() # Reset all graphic/shape's position
        self.iterate()
        self.CalculateMPC()
        size_vector  = len(self.result_x)
        # self.run_rl()
        if(self.trans_corr == -1):
            self.trans_lastx = self.result_x[size_vector - 1]
            self.trans_corr = 0
        else:
            deltax = self.result_x[size_vector - 1] - self.trans_lastx
            if (deltax >= 35 ):
                self.trans_corr  = self.trans_corr  + 1
                self.trans_lastx = self.result_x[size_vector - 1]
        glTranslatef(-15 - (self.trans_corr*35),5,0)
        print("self.X = ", self.X)
        self.draw_tank(np.array([[self.X[0]],[self.X[1]],[self.X[2]]]))
        self.drawReference()
        self.drawRobotReference()



        glutSwapBuffers()


    # //Visualize MPC results
    # drawTank();
    # drawRobotReference();
    # drawReference();
    # glutSwapBuffers();
    # // getchar();
    def reshape_mpc (self):
        width = glutGet(GLUT_WINDOW_WIDTH)
        height = glutGet(GLUT_WINDOW_HEIGHT)
        glViewport (0, 0, width, height)
        glMatrixMode (GL_PROJECTION)
        glLoadIdentity ()
        gluPerspective (60, width / height, 0.1, 100.0)
        # glOrtho(0.0, width, height, 0.0, 0.0, 1.0)
        glMatrixMode (GL_MODELVIEW)
        glLoadIdentity()

        # glViewport(0, 0, 500,500)
        # glMatrixMode(GL_PROJECTION)
        # glLoadIdentity()
        # glOrtho(-30.0, 10, -30.0, 10.0, 0.0, 1.0)
        # glMatrixMode (GL_MODELVIEW)
        # glLoadIdentity()
    def test_run(self):
        print("nothing here")
    def run(self):
        x = np.zeros(5)
        self.glInitEnv(x)
        glutInit()
        glutInitDisplayMode(GLUT_RGBA) # Set the display mode to be colored
        glutInitWindowSize(2000, 1000)   # Set the w and h of your window
        glutInitWindowPosition(0, 0)   # Set the position at which this windows should appear
        wind = glutCreateWindow("F110Env Simulation") # Set a window title
        glutDisplayFunc(self.run_mpc)
        glutIdleFunc(self.run_mpc) # Keeps the window open
        # glutReshapeFunc (self.reshape_mpc)
        glutMainLoop()  # Keeps the above created window displaying/running in a loop


if __name__ == '__main__':
    with tf.Session() as sess:
        f110_rl = F110_RL(sess)
        f110_rl.run()
