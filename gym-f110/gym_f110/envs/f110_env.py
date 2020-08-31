# MIT License

# Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


# gym imports
import gym
from gym import error, spaces, utils
from gym.utils import seeding

# zmq imports
import zmq

# others
import numpy as np
# no cv2 if python 3
# import cv2
from PIL import Image
import sys
import os
import signal
import subprocess
import math
import yaml
import csv
import numpy as np
import math

# from matplotlib.pyplot import imshow
# import matplotlib.pyplot as plt

class F110Env(gym.Env):
    """
    OpenAI gym environment for F1/10 simulator
    Use 0mq's REQ-REP pattern to communicate to the C++ simulator
    ONE env has ONE corresponding C++ instance
    Need to create env with map input, full path to map yaml file, map pgm image and yaml should be in same directory

    should be initialized with a map, a timestep, and number of agents
    """
    metadata = {'render.modes': []}

    def __init__(self):
        # simualtor params
        self.params_set = False
        # params list is [mu, h_cg, l_r, cs_f, cs_r, I_z, mass]
        self.params = []
        # TODO: add multi agent stuff, need a _add_agent function of sth
        self.num_agents = 1
        self.timestep = 0.0

        # default
        self.ego_idx = 0
        self.dt = 0.01
        # env states
        # more accurate description should be ego car state
        # might not need to keep scan
        self.x = None
        self.y = None
        self.theta = None
        self.speed = None

        self.in_collision = False
        self.collision_angle = None

        self.origin = []

        #racecar parameters
        self.max_speed = 20.0
        self.max_accel = 9.51
        self.max_decel = 13.26
        self.max_steering_vel = 3.2
        self.max_steering_angle = 0.4189

        self.wheelbase = 0.3302
        self.mass= 3.74
        self.l_r = 0.17145
        self.l_f = self.wheelbase - self.l_r
        self.time_thresh = 5.0


    def _check_done(self,time):
        """
        Check if the episode is done
        This is in terms of the ego car
        For our case, whether the car ends up close enough to the starting point
        And if accumulated time is over the timeout
        return true if done, false if not
        This assumes start is always (0, 0)

        """
        done = 0
        if(time >= self.time_thresh):
            done = 1

        return done

        # to do
        #fazer com base no tempo (?)


    def _update_state(self, obs_dict):
        """
        Update the env's states according to observations
        obs is observation dictionary
        """
        self.x = obs_dict['poses_x'][obs_dict['ego_idx']]
        self.y = obs_dict['poses_y'][obs_dict['ego_idx']]
        self.theta = obs_dict['poses_theta'][obs_dict['ego_idx']]
        self.speed = obs_dict['linear_vels'][obs_dict['ego_idx']]



    def _compute_reward(self, last_action,action, observations, done, state_map):
        if(len(state_map) == 0 ):
            return 0


        X = observations['poses_x'][observations['ego_idx']]
        Y = observations['poses_y'][observations['ego_idx']]
        theta = observations['poses_theta'][observations['ego_idx']]

        first_ref_x = state_map[0,0]
        first_ref_y = state_map[0,1]

        last_ref_x = state_map[0,len(state_map[0,:]) - 2]
        last_ref_y = state_map[0,len(state_map[0,:]) - 1]

        dy = last_ref_y - first_ref_y
        dx = last_ref_x - first_ref_x

        angle_ref = math.atan2(dy,dx)

        reward = -(last_action - action)**2

        map_dist = (first_ref_x - X)**2 + (first_ref_y - Y)**2

        reward = reward - map_dist

        # reward =  max(min(reward, 0), -10)

        return reward

    def _compute_accel(self,desired_speed):
        dif = (desired_speed - self.speed)
        accel = 0

        if (self.speed > 0):
            if (dif > 0):
                kp = 2.0 * self.max_accel / self.max_speed
                accel = kp*dif
                accel = min(max(accel, -self.max_accel), self.max_accel)
            else:
                accel = -self.max_decel

        else:
            if (dif > 0):
                accel = self.max_decel
            else:
                kp = 2.0 * self.max_accel / self.max_speed;
                accel = kp*dif
                accel = min(max(accel, -self.max_accel), self.max_accel)

        return accel

    def _propagate(self, speed, steer_angle):
        accel = self._compute_accel(speed)
        obs = {'ego_idx': self.ego_idx, 'poses_x': [], 'poses_y': [], 'poses_theta': [], 'linear_vels': []}
        x1 = self.x + self.speed *math.cos(self.theta) * self.dt
        y1 = self.y + self.speed *math.sin(self.theta) * self.dt
        theta1 = self.theta + self.speed*math.sin(steer_angle) /self.l_f* self.dt
        v1 = self.speed + accel* self.dt

        obs['poses_x'].append(x1)
        obs['poses_y'].append(y1)
        obs['poses_theta'].append(theta1)
        obs['linear_vels'].append(v1)

        return obs

    def step(self, action, state_map,last_action,time):
        # action is a list of steering angles + command velocities
        # also a ego car index
        # action should a DICT with {'ego_idx': int, 'speed':[], 'steer':[]}
        requested_vel = action['speed'][self.ego_idx]
        requested_ang = action['steer'][self.ego_idx]


        obs = self._propagate(requested_vel,requested_ang)

        # update accumulated time in env
        self.current_time = self.current_time + self.timestep

        self._update_state(obs)

        done = self._check_done(time)

        reward = self._compute_reward(last_action,requested_ang,obs, done, state_map)

        return obs, reward, done

    def reset(self, poses=None):

        self.current_time = 0.0
        if poses:
            pose_x = poses['x']
            pose_y = poses['y']
            pose_theta = poses['theta']
            self.x = pose_x[0]
            self.y = pose_y[0]
            self.theta = pose_theta[0]
            self.speed = 0.0

        else:
            # create reset proto
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.speed = 0.0


        # TODO: return with gym convention, one step?
        vels = [0.0] * self.num_agents
        angs = [0.0] * self.num_agents
        action = {'ego_idx': self.ego_idx, 'speed': vels, 'steer': angs}
        ref_points = []
        last_action = 0.0
        time = 0.0
        obs, reward, done = self.step(action, ref_points,last_action,time)

        return obs, reward, done
