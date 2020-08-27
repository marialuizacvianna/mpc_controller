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

# protobuf import
import sim_requests_pb2

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

# from matplotlib.pyplot import imshow
# import matplotlib.pyplot as plt

class F110Env(gym.Env, utils.EzPickle):
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

        # env states
        # more accurate description should be ego car state
        # might not need to keep scan
        self.x = None
        self.y = None
        self.theta = None

        self.in_collision = False
        self.collision_angle = None

        self.origin = []


    def _start_executable(self, path):
        mu = self.params[0]
        h_cg = self.params[1]
        l_r = self.params[2]
        cs_f = self.params[3]
        cs_r = self.params[4]
        I_z = self.params[5]
        mass = self.params[6]
        args = [path+'sim_server', str(self.timestep), str(self.num_agents), str(self.port), str(mu), str(h_cg), str(l_r), str(cs_f), str(cs_r), str(I_z), str(mass)]


    def _check_done(self):
        """
        Check if the episode is done
        This is in terms of the ego car
        For our case, whether the car ends up close enough to the starting point
        And if accumulated time is over the timeout
        return true if done, false if not
        This assumes start is always (0, 0)

        """

        # to do


    def _update_state(self, obs_dict):
        """
        Update the env's states according to observations
        obs is observation dictionary
        """
        self.x = obs_dict['poses_x'][obs_dict['ego_idx']]
        self.y = obs_dict['poses_y'][obs_dict['ego_idx']]
        self.theta = obs_dict['poses_theta'][obs_dict['ego_idx']]



    def compute_reward(self, observations, done, state_map):
        if(len(state_map) == 0 ):
            return 0

        #TODO

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


        map_dist = (first_ref_x - X)**2 + (first_ref_y - Y)**2 + 100*(angle_ref - theta)**2

        reward = -map_dist

        reward =  max(min(reward, 0), -10)

        return reward


    def step(self, action, state_map):
        #TODO
        # can't step if params not set
        if not self.params_set:
            print('ERROR - Gym Env - Params not set, call update params before stepping.')
            sys.exit()
        # action is a list of steering angles + command velocities
        # also a ego car index
        # action should a DICT with {'ego_idx': int, 'speed':[], 'steer':[]}
        requested_vel = action['speed']
        requested_ang = action['steer']

        #TODO APPLY REQUEST STEP

        # TODO get results
        # TODO: also check for stepping fail
        #observations and future entry to neural network

        # construct observation dict
        # Observation DICT, assume indices consistent: {'ego_idx':int, 'scans':[[]], 'poses_x':[], 'poses_y':[], 'poses_theta':[], 'linear_vels_x':[], 'linear_vels_y':[], 'ang_vels_z':[], 'collisions':[], 'collision_angles':[]}
        obs = {'ego_idx': observations_proto.ego_idx, 'scans': [], 'poses_x': [], 'poses_y': [], 'poses_theta': [], 'linear_vels_x': [], 'linear_vels_y': [], 'ang_vels_z': [], 'collisions': [], 'collision_angles': [], 'lap_times': [], 'lap_counts': []}
        for car_obs in carobs_list:
            obs['scans'].append(car_obs.scan)
            obs['poses_x'].append(car_obs.pose_x)
            obs['poses_y'].append(car_obs.pose_y)
            if abs(car_obs.theta) < np.pi:
                obs['poses_theta'].append(car_obs.theta)
            else:
                obs['poses_theta'].append(-((2 * np.pi) - car_obs.theta))
            obs['linear_vels_x'].append(car_obs.linear_vel_x)
            obs['linear_vels_y'].append(car_obs.linear_vel_y)
            obs['ang_vels_z'].append(car_obs.ang_vel_z)
            obs['collisions'].append(car_obs.collision)
            obs['collision_angles'].append(car_obs.collision_angle)

        obs['lap_times'] = self.lap_times
        obs['lap_counts'] = self.lap_counts


        # update accumulated time in env
        self.current_time = self.current_time + self.timestep
        # TODO: donezo should be done in simulator? could be done here as well
        self._update_state(obs)
        if self.double_finish:
            done, temp = self._check_done()
            info = {'checkpoint_done': temp}
        else:
            done = self._check_done()
            info = {}

        # print("step , x = ",obs['poses_x'][0], " ,y = ",obs['poses_y'][0], " , theta = ", obs['poses_theta'][0])
        reward = self.compute_reward(obs, done, state_map)
        # TODO: return obs, reward, done, info
        return obs, reward, done, info

    def reset(self, poses=None):

        self.current_time = 0.0
        self.in_collision = False
        self.collision_angles = None
        self.num_toggles = 0
        self.near_start = True
        self.near_starts = np.array([True]*self.num_agents)
        self.toggle_list = np.zeros((self.num_agents,))
        if poses:
            pose_x = poses['x']
            pose_y = poses['y']
            pose_theta = poses['theta']
            self.start_x = pose_x[0]
            self.start_y = pose_y[0]
            self.start_theta = pose_theta[0]
            self.start_xs = np.array(pose_x)
            self.start_ys = np.array(pose_y)
            self.start_thetas = np.array(pose_theta)
            self.start_rot = np.array([[np.cos(-self.start_theta), -np.sin(-self.start_theta)],
                                        [np.sin(-self.start_theta), np.cos(-self.start_theta)]])
            # create reset by pose proto
            reset_request_proto = sim_requests_pb2.SimRequest()
            reset_request_proto.type = 4
            reset_request_proto.reset_bypose_request.num_cars = self.num_agents
            reset_request_proto.reset_bypose_request.ego_idx = 0
            reset_request_proto.reset_bypose_request.car_x.extend(pose_x)
            reset_request_proto.reset_bypose_request.car_y.extend(pose_y)
            reset_request_proto.reset_bypose_request.car_theta.extend(pose_theta)
            reset_request_string = reset_request_proto.SerializeToString()
            self.socket.send(reset_request_string)
        else:
            # create reset proto
            self.start_x = 0.0
            self.start_y = 0.0
            self.start_theta = 0.0
            self.start_rot = np.array([[np.cos(-self.start_theta), -np.sin(-self.start_theta)],
                                        [np.sin(-self.start_theta), np.cos(-self.start_theta)]])
            reset_request_proto = sim_requests_pb2.SimRequest()
            reset_request_proto.type = 2
            reset_request_proto.reset_request.num_cars = self.num_agents
            reset_request_proto.reset_request.ego_idx = 0
            # serialize reset proto
            reset_request_string = reset_request_proto.SerializeToString()
            # send reset proto string
            self.socket.send(reset_request_string)
        # receive response from sim
        reset_response_string = self.socket.recv()
        reset_response_proto = sim_requests_pb2.SimResponse()
        reset_response_proto.ParseFromString(reset_response_string)
        if reset_response_proto.reset_resp.result:
            print('Gym env - Reset failed')
            # TODO: failure handling
            return None
        # TODO: return with gym convention, one step?
        vels = [0.0] * self.num_agents
        angs = [0.0] * self.num_agents
        action = {'ego_idx': self.ego_idx, 'speed': vels, 'steer': angs}
        # print('Gym env - Reset done')
        ref_points = []

        obs, reward, done, info = self.step(action, ref_points)
        # print('Gym env - step done for reset')
        return obs, reward, done, info

    def init_map(self, map_path, img_ext, rgb, flip):
        """
            init a map for the gym env
            map_path: full path for the yaml, same as ROS, img and yaml in same dir
            rgb: map grayscale or rgb
            flip: if map needs flipping
        """

        self.map_path = map_path
        if not map_path.endswith('.yaml'):
            print('Gym env - Please use a yaml file for map initialization.')
            print('Exiting...')
            sys.exit()

        # split yaml ext name
        map_img_path = os.path.splitext(self.map_path)[0] + img_ext
        self.map_img = np.array(Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM))
        self.map_img = self.map_img.astype(np.float64)
        if flip:
            self.map_img = self.map_img[::-1]

        if rgb:
            self.map_img = np.dot(self.map_img[..., :3], [0.29, 0.57, 0.14])

        # update map metadata
        self.map_height = self.map_img.shape[0]
        self.map_width = self.map_img.shape[1]
        self.free_thresh = 0.6  # TODO: double check
        with open(self.map_path, 'r') as yaml_stream:
            try:
                map_metadata = yaml.safe_load(yaml_stream)
                self.map_resolution = map_metadata['resolution']
                self.origin = map_metadata['origin']
            except yaml.YAMLError as ex:
                print(ex)
        self.map_inited = True

        # load waypoints
        # self.csv_path = os.path.splitext(self.map_path)[0] + '.csv'
        # with open(self.csv_path) as f:
        #     self.waypoints = [tuple(line) for line in csv.reader(f)]
        #     # waypoints are [x, y, speed, theta]
        #     self.waypoints = np.array([(float(pt[0]), float(pt[1]), float(pt[2]), float(pt[3])) for pt in self.waypoints])


    def render(self, mode='human', close=False):
        return

    # def get_min_dist(self, position):
    #     wpts = self.waypoints[:, 0:2]
    #      # = position[0:2]
    #     nearest_point, nearest_dist, t, i = self.nearest_point_on_trajectory(position, wpts)
    #     # speed = self.waypoints[i, 2]
    #     return nearest_dist

    # def nearest_point_on_trajectory(self, point, trajectory):
    #     '''
    #     Return the nearest point along the given piecewise linear trajectory.

    #     Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
    #     not be an issue so long as trajectories are not insanely long.

    #         Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)

    #     point: size 2 numpy array
    #     trajectory: Nx2 matrix of (x,y) trajectory waypoints
    #         - these must be unique. If they are not unique, a divide by 0 error will destroy the world
    #     '''
    #     diffs = trajectory[1:,:] - trajectory[:-1,:]
    #     l2s   = diffs[:,0]**2 + diffs[:,1]**2
    #     # this is equivalent to the elementwise dot product
    #     dots = np.sum((point - trajectory[:-1,:]) * diffs[:,:], axis=1)
    #     t = np.clip(dots / l2s, 0.0, 1.0)
    #     projections = trajectory[:-1,:] + (t*diffs.T).T
    #     dists = np.linalg.norm(point - projections,axis=1)
    #     min_dist_segment = np.argmin(dists)
    #     return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment

    def update_params(self, mu, h_cg, l_r, cs_f, cs_r, I_z, mass, exe_path, double_finish=False):
        # if not self.sim_p is None:
        #     print('Gym env - Sim server exists, killing...')
        #     self.socket.send(b'dead')
        #     self.sim_p.kill()
        #     os.kill(self.sim_p.pid, signal.SIGINT)
        #     self.sim_p = None
        # print('in update params')

        self.params = [mu, h_cg, l_r, cs_f, cs_r, I_z, mass]
        self.params_set = True
        if self.sim_p is None:
            # print('starting ex and setting map')
            self._start_executable(exe_path)
            self._set_map()
        self.double_finish = double_finish
        # print('before creating proto')

        # create update proto
        update_param_proto = sim_requests_pb2.SimRequest()
        update_param_proto.type = 3
        update_param_proto.update_request.mu = mu
        update_param_proto.update_request.h_cg = h_cg
        update_param_proto.update_request.l_r = l_r
        update_param_proto.update_request.cs_f = cs_f
        update_param_proto.update_request.cs_r = cs_r
        update_param_proto.update_request.I_z = I_z
        update_param_proto.update_request.mass = mass
        # serialize reset proto
        update_param_string = update_param_proto.SerializeToString()
        # print('proto serialized')
        # send update param request
        self.socket.send(update_param_string)
        # print('Gym env - Update param request sent.')
        # receive response
        update_response_string = self.socket.recv()
        update_response_proto = sim_requests_pb2.SimResponse()
        update_response_proto.ParseFromString(update_response_string)
        if update_response_proto.update_resp.result:
            print('Gym env - Update param failed')
            return None

        # print('Gym env - params updated.')
        # start executable
        # self._start_executable()
        # call set map
        # self._set_map()
