import os
import random
import time

import numpy as np


import tensorflow.compat.v1 as tf

from Actor import ActorNetwork

from Critic import CriticNetwork



from utils import Memory, H5Buffer

# Learning rates
MAX_ACTOR_LEARNING_RATE = 1e-3
MIN_ACTOR_LEARNING_RATE = 1e-4

MAX_CRITIC_LEARNING_RATE = 5e-4
MIN_CRITIC_LEARNING_RATE = 5e-4


# Tau reflects how quickly target networks should be updated
TAU = 0.001

class DDPG_Agent():
    def __init__(self, sess,state_size, action_size, buffer_max_size, chunk_size, action_bound, always_explore=False):
        file_name = 'ddpg_learnig_nn'
        self.sess = sess


        self.chunk_size = chunk_size
        self.always_explore = always_explore

        self.working_dir = "home/maria/Documents"
        self.weight_backup      = os.path.join(self.working_dir, file_name+'.h5')
        print("self.working_dir, file_name+'.h5' = ", self.working_dir, file_name+'.h5')

        self.state_size         = state_size
        self.action_size        = action_size
        self.buffer             = H5Buffer(state_size, buffer_max_size)
        self.learning_rate      = 0.001
        self.gamma              = 0.99
        self.exploration_rate   = 0.85
        self.exploration_min    = 0.01
        self.exploration_decay  = 0.9

        self.action_bound = action_bound

        self.actor = ActorNetwork(self.sess, self.state_size, self.action_size, self.action_bound,
                                 MAX_ACTOR_LEARNING_RATE, TAU)


        self.critic = CriticNetwork(self.sess, self.state_size, self.action_size,
                               MIN_CRITIC_LEARNING_RATE, TAU, self.actor.get_num_trainable_vars())




    def save_model(self):
        rospy.loginfo("Model saved")
        self.model.save(self.weight_backup)

    def act(self, state):
        # if np.random.rand() < self.exploration_rate:
        #     return random.uniform(-1, 1)*self.action_bound
        act_values = self.actor.predict(state)
        return act_values

    def replay(self, new_data):
        rospy.loginfo("Replaying..."),

        self.buffer.extend(new_data)
        buffer_length = self.buffer.length()

        chunks = buffer_length / self.chunk_size

        chunk_n = 2
        if chunks < 2:
            chunk_n = 1
            chunks=1
        print('buffer length', buffer_length)
        print('chunks', chunks)

        for i in np.random.choice(range(chunks), chunk_n, False):
            print('fitting', i)
            start_idx = i * self.chunk_size
            end_idx = start_idx + self.chunk_size

            loading_time = time.time()
            actions, states, next_states, rewards, terminates = self.buffer.sample(start_idx, end_idx)
            print('loading {} samples time: {}'.format(self.chunk_size, time.time()-loading_time))

            not_done = np.invert(terminates)
            rewards_new = np.copy(rewards)

            tmp_pred = self.model.predict(next_states[not_done], batch_size=1000)
            next_pred = tmp_pred.max(axis=1)
            rewards_new[not_done]+= self.gamma * next_pred
            targets = self.model.predict(states)
            targets[np.arange(len(actions)), actions] = rewards_new


            fit_time = time.time()
            self.model.fit(states, targets, shuffle=True, batch_size=1000, epochs=1, verbose=0)
            print('fit time:', time.time()-fit_time)

        if self.exploration_rate > self.exploration_min:
            self.exploration_rate *= self.exploration_decay

        self.save_model()
