"""
This class implements the policy neural network.
"""


import numpy as np
import math
from keras.initializers import normal, identity, uniform
from keras.models import model_from_json
from keras.models import Sequential, Model
from keras.layers import Dense, Flatten, Input, merge, Lambda, Activation
from keras.optimizers import Adam
import tensorflow.compat.v1 as tf
import keras.backend as K
from keras.constraints import max_norm
# Actor/Critic Neural Network Architecture #remove


class ActorNetwork(object):

    def __init__(self, sess, state_dim, action_dim, action_bound, learning_rate, tau):
        self.sess = sess
        self.s_dim = state_dim #40
        self.a_dim = action_dim #1
        self.action_bound = action_bound
        self.learning_rate = learning_rate
        self.tau = tau


        # Create non_target policy network
        self.inputs, self.out, self.scaled_out,self.model = self.create_actor_network(nn_type = "non_target", name='non-target')

        # Find trainable variables that will be needed for gradient calculations
        self.network_params = tf.trainable_variables()
        print(self.network_params[0].get_shape().as_list())


        # Create target policy network
        self.target_inputs, self.target_out, self.target_scaled_out,self.model_target = self.create_actor_network(nn_type = "target", name='target')

        # Find trainable variables that will be needed for gradient calculations
        self.target_network_params = tf.trainable_variables()[len(self.network_params):]


        # Below are some check that the trainable weights have the correct shape
        for i in range(len(self.target_network_params)):
            print(i)
            elem_1 = tf.multiply(self.network_params[i],self.tau)
            print("elem_1",elem_1.get_shape().as_list())
            elem_2 = tf.multiply(self.target_network_params[i],1.-self.tau)
            print("elem_2",elem_2.get_shape().as_list())
            tensor_sum = elem_1 + elem_2

        print("act_nn shape",elem_1)
        print("target_nn shape",elem_2)
        tensor_sum = (elem_1 + elem_2).get_shape().as_list()
        print("Sum shape",tensor_sum)

        # Create operation in graph to update target neural network
        self.update_target_network_params = [self.target_network_params[i].assign(tf.multiply(self.network_params[i], self.tau) + tf.multiply(self.target_network_params[i], 1. - self.tau)) for i in range(len(self.target_network_params))]
        print("length",len(self.update_target_network_params))

        # Store critic gradient with respect to actions.  To be used in policy gradient update.
        self.action_gradient = tf.placeholder(tf.float32, [None, self.a_dim])

        # Determine gradients of actions with respect to parameters of actor network.  Combine gradients of Q-value with respect
        # to action and action with respect to parameters of neural network.  The produce is negated to perform gradient ascent
        self.actor_gradients = tf.gradients(self.scaled_out, self.network_params, -self.action_gradient)

        self.optimize = tf.train.AdamOptimizer(self.learning_rate).apply_gradients(zip(self.actor_gradients, self.network_params))

        # self.optimize = Adam(self.learning_rate, clipnorm=1)
        #
        # self.optimize.apply_gradients(zip(self.actor_gradients, self.network_params))

        self.num_trainable_vars = len(self.network_params) + len(self.target_network_params)


    def save_network(self,file):
        print("nb layers = ",len(self.model.layers) )
        # print("layer 1 = ", len(self.model.layers[0])) #imput layer has no attribute shape or len
        # print("layer 1 = ", self.model.layers[1].shape) #'Flatten' object has no attribute 'shape'
        # print("layer 1 = ", self.model.layers[2].shape) #'Dense' object has no attribute 'shape'
        # print("layer 1 = ", self.model.layers[3].shape) #'Dense' object has no attribute 'shape'
        w1_smallest = self.model.layers[2].get_weights()[0]
        b1_smallest  = self.model.layers[2].get_weights()[1]
        wo_smallest = self.model.layers[3].get_weights()[0]
        bo_smallest  = self.model.layers[3].get_weights()[1]
        # print("w1_smallest = ", w1_smallest.shape) #('w1_smallest = ', (40, 50))
        # print("b1_smallest = ", b1_smallest.shape) #('b1_smallest = ', (50,))
        # print("wo_smallest = ", wo_smallest.shape) #('wo_smallest = ', (50, 1))
        # print("bo_smallest = ", bo_smallest.shape) #('bo_smallest = ', (1,))
        no_of_hidden_layers = 1
        no_of_neurons_0 = 50
        file.writelines(str(self.s_dim) + '\n')
        file.writelines(str(self.a_dim) + '\n')
        file.writelines(str(no_of_hidden_layers) + '\n')
        for i in range(no_of_hidden_layers):
            file.writelines(str(no_of_neurons_0) + '\n')

        for col_no in range(no_of_neurons_0):
            for row_no in range(self.s_dim):
                file.writelines(str(w1_smallest[row_no, col_no]) + '\n')
            file.writelines(str(b1_smallest[col_no]) + '\n')

        for col_no in range(self.a_dim):
            for row_no in range(no_of_neurons_0):
                file.writelines(str(wo_smallest[row_no, col_no]) + '\n')
            file.writelines(str(bo_smallest[col_no]) + '\n')

    # Create policy network with architecture provided.  The policy network is a mapping from states to actions.
    def create_actor_network(self, nn_type, name):
        # States are the input variable
        states = Input(shape = (self.s_dim,) + (1,))
        flattened = Flatten()(states)

        h1 = Dense(units = 50, activation='tanh', name=name + 'hl1')(flattened)


        act_not_scaled = Dense(units = self.a_dim, activation = 'tanh', name = name + 'ol')(h1)

        act_scaled = Lambda(lambda x: x * self.action_bound, name=name + 'ols')(act_not_scaled)

        model = Model(states, act_not_scaled)

        return states, act_not_scaled, act_scaled, model


    # The train function is how the "Learning" phase of the learning loop is implemented.  The policy neural network
    # parameters are updated in the diretion of the deterministic policy gradient
    def train(self, inputs, a_gradient):
        self.sess.run(self.optimize, feed_dict={
            self.inputs: inputs,
            self.action_gradient: a_gradient
        })

    # The predict function is used to find the action prescribed by the policy network given the current state.
    # The agent will experiment by adding noise to the output of this function
    def predict(self, inputs):
        return self.sess.run(self.scaled_out, feed_dict={
            self.inputs: inputs
        })

    def predict_target(self, inputs):
        return self.sess.run(self.target_scaled_out, feed_dict={
            self.target_inputs: inputs
        })

    # This function updates the target neural network by using values from the non_target network
    def update_target_network(self):
        self.sess.run(self.update_target_network_params)

    def get_num_trainable_vars(self):
        return self.num_trainable_vars
