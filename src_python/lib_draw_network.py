import numpy as np
import matplotlib.pyplot as plt
import time
import math

def read_inputs(input_files):
    inputs = []
    results = []

    for line in input_files:
        input_i = []
        split_line = line.split(',')
        for i in range(len(split_line) - 1):
            input_i.append(float(split_line[i]))
        results.append(float(split_line[len(split_line) - 1 ]))
        inputs.append(input_i)

    return inputs,results

def read_nn_file(nn_file):
    count_line  = 0
    network_structure = []
    no_of_neurons_hidd = []
    coefficients = []
    no_of_hidd = 10000
    for line in nn_file:
        if (count_line == 0):
            no_of_inputs = int(line)
            network_structure.append(no_of_inputs)
            print('no_of_inputs = ', no_of_inputs)

        if (count_line == 1):
            no_of_hidd = int(line)
            print('no_of_hidd = ', no_of_hidd)

        if (count_line == 2):
            no_of_outputs = int(line)
            print('no_of_outputs = ', no_of_outputs)

        if(count_line > 2 and count_line <= no_of_hidd+ 2):
            print("no_of_neurons_hidd = ",int(line))
            network_structure.append(int(line))
            no_of_neurons_hidd.append(int(line))

        if(count_line == no_of_hidd+ 2 ):
            network_structure.append(no_of_outputs)

        if(count_line > no_of_hidd+ 2):
            coefficients.append(float(line))

        count_line = count_line + 1

    coefficients  = np.array(coefficients)
    W = []
    b = []

    parse_coefficients = 0

    for i in range(len(network_structure) - 1):
        wi =  np.zeros((network_structure[i+1], network_structure[i]))
        bi =  np.zeros((network_structure[i+1], 1))

        for row in range(network_structure[i+1]):
            for col in range(network_structure[i]):
                wi[row,col] = coefficients[parse_coefficients]
                parse_coefficients = parse_coefficients + 1
            bi[row,0] = coefficients[parse_coefficients]
            parse_coefficients = parse_coefficients + 1

        W.append(wi)
        b.append(bi)

    return W,b


def ReLu (x):
    A =  np.zeros((len(x),1))
    mask = (x > 0.0)
    A = mask*x

    return A

def sigmoid(x):
  return 1/(1 + np.exp(-x))


def Solve_NN(inputs_nn,W,b,type):
    x = np.array([inputs_nn]).T
    for i in range(len(W) - 1):
        y = W[i]@x +  b[i]
        if(type == "relu"):
            y = ReLu(y)
        elif(type == "sigmoid"):
            y = sigmoid(y)
        else:
            y = np.tanh(y)
        vec_values = y
        x = y
    #output layer
    yo = W[len(W)- 1]@x +  b[len(W)- 1]
    u = max(min(yo,0.836332),-0.836332)
    return vec_values,u



def draw_neural_net(ax, vec_values, left, right, bottom, top, layer_sizes,type):
    '''
    Draw a neural network cartoon using matplotilb.

    :usage:
        >>> fig = plt.figure(figsize=(12, 12))
        >>> draw_neural_net(fig.gca(), .1, .9, .1, .9, [4, 7, 2])

    :parameters:
        - ax : matplotlib.axes.AxesSubplot
            The axes on which to plot the cartoon (get e.g. by plt.gca())
        - left : float
            The center of the leftmost node(s) will be placed here
        - right : float
            The center of the rightmost node(s) will be placed here
        - bottom : float
            The center of the bottommost node(s) will be placed here
        - top : float
            The center of the topmost node(s) will be placed here
        - layer_sizes : list of int
            List of layer sizes, including input and output dimensionality
    '''

    count_blue = 0
    count_red = 0
    count_white = 0
    nb_neurons_active = 0
    n_layers = len(layer_sizes)
    v_spacing = (top - bottom)/float(max(layer_sizes))
    h_spacing = (right - left)/float(len(layer_sizes) - 1)
    # Nodes
    for n, layer_size in enumerate(layer_sizes):
        layer_top = v_spacing*(layer_size - 1)/2. + (top + bottom)/2.
        for m in range(layer_size):
            if(type == "sigmoid"):
                if(n != 1):
                    circle = plt.Circle((n*h_spacing + left, layer_top - m*v_spacing), v_spacing/4.,color='w', ec='k', zorder=4)
                else:
                    if(vec_values[m] <= 0.1):
                        circle = plt.Circle((n*h_spacing + left, layer_top - m*v_spacing), v_spacing/4.,color='b', ec='k', zorder=4)
                        count_blue = count_blue +1
                    elif(vec_values[m] >= 0.9):
                        circle = plt.Circle((n*h_spacing + left, layer_top - m*v_spacing), v_spacing/4.,color='r', ec='k', zorder=4)
                        count_red = count_red +1
                    else:
                        circle = plt.Circle((n*h_spacing + left, layer_top - m*v_spacing), v_spacing/4.,color='w', ec='k', zorder=4)
                        count_white = count_white + 1

                circle_leg_b = plt.Circle((right,top), v_spacing/4.,color='b', ec='k', zorder=4)
                circle_leg_w = plt.Circle((right,top - 0.05), v_spacing/4.,color='w', ec='k', zorder=4)
                circle_leg_r = plt.Circle((right,top - 0.1), v_spacing/4.,color='r', ec='k', zorder=4)

                ax.add_artist(circle)
                ax.add_artist(circle_leg_b)
                ax.add_artist(circle_leg_w)
                ax.add_artist(circle_leg_r)

            elif(type == "relu"):
                if(n != 1):
                    circle = plt.Circle((n*h_spacing + left, layer_top - m*v_spacing), v_spacing/4.,color='w', ec='k', zorder=4)
                else:
                    if(vec_values[m] <= 0.0):
                        circle = plt.Circle((n*h_spacing + left, layer_top - m*v_spacing), v_spacing/4.,color='w', ec='k', zorder=4)
                        count_white = count_white +1
                    else:
                        circle = plt.Circle((n*h_spacing + left, layer_top - m*v_spacing), v_spacing/4.,color='r', ec='k', zorder=4)
                        count_red = count_red + 1

                circle_leg_w = plt.Circle((right,top), v_spacing/4.,color='w', ec='k', zorder=4)
                circle_leg_r = plt.Circle((right,top - 0.05), v_spacing/4.,color='r', ec='k', zorder=4)

                ax.add_artist(circle)
                ax.add_artist(circle_leg_w)
                ax.add_artist(circle_leg_r)

            elif(type == "tanh"):
                if(n != 1):
                    circle = plt.Circle((n*h_spacing + left, layer_top - m*v_spacing), v_spacing/4.,color='w', ec='k', zorder=4)
                else:
                    if(vec_values[m] <= -0.9):
                        circle = plt.Circle((n*h_spacing + left, layer_top - m*v_spacing), v_spacing/4.,color='b', ec='k', zorder=4)
                        count_blue = count_blue +1
                    elif(vec_values[m] >= 0.9):
                        circle = plt.Circle((n*h_spacing + left, layer_top - m*v_spacing), v_spacing/4.,color='r', ec='k', zorder=4)
                        count_red = count_red +1
                    else:
                        circle = plt.Circle((n*h_spacing + left, layer_top - m*v_spacing), v_spacing/4.,color='w', ec='k', zorder=4)
                        count_white = count_white + 1

                circle_leg_b = plt.Circle((right,top), v_spacing/4.,color='b', ec='k', zorder=4)
                circle_leg_w = plt.Circle((right,top - 0.05), v_spacing/4.,color='w', ec='k', zorder=4)
                circle_leg_r = plt.Circle((right,top - 0.1), v_spacing/4.,color='r', ec='k', zorder=4)

                ax.add_artist(circle)
                ax.add_artist(circle_leg_b)
                ax.add_artist(circle_leg_w)
                ax.add_artist(circle_leg_r)
        # # Edges
    # # Edges
    for n, (layer_size_a, layer_size_b) in enumerate(zip(layer_sizes[:-1], layer_sizes[1:])):
        layer_top_a = v_spacing*(layer_size_a - 1)/2. + (top + bottom)/2.
        layer_top_b = v_spacing*(layer_size_b - 1)/2. + (top + bottom)/2.
        for m in range(layer_size_a):
            for o in range(layer_size_b):
                line = plt.Line2D([n*h_spacing + left, (n + 1)*h_spacing + left],
                                  [layer_top_a - m*v_spacing, layer_top_b - o*v_spacing], c='k')
                if((m == 0 and o == 0 ) or (m == layer_size_a -1 and o == layer_size_b -1)):
                    ax.add_artist(line)

    return count_blue,count_red,count_white

def init_figure():
    fig = plt.figure(figsize=(20, 20))
    ax = fig.add_subplot(111)
    ax.axis('off')
    clear(ax)
    return ax

def clear(ax):
    plt.pause(0.001)
    ax.cla()
    ax.axis('off')
