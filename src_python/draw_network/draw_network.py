from lib_draw_network import *
import time

name_inputs_file = ("../inputs_nn/line_sigmoid.txt")
# name_nn_file = ("/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/2704_2244/trained_nn_50") #relu
name_nn_file = ("/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/1805_0808/trained_nn_50") #sigmoid
type = "sigmoid"

inputs_file = open(name_inputs_file)
inputs_nn,results = read_inputs(inputs_file)
inputs_file.close()

nn_file = open(name_nn_file)

W,b = read_nn_file(nn_file)

nn_file.close()

no_of_inputs = W[0].shape[1]
no_of_outputs = W[1].shape[0]
no_of_neurons_hidd = W[0].shape[0]


time = 0.0
# fig = plt.figure(figsize=(20, 20))
# ax = fig.add_subplot(111)
ax = init_figure()
# inputs_nn = inputs_nn[0:3]
for inputs_nn_i in inputs_nn:
    clear(ax)
    vec_values, u = Solve_NN(inputs_nn_i,W,b,type)

    count_blue,count_red,count_white = draw_neural_net(ax,vec_values, .1, .9, .1, .9, [40, 50, 1],type)
    try:
        u = u[0][0]
    except:
        u = u

    plt.text(1.0,0.5,str(-u) )


    if(type == "sigmoid"):
        plt.text(.9 + 0.01,.9," Number of units <= 0.1 =  {:3d}".format(count_blue))
        plt.text(.9 + 0.01,.9 - 0.05," Number of units > 0.1 and < 0.9 =  {:3d}".format(count_white))
        plt.text(.9 + 0.01,.9 - 0.1," Number of units >= 0.9 =  {:3d}".format(count_red))

    elif(type == "tanh"):
        plt.text(.9 + 0.01,.9," Number of units <= -0.9 =  {:3d}".format(count_blue))
        plt.text(.9 + 0.01,.9 - 0.05," Number of units > -0.9 and < 0.9 =  {:3d}".format(count_white))
        plt.text(.9 + 0.01,.9 - 0.1," Number of units >= 0.9 =  {:3d}".format(count_red))

    elif(type == "relu"):
        plt.text(.9 + 0.01,.9," Number of units <= 0.0 =  {:3d}".format(count_white))
        plt.text(.9 + 0.01,.9 - 0.05," Number of units > 0.0 =  {:3d}".format(count_red))

    else:
        print("Activation function not recognized, try: relu, sigmoid or tanh")
    #
    plt.text(0.0,0.0,"T = {:.2f} ".format(time))
    #
    #
    time = time + 0.01

    plt.pause(0.05)

plt.close()
plt.show()
