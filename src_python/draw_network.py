from lib_draw_network import *
import time

name_inputs_file = ("../inputs_nn/line_sigmoid.txt")
# name_nn_file = ("/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/2704_2244/trained_nn_50") #relu
name_nn_file = ("/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/0505_1600/trained_nn_50") #sigmoid

inputs_file = open(name_inputs_file)
inputs_nn,results = read_inputs(inputs_file)
inputs_file.close()

nn_file = open(name_nn_file)

W,b = read_nn_file(nn_file)

nn_file.close()

no_of_inputs = W[0].shape[1]
no_of_outputs = W[1].shape[0]
no_of_neurons_hidd = W[0].shape[0]

count = 0

# fig = plt.figure(figsize=(20, 20))
# ax = fig.add_subplot(111)
ax = init_figure()
# inputs_nn = inputs_nn[0:3]
for inputs_nn_i in inputs_nn:
    clear(ax)
    relu_values, u = Solve_NN(inputs_nn_i,W,b)
    count = count + 1
    nb_neurons_active = draw_neural_net(ax,relu_values, .1, .9, .1, .9, [40, 50, 1])
    try:
        u = u[0][0]
    except:
        u = u
    plt.text(1.0,0.5,str(-3.0*u) )

    plt.text(0.0,0.0,"T = "+str(count*0.01)+" , Number of active sigmoid units: " + str(nb_neurons_active))
    print("ok")
    count = count +1
    plt.pause(0.05)

plt.close()
plt.show()
