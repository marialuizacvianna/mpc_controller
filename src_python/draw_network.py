from lib_draw_network import *
import time

name_inputs_file = ("../inputs_nn/line_relu.txt")
name_nn_file = ("/home/maria/Documents/nn_mpc/trained_nn/50 neurons/training_minimum_cost/2704_2244/trained_nn_50")

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

for inputs_nn_i in inputs_nn:
    clear(ax)
    u = Solve_NN(inputs_nn_i,W,b)
    # print("u = ", -3.0*u)
    # print("expected u = ",results[count])
    count = count + 1
    draw_neural_net(ax, .1, .9, .1, .9, [40, 50, 1])
    print("ok")
    plt.pause(0.05)


plt.show()
