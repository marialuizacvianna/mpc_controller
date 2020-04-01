import numpy as np
import math
import matplotlib.pyplot as plt

results_file = open("/home/maria/Documents/MPC/mpc_controller/dataset/result08.txt")

x0 = []
x1 = []
x2 = []
ref_x0 = []
ref_x1 = []
ref_theta = []
u = []


for line in results_file:
    split_line = line.split(',')
    if(len(split_line) == 7):
        x0.append(float(split_line[0]))
        x1.append(float(split_line[1]))
        x2.append(float(split_line[2]))

        ref_x0.append(float(split_line[3]))
        ref_x1.append(float(split_line[4]))
        ref_theta.append(float(split_line[5]))

        u.append(float(split_line[6]))


x0 = np.array(x0)
x1 = np.array(x1)
x2 = np.array(x2)
ref_x0 = np.array(ref_x0)
ref_x1 = np.array(ref_x1)
ref_theta = np.array(ref_theta)
u = np.array(u)


results_file.close()


fig1 = plt.figure()
ax1 = fig1.add_subplot(111)
time_plot_mpc, = plt.plot(u)
time_plot_mpc.set_label('MPC controller')
ax1.set_ylabel('Angular speed')
ax1.set_xlabel('Time')
ax1.set_title("Line following")

plt.show()
