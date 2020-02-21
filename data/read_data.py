import numpy as np
import matplotlib.pyplot as plt

plt.close('all')

file = open('/home/maria/Documents/540-Project/data/testQuad.txt')

y = []
u =  []
ref = []

for line in file:
    line = line.split()
    u.append(float(line[0]))
    y.append(float(line[1]))
    ref.append(float(line[2]))

file.close()

u = np.array(u)
y = np.array(y)
ref = np.array(ref)

# print(u.shape)
# print(y.shape)
# print(ref.shape)

plt.figure()
# plt.plot(y[0:100],'r')
plt.plot(ref,'b')
plt.show()


file = open('/home/maria/Documents/MPC/mpc_controller/data/ref_files/quad_ref.txt', 'w')
for line in ref:
  file.write(str(line))
  file.write("\n")
file.close()
