# Model Predictive Control
A model predictive control (MPC) for an autonomous vehicle programmed in C++. This simulation is used for creating a dataset for supervised learning of a neural predictive controller. The simulation is also used for validating the neural controllers, replacing the classical MPC by the trained neural networks and allowing a comparison between the trajectory followed by the two types of controllers.

## Installing dependencies

We use Ipopt for realizing the non-linear optimization at each time step. I recommend the [example](https://www.coin-or.org/CppAD/Doc/ipopt_solve_get_started.cpp.htm) available on the official documentation for understanding how it works.

Installing on **ubuntu**:

```
sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
```
```
sudo apt-get install coinor-libipopt-dev
```

For the simulation we use OpenGL, it should not require an special installation.

## Make
Inside */build*, run:

```
make
```
 The executable **mpc** will be created according to the dependencies established at the file *CMakeLists.txt*.

## Run
Inside */build*,

```
./mpc trajectory neurons_hidden_layer initial_offset initial_orientation frequency
```

The executable requires five inputs,

1. trajectory
  * **0** -  The vehicle will follow a straight trajectory.
  * **1** -  The vehicle will follow a sinusoidal trajectory.
  * **2** -  The vehicle will follow a spiral trajectory.
  * **3** -  The vehicle will follow a quadratic trajectory.


2. neurons_hidden_layer , It concerns the number of neurons used on the hidden layer of the network, options are:
  * **0** -  No vehicle controlled by a neural network. Only simulate the classical MPC.
  * **3 to 155** -  One vehicle will be controlled by the classical MPC and another by a feed forward neural network with one hidden layer with 3 to 155 neurons.

3. initial_offset , The initial position of the robot on the y coordinate. It can be any rational number.

4. initial_orientation , The initial yaw angle of the robot with respect to the global frame in radians.

5. frequency , This variable is only important when the vehicle must follow a sinusoidal trajectory. It corresponds to the frequency of the sine/cosine that forms the reference trajectory.


## Other running options

On file */src/lib/mpc/lib_mpc.h* there are other options for running the simulation. Normally, these options are set to default values and do not need to be modified. However, they can be modified according to the objective of the simulation. For example,


1. *T* - It represents the total time of the simulation. The default value is 20 seconds. It must be a positive value.

2. *LINE* - It represents the offset of the reference trajectory regarding the y coordinate. The default value is zero.

3. *RESULTS_FILE* -
  * **0** - Default value.
  * **1** -  If 1, it creates a file containing a dataset for supervised learning of the neural controller with data from the simulation. The dataset file is stored on the folder  */dataset*.

    It is necessary to **change the path** for saving the file at line 174 of file */src/lib_io.cpp* .

3. *RESULTS_NN_FILE* -
  * **0** - Default value.
  * **1** -  If 1, it creates a file containing the trajectory followed by the vehicle controlled by the neural network and the values of angular speed calculated by it. It also includes the same information for the vehicle controlled by the classical MPC, this allows the comparison between both performances.

    It is necessary to **change the path** for saving the file at line 178 of file */src/lib_io.cpp* .


## Reading the Neural Network file

The constructor of the neural network solver uses a script for parsing the neural network file transforming it in matrices of weights and vectors of bias. The script can be found on the file */src/lib_nn.cpp* on the function *NN_CONTROLLER::NN_CONTROLLER(std::string file_name)* .
