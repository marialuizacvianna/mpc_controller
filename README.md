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
./mpc  
```

The executable requires five inputs,

1. trajectory
  * **-traj** -  line,sinus,circle,race
  * **-freq** -  Frequency for sinusoidal trajectory.
  * **x0** -  Initial position in x.
  * **y0** -  Initial position in y.
  * **theta0** -  Initial position in theta.
