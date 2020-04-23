# Model Predictive Control
A model predictive control (MPC) for an autonomous vehicle programmed in C++. This simulation is used for creating a dataset for supervised learning of a neural predictive controller.

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
The file CMakeLists.txt contains all requirements for creating the executable **mpc**. On the folder *build*

**First dataset :**


```
python3 train_MPC_network_controller_case0.py
```
