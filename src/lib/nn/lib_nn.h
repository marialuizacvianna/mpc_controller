#ifndef LIB_NN
#define LIB_NN

#include <fstream>
#include <string>
#include <iostream>
#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"
#include <stdlib.h>

class NN_CONTROLLER
{
  public:
    NN_CONTROLLER(std::string file_name);
    int no_of_inputs_;
    int no_of_neurons_hidd_;
    int no_of_hidd_;
    int no_of_outputs_;
    double Solve_NN(Eigen::VectorXd inputs);
  private:
    int value;
    Eigen::MatrixXd W0_;
    Eigen::VectorXd b0_;

    Eigen::MatrixXd W1_;
    Eigen::VectorXd b1_;

    std::string trained_nn_file_name_;

    Eigen::VectorXd ReLu(Eigen::VectorXd x);
};



#endif
