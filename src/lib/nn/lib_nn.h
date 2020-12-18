#ifndef LIB_NN
#define LIB_NN

#include <fstream>
#include <string>
#include <iostream>
#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"
#include <stdlib.h>
#include <vector>

class NN_CONTROLLER
{
  public:
    NN_CONTROLLER();
    void initialize_nn(std::string file_name);
    int no_of_inputs_;
    std::vector<int>  no_of_neurons_hidd_;
    int no_of_hidd_;
    int no_of_outputs_;
    double Solve_NN(Eigen::VectorXd inputs);
  private:
    int value;

    std::vector<Eigen::MatrixXd> W_;

    std::vector<Eigen::VectorXd> b_;
    std::string trained_nn_file_name_;

    Eigen::VectorXd Sigmoid(Eigen::VectorXd x);
    Eigen::VectorXd ReLu(Eigen::VectorXd x);
    Eigen::VectorXd Tanh(Eigen::VectorXd x);
};



#endif
