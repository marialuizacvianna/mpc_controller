#include "lib/nn/lib_nn.h"
#include "math.h"

NN_CONTROLLER::NN_CONTROLLER(){
  //initialize empty;
}

void NN_CONTROLLER::initialize_nn(std::string file_name){
  trained_nn_file_name_ = file_name;
  std::ifstream trained_nn_file(trained_nn_file_name_);

  std::string line;

  std::getline(trained_nn_file, line);
  no_of_inputs_ = atoi(line.c_str());

  std::getline(trained_nn_file, line);
  no_of_outputs_ = atoi(line.c_str());

  std::getline(trained_nn_file, line);
  no_of_hidd_ = atoi(line.c_str());

  for(int i =0; i < no_of_hidd_; i ++){
    std::getline(trained_nn_file, line);
    no_of_neurons_hidd_.push_back(atoi(line.c_str()));
  }

  Eigen::MatrixXd W_tmp;
  Eigen::VectorXd b_tmp;

  int no_of_rows;
  int no_of_cols;

  for(int i =0; i < no_of_hidd_ + 1 ; i ++){
    if(i == 0){
      W_tmp = Eigen::MatrixXd::Zero(no_of_neurons_hidd_.at(0), no_of_inputs_);
      b_tmp = Eigen::VectorXd::Zero(no_of_neurons_hidd_.at(0));

      no_of_rows = no_of_neurons_hidd_.at(0);
      no_of_cols = no_of_inputs_;
    }
    else if(i == no_of_hidd_){
      W_tmp = Eigen::MatrixXd::Zero(no_of_outputs_, no_of_neurons_hidd_.at(no_of_hidd_ - 1));
      b_tmp = Eigen::VectorXd::Zero(no_of_outputs_);

      no_of_rows = no_of_outputs_;
      no_of_cols =  no_of_neurons_hidd_.at(no_of_hidd_ - 1);

    }
    else{
      W_tmp = Eigen::MatrixXd::Zero(no_of_neurons_hidd_.at(i), no_of_neurons_hidd_.at(i-1));
      b_tmp = Eigen::VectorXd::Zero(no_of_neurons_hidd_.at(i));

      no_of_rows = no_of_neurons_hidd_.at(i);
      no_of_cols =  no_of_neurons_hidd_.at(i-1);

    }

    for(int row = 0; row < no_of_rows; row++){
      for(int col = 0; col < no_of_cols; col++){
        std::getline(trained_nn_file, line);
        W_tmp(row,col) = stod(line);
      }
      std::getline(trained_nn_file, line);
      b_tmp(row) = stod(line);
    }

    W_.push_back(W_tmp);
    b_.push_back(b_tmp);

  }

  trained_nn_file.close();

}

double NN_CONTROLLER::Solve_NN(Eigen::VectorXd inputs){
  double output;
  Eigen::VectorXd inputs_i = inputs;

  for(int i = 0; i < no_of_hidd_; i++){
    Eigen::VectorXd y_aux = Eigen::VectorXd::Zero(no_of_neurons_hidd_.at(i));
    y_aux = W_.at(i)*inputs_i + b_.at(i);
    // y_aux = Tanh(y_aux);
    // y_aux = ReLu(y_aux);
    y_aux = Sigmoid(y_aux);
    inputs_i = y_aux;
  }
  Eigen::VectorXd yo = Eigen::VectorXd::Zero(no_of_outputs_);
  yo = (W_.at(no_of_hidd_ )*inputs_i) + b_.at(no_of_hidd_);
  yo = Tanh(yo)*0.836332;

  output = yo(0);

  return output;
}


Eigen::VectorXd NN_CONTROLLER::Sigmoid(Eigen::VectorXd x){
    Eigen::VectorXd A(x.rows());

    for( int i = 0; i < x.rows(); i++ ) {
        A(i) = 1.0 / (1.0 + exp(-x(i)));
    }

    return A;
}

Eigen::VectorXd NN_CONTROLLER::Tanh(Eigen::VectorXd x){

    Eigen::VectorXd A(x.rows());

    for( int i = 0; i < x.rows(); i++ ) {
        A(i) = tanh(x(i));
    }

    return A;
}


Eigen::VectorXd NN_CONTROLLER::ReLu(Eigen::VectorXd x){
  Eigen::VectorXd A(x.rows());
  for( int i = 0; i< x.rows(); i++ ){
        if (x(i) <= 0)
            A(i)=(0.0);
        else
            A(i)=x(i);
  }

  return A;
}
