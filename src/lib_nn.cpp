#include "lib/nn/lib_nn.h"
#include "math.h"

NN_CONTROLLER::NN_CONTROLLER(std::string file_name){
  trained_nn_file_name_ = file_name;
  std::ifstream trained_nn_file(trained_nn_file_name_);

  std::string line;

  std::getline(trained_nn_file, line);
  no_of_inputs_ = atoi(line.c_str());

  std::getline(trained_nn_file, line);
  no_of_hidd_ = atoi(line.c_str());

  std::getline(trained_nn_file, line);
  no_of_outputs_ = atoi(line.c_str());

  std::getline(trained_nn_file, line);
  no_of_neurons_hidd_ = atoi(line.c_str());

  W1_ = Eigen::MatrixXd::Zero(no_of_neurons_hidd_, no_of_inputs_);
  b1_ = Eigen::VectorXd::Zero(no_of_neurons_hidd_);
  W0_ = Eigen::MatrixXd::Zero(no_of_outputs_, no_of_neurons_hidd_);
  b0_ = Eigen::VectorXd::Zero(no_of_outputs_);


  for(int row = 0; row < no_of_neurons_hidd_; row++){
    for(int col = 0; col < no_of_inputs_; col++){
      std::getline(trained_nn_file, line);
      W1_(row,col) = stod(line);
    }
    std::getline(trained_nn_file, line);
    b1_(row) = stod(line);
  }

  for(int row = 0; row < no_of_outputs_; row++){
    for(int col = 0; col < no_of_neurons_hidd_; col++){
      std::getline(trained_nn_file, line);
      W0_(row,col) = stod(line);
    }

    std::getline(trained_nn_file, line);
    b0_(row) = stod(line);
  }

  trained_nn_file.close();

}

double NN_CONTROLLER::Solve_NN(Eigen::VectorXd inputs){
  double output;

  Eigen::VectorXd y1 = Eigen::VectorXd::Zero(no_of_neurons_hidd_);

  y1 = (W1_*inputs) + b1_;

  // y1 = ReLu(y1);
  // y1 = Sigmoid(y1);
  y1 = Tanh(y1);

  Eigen::VectorXd yo = Eigen::VectorXd::Zero(no_of_outputs_);

  yo = (W0_*y1) + b0_;

  for(int i =0; i < no_of_outputs_; i ++){
    if(yo(i) < -0.836332)
      yo(i) = -0.836332;

    else if(yo(i) > 0.836332)
      yo(i) = 0.836332;
  }

  output = yo(0);

  return output;
}


Eigen::VectorXd NN_CONTROLLER::Sigmoid(Eigen::VectorXd x){
    const unsigned long VECTOR_SIZE = x.size();
    Eigen::VectorXd output (VECTOR_SIZE);

    for( unsigned i = 0; i != VECTOR_SIZE; ++i ) {
        output[ i ] = 1 / (1 + exp(-x[ i ]));
    }

    return output;
}

Eigen::VectorXd NN_CONTROLLER::Tanh(Eigen::VectorXd x){
    const unsigned long VECTOR_SIZE = x.size();
    Eigen::VectorXd output (VECTOR_SIZE);

    for( unsigned i = 0; i != VECTOR_SIZE; ++i ) {
        output[ i ] = tanh(x[ i ]);
    }

    return output;
}


Eigen::VectorXd NN_CONTROLLER::ReLu(Eigen::VectorXd x){
  Eigen::VectorXd A(x.rows());
  for( int i = 0; i< x.rows(); ++i ){
        if (x(i) <= 0)
            A(i)=(0.0);
        else
            A(i)=x(i);
  }

  return A;
}
