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
  // std::cout << "no_of_inputs_ == " << no_of_inputs_ <<std::endl;
  // std::cout << "no_of_hidd_ == " << no_of_hidd_ <<std::endl;
  // std::cout << "no_of_outputs_ == " << no_of_outputs_ <<std::endl;
  // for(int i =0; i < no_of_hidd_  ; i ++){
  //   std::cout << "no_of_neurons_hidd_ == " << no_of_neurons_hidd_.at(i) <<std::endl;
  // }
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
        // std::cout << "row = " << row << std::endl;
        // std::cout << "col = " << col << std::endl;
        // std::cout << "W_tmp(row,col) = " << W_tmp(row,col) << std::endl;
      }
      std::getline(trained_nn_file, line);
      b_tmp(row) = stod(line);
    }

    W_.push_back(W_tmp);
    b_.push_back(b_tmp);

  }



  //
  // for(int row = 0; row < no_of_neurons_hidd_; row++){
  //   for(int col = 0; col < no_of_inputs_; col++){
  //     std::getline(trained_nn_file, line);
  //     W1_(row,col) = stod(line);
  //   }
  //   std::getline(trained_nn_file, line);
  //   b1_(row) = stod(line);
  // }
  //
  // for(int row = 0; row < no_of_outputs_; row++){
  //   for(int col = 0; col < no_of_neurons_hidd_; col++){
  //     std::getline(trained_nn_file, line);
  //     W0_(row,col) = stod(line);
  //   }
  //
  //   std::getline(trained_nn_file, line);
  //   b0_(row) = stod(line);
  // }

  trained_nn_file.close();

}

double NN_CONTROLLER::Solve_NN(Eigen::VectorXd inputs){
  double output;


  Eigen::VectorXd inputs_i = inputs;

  for(int i = 0; i < no_of_hidd_; i++){
    // std::cout <<" i = " << i <<std::endl;
    Eigen::VectorXd y_aux = Eigen::VectorXd::Zero(no_of_neurons_hidd_.at(i));
    // std::cout <<" y_aux before relu= " << y_aux <<std::endl;
    y_aux = W_.at(i)*inputs_i + b_.at(i);
    // std::cout <<" W_.at(i) = " << W_.at(i) <<std::endl;
    // std::cout <<" b_.at(i) = " << b_.at(i) <<std::endl;
    // std::cout <<" W_.at(i).rows() = " << W_.at(i).rows() <<std::endl;
    // std::cout <<" W_.at(i).cols() = " << W_.at(i).cols() <<std::endl;
    // y_aux = Tanh(y_aux);
    // y_aux = ReLu(y_aux);
    y_aux = Sigmoid(y_aux);
    // std::cout <<" y_aux after relu= " << y_aux <<std::endl;
    // std::cout <<" y_aux ReLu= " << y_aux <<std::endl;
    inputs_i = y_aux;
  }
  // std::cout <<" no_of_outputs_ = " << no_of_outputs_ <<std::endl;
  // std::cout <<" inputs_i.cols() = " << inputs_i.cols() <<std::endl;
  Eigen::VectorXd yo = Eigen::VectorXd::Zero(no_of_outputs_);
  yo = (W_.at(no_of_hidd_ )*inputs_i) + b_.at(no_of_hidd_);
  // std::cout <<" W_.at(no_of_hidd_ ).rows() = " << W_.at(no_of_hidd_ ).rows() <<std::endl;
  // std::cout <<"yo = " << yo <<std::endl;
  // std::cout <<"Tanh(yo) = " << Tanh(yo) <<std::endl;
  yo = Tanh(yo)*0.836332;

  // std::cout <<" ### "  <<std::endl;
  // Eigen::VectorXd y1 = Eigen::VectorXd::Zero(no_of_neurons_hidd_);
  //
  // y1 = (W1_*inputs) + b1_;
  //
  // // y1 = ReLu(y1);
  // // y1 = Sigmoid(y1);
  // y1 = Tanh(y1);

  // Eigen::VectorXd yo = Eigen::VectorXd::Zero(no_of_outputs_);
  //
  // yo = (W0_*y1) + b0_;

  // for(int i =0; i < no_of_outputs_; i ++){
  //   if(yo(i) < -0.836332)
  //     yo(i) = -0.836332;
  //
  //   else if(yo(i) > 0.836332)
  //     yo(i) = 0.836332;
  // }

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
