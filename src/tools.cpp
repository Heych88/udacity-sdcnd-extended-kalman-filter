#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
/*
 * Calculates the Root Mean Square Error of between the the estimated position
 * and the true position.
 * RMSE = sqrt(1/(vector size) * sum(square(estimate-truth)))
 */
   
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the input vectors have elements.
  int size = estimations.size();
  if (size != ground_truth.size() || size == 0){
      std::cout << "Error: estimations,size() != ground_truth.size()" << std::endl;
      return rmse;
  }

  // Root Mean Square Error
  // RMSE = sqrt(1/(vector size) * sum(square(estimate-truth)))
  for(int i=0; i < size; i++){
    VectorXd difference = estimations[i] - ground_truth[i];
    difference = difference.array()*difference.array();
    rmse += difference;
  }
  rmse = rmse/size; // mean
  rmse = rmse.array().sqrt(); 

  return rmse;
}

float Tools::ZeroCheck(const float &value, const float epsilon=1.0e-4) {
  /*
   * Checks to see if a value is near zero.
   * @param value : value to check
   * @param epsilon : near zero value to compare input against
   * @return : value if not between +/- epsilon else epsilon
   */
  if(value < epsilon && -value > -epsilon){
    return epsilon;
  }else{
    return value;
  }
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state positions and velocity
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // check the input vectors have elements.
  if(px == 0 ||py == 0||vx == 0||vy == 0){
      std::cout << "Error with divide by Zero" << std::endl;
  }

  //compute the Jacobian matrix
  float pSquare = Tools::ZeroCheck(px*px + py*py);
  float pSquareRoot =  Tools::ZeroCheck(sqrt(pSquare));
  float pSquareCube =  Tools::ZeroCheck(sqrt(pSquare*pSquare*pSquare));
  float pvDiff1 = vx*py - vy*px;
  float pvDiff2 = vy*px - vx*py;

  Hj << px/pSquareRoot, py/pSquareRoot, 0, 0,
       -py/pSquare, px/pSquare, 0, 0,
       (py*pvDiff1)/pSquareCube, (py*pvDiff2)/pSquareCube, px/pSquareRoot, py/pSquareRoot;

  return Hj;
}
