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
 * @param estimations : predicted position
 * @param ground_truth : actual measured position
 * @return : the error between the estimate and ground_truth 
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

double Tools::ZeroCheck(const double &value, const double epsilon) {
  /*
   * Checks to see if a value is near zero.
   * @param value : value to check
   * @param epsilon : near zero value to compare input against
   * @return : value if not between +/- epsilon else epsilon
   */
  if(value < epsilon){ 
    return epsilon;
  }else{
    return value;
  }
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /*
   * Calculate the Jacobian for non-linear function.
   * @param x_state : vector containing position and velocity
   * @return : jacobian
  */
  //recover state positions and velocity
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  //compute the Jacobian matrix
  double pSquareSum = Tools::ZeroCheck(px*px + py*py);
  double pSquareRoot =  Tools::ZeroCheck(sqrt(pSquareSum));
  double pSquareCube =  Tools::ZeroCheck(sqrt(pSquareSum*pSquareSum*pSquareSum));
  double pvDiff1 = vx*py - vy*px;
  double pvDiff2 = vy*px - vx*py;

  MatrixXd Hj = MatrixXd(3,4);
  Hj << px/pSquareRoot, py/pSquareRoot, 0, 0,
       -1.*py/pSquareSum, px/pSquareSum, 0, 0,
       (py*pvDiff1)/pSquareCube, (py*pvDiff2)/pSquareCube, px/pSquareRoot, py/pSquareRoot;

  return Hj;
}
