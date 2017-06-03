#include "kalman_filter.h"
#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

// Predict the position of the object at the current time step 
void KalmanFilter::Predict() {
  x_ = F_ * x_; // predict the new mean
  P_ = F_ * P_ * F_.transpose() + Q_ ; // predict new covariance  
}

// Update the predicted position with measurement data with the same spacial 
// plane coordinates
void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - (H_ * x_);
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = (P_ * Ht) * S.inverse();
  
  // Update the mean and covarience with the measured data
  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

// Update the predicted position with measurement data with a different spacial 
// plane coordinates using an extended kalman filter
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  
  // Calculate the polar coordinates of the object from the predicted position.
  // This is used to find the error between the measured polar data and the
  // predicted state. 
  // convert the predicted state position to a polar range
  float range_dash = sqrt(px * px + py * py);
  float angle_dash = atan2(py, px); // get the polar angle of the prediction
  // convert prediction to a polar velocity
  float velocity_dash = (px * vx + py * vy) / range_dash;
  
  const float PI = 3.14159265;
  // Make sure that the angle is between PI and -PI
  while((angle_dash > PI) || (angle_dash < -PI)){
    if(angle_dash > PI){
      angle_dash -= 2 * PI;
    } else {
      angle_dash += 2 * PI;
    }
  }

  VectorXd hx_dash = VectorXd(3);
  hx_dash << range_dash, angle_dash, velocity_dash;
  VectorXd y = z - hx_dash;  // error between measured and predicted data
  
  Hj_ = MatrixXd(3,4);
  Hj_ = tools.CalculateJacobian(x_);
  
  // calculate the kalman gain
  MatrixXd Hjt = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Hjt + R_;
  MatrixXd K = (P_ * Hjt) * S.inverse();
  cout << "K " << K << endl;
  
  // Update the prediction with the measured data
  x_ = x_ + K * y; // update the state/position matrix
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_) * P_; // update the covarience matrix
}
