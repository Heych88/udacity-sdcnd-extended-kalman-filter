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

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_; // predict the new mean
  P_ = F_ * P_ * F_.transpose() + Q_ ; // predict new covariance  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  
  float range_dash = sqrt(px * px + py * py); // convert prediction position to a range
  float angle_dash = atan2(py, px); // convert prediction to an angle
  float velocity_dash = (px * vx + py * vy) / range_dash; // convert prediction to a velocity

  VectorXd hx_dash = VectorXd(3);
  hx_dash << range_dash, angle_dash, velocity_dash;
  
  Hj_ = MatrixXd(3,4);
  Hj_ = tools.CalculateJacobian(x_);
  
  VectorXd y = z - hx_dash;
  MatrixXd Hjt = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Hjt + R_;
  MatrixXd K = (P_ * Hjt) * S.inverse();
  cout << "K " << K << endl;
  
  // Update the mean and covarience with the measured data
  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_) * P_;
}
