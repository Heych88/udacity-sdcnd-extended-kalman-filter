#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
  
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.H_ = MatrixXd(2, 4);
  ekf_.H_ << 1, 0, 0, 0,
             0, 1, 0, 0;
  ekf_.Hj_ = MatrixXd(3, 4);

  previous_timestamp_ = 0.0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    return;
  }
    
  short noise_ax = 9;
  short noise_ay = 9;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // first measurement
    cout << endl;
    cout << "INIT: " << measurement_pack.raw_measurements_[0] << endl;
    cout << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << measurement_pack.raw_measurements_[0], 
               measurement_pack.raw_measurements_[0],
               0,
               0;
    ekf_.P_ = MatrixXd(4,4);
    // Init covariance matrix to be large so as to rely on the measured data to 
    // start with
    ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000; 

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      cout << "I am the first radar" << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */      
      ekf_.R_ = R_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  //dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; 
  previous_timestamp_ = measurement_pack.timestamp_;
  
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  float dt_2 = dt * dt;
  float dt_4_4 = (dt_2 * dt_2) / 4; 
  float dt_3_2 = (dt_2 * dt) / 2;
  
  ekf_.Q_ << dt_4_4 * noise_ax, 0, dt_3_2 * noise_ax, 0,
             0, dt_4_4 * noise_ay, 0, dt_3_2 * noise_ay,
             dt_3_2 * noise_ax, 0, dt_2 * noise_ax, 0,
             0, dt_3_2 * noise_ay, 0, dt_2 * noise_ay;

  //cout << "Q_ " << ekf_.Q_ << endl;
  //cout << "F_ " <<  ekf_.F_ << endl;
  //cout << "H_ " << ekf_.H_ << endl;
  //cout << "R_ " << ekf_.R_ << endl;
  
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    //cout << "update radar" << endl;
  } else {
    // Laser updates
    //cout << "update laser" << endl;
    ekf_.R_ = R_laser_;
    cout << "measurement_pack.raw_measurements_ " << measurement_pack.raw_measurements_ << endl;
    ekf_.Update(measurement_pack.raw_measurements_);
    
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
