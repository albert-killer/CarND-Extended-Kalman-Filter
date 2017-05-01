#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * (Default) Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
   R_laser_ << 0.0225, 0,
        0, 0.0225;
  /*
  // experimenting with R
  R_laser_ << 0.0015, 0,
        0, 0.0020;
  */

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /*
  // experimenting with R
  R_radar_ << 0.5, 0, 0,
        0, 0.002, 0,
        0, 0, 2.5;
  */

  //measurement matrix laser
  H_laser_ << 1, 0, 0, 0,
        0, 1, 0, 0;

  //(State) Covariance matrix
  ekf_.P_ =  MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  // Set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Converting from polar to cartersian coordinates:
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];
      // x = r * cos(phi) and y  = r * sin(phi)
      // float px = rho * cos(phi);
      // float py = rho * sin(phi);
      // Alternative:
      float px = sqrt(rho*rho / (1 + tan(phi)*tan(phi)));
      float py = tan(phi) * px;
      float vx = 0; // phi can only be used for px and py
      float vy = 0;
      // Initialize state on basis of radar measurement
      ekf_.x_ << px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state on basis of lidar measurement
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    }
    // Taking time stamp
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Compute time elapsed between current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  // Integrate time dt into F matrix
  ekf_.F_ << 1, 0, dt, 0,
          0, 1, 0, dt,
          0, 0, 1, 0,
          0, 0, 0, 1;

  // Set Process Covariance matrix Q
  float t4 = pow(dt,4.0) / 4.0;
  float t3 = pow(dt,3.0) / 2.0;
  float t2 = pow(dt,2.0);
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << t4*noise_ax, 0, t3*noise_ax, 0,
          0, t4*noise_ay, 0, t3*noise_ay,
          t3*noise_ax, 0, t2*noise_ax, 0,
          0, t3*noise_ay, 0, t2*noise_ay;

  // call Kalman FIlter predict function
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  // Radar updates
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    Tools littlehelper;
    Hj_= littlehelper.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  // Laser updates
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // Print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
