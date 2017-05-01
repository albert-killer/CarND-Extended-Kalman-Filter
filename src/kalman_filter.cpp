#include "kalman_filter.h"
#define PI 3.14159265
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
  // Predict the state
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

  // Update the state by using Kalman Filter equations
  // --> here used for LASER measurements!
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  // New estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  // Update  state by using Extended Kalman Filter equations
  // --> here used for RADAR measurements!

  // Recover prediction state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float phi = 0.0;
  float rho_dot = 0.0;

  // Convert vector x_ to polar coordinates:
  float rho = sqrt(px*px + py*py);
  if (fabs(rho) < 0.0001) {
    rho = 0.0001;
  }

  // Avoid division by zero
  if(fabs(px) < 0.0001){
    cout << "Error while converting vector x_ to polar coordinates: Division by Zero" << endl;
  }
  // TODO: Maybe add condition for atan2(0,0) ?
  else {
    phi = atan2(py, px);
    // No need to normalize angle at this point
    // because atan2 spits out radians between pi and -pi already
  }

  // Avoid division by zero
  if (rho < 0.0001) {
    cout << "Error while converting vector x_ to polar coordinates: Division by Zero" << endl;
  }
  else {
    rho_dot = (px*vx + py*vy) / rho;
  }

  // For radar H * x becomes h(x)
  VectorXd h = VectorXd(3);
  h << rho, phi, rho_dot;
  VectorXd y = z - h; // Using h instead of Jacobian Hj_ here!

  // Apply angle normalization AFTER comparing prediction with sensor data y = z - h(x)



  while (y(1)>PI) {
    y(1) -= 2 * PI;
  }
  while (y(1)<-PI) {
    y(1) += 2 * PI;
  }
  // Alternative:
  // this->NormalizeAngle(y(1));

  MatrixXd Ht = H_.transpose(); // Using Jacobian Hj_ here
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_; // Using Jacobian Hj_ here
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si; // Using Jacobian Hj_ here

  // New estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_; // Using Jacobian Hj_ here

}

/*
float KalmanFilter::NormalizeAngle(float pValue)
{
  if (fabs(pValue) > M_PI)
  {
    pValue -= round(pValue / (2.0 * M_PI)) * (2.0 * M_PI);
    return pValue;
  }
}
*/

