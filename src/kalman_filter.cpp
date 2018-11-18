#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

// constructor
KalmanFilter::KalmanFilter() {}

// destructor
KalmanFilter::~KalmanFilter() {}

// initialisation
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

// prediction function
void KalmanFilter::Predict() {

  // predict the state
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

// update function for laser
void KalmanFilter::Update(const VectorXd &z) {

  // update the state by using Kalman Filter equations

  // error calculation
  VectorXd z_pred = H_ * x_;

  VectorXd y = z - z_pred;

  // apply Kalman Filter
  ApplyKalmanFilter(y);

}

// update function for Radar
void KalmanFilter::UpdateEKF(const VectorXd &z) {

  // update the state by using Extended Kalman Filter equations

  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  // convert from cartestian to polar coordinates
  float rho = sqrt(px*px + py*py);
  float phi = atan2(py,px);

  // avoid division by 0
  if (fabs(rho) < 0.0001) {
    rho = 0.0001;

  }

  float rhodot = (px*vx + py*vy)/rho;

  VectorXd z_pred(3);
  z_pred << rho, phi, rhodot;

  VectorXd y = z - z_pred;


  // modify the angle if not between -PI to PI
  while(y(1) > M_PI){
    y(1) -= 2*M_PI;
  }

  while(y(1) < -M_PI){
    y(1) += 2*M_PI;
  }

  // apply Kalman Filter
  ApplyKalmanFilter(y);

}

// Kalman Filter core
void KalmanFilter::ApplyKalmanFilter(const VectorXd &y) {

  // Kalman filter
  MatrixXd Ht = H_.transpose();

  MatrixXd PHt = P_ * Ht;

  MatrixXd S = H_ * PHt +R_;

  MatrixXd Si = S.inverse();

  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

