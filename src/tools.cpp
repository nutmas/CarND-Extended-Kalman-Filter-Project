#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  // RMSE to calculate performance of Kalman filter against ground truth
  // create and intitalise rmse vector
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check: input estimation is not zero
  // check: estimation vector and ground truth vector are equal in size
  if(estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "Invalid estimation or ground truth data" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i) {

    // temporary vector to get difference between est and gt
    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;

  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate sqaure root
  rmse = rmse.array().sqrt();

  // return result
  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  // Calculate a Jacobian

  // initialise 3 row x 4 column matrix

  // measurement matrix for the radar - 3 measurements rho, phi, rhodot
  MatrixXd Hj(3,4);
  Hj << 0,0,0,0,
        0,0,0,0,
        0,0,0,0;
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if(fabs(c1) < 0.0001) {
    cout << "CalculateJacobian() - error - Division by zero" << endl;
    return Hj;
  }

  // compute Jacobian Matrix
  Hj << (px/c2), (py/c2), 0, 0,
        -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;

}
