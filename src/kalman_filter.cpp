#include "kalman_filter.h"

#define EPS 0.0001

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_ ;
}

void KalmanFilter::Update(const VectorXd &z) {
  // update the state by using Kalman Filter equations

  if(fabs(x_(0))<EPS && fabs(x_(1))<EPS){
    x_(0) = EPS;
    x_(1) = EPS;
  }


  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // update the state by using Extended Kalman Filter equations

  if(fabs(x_(0)) < EPS && fabs(x_(1)) < EPS){
    x_(0) = EPS;
    x_(1) = EPS;
  }

  //x_(0) = (x_(0)<= EPS) ? EPS : x_(0);

  VectorXd z_pred = VectorXd(3);
  float px=x_(0);
  float py=x_(1);
  float vx=x_(2);
  float vy=x_(3);

  z_pred(0) = sqrt(px*px+py*py); 
  z_pred(1) = atan2(py,px);  // px is greater than EPS
  z_pred(2) = (px*vx+py*vy)/z_pred(0); // z_pred(0) is greater than 0
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
