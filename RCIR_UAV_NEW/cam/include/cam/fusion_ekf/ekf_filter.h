#ifndef EKF_FILTER_H
#define EKF_FILTER_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>

class EKF_Fusion
{
public:
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  // state transistion matrix
  Eigen::MatrixXd F_;
  // process covariance matrix
  Eigen::MatrixXd Q_;
  // measurement matrix
  Eigen::MatrixXd H_;
  // measurement covariance matrix
  Eigen::MatrixXd R_;
  //camera K
  Eigen::Matrix4d K_;
  //radar tansform to camera T
  Eigen::Matrix4d T_;

  EKF_Fusion();

  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  void Predict();

  void Update(const Eigen::VectorXd &z);

  Eigen::VectorXd camera_factor(const Eigen::VectorXd &x_state);

  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  void UpdateEKF(const Eigen::VectorXd &z);

};

#endif // EKF_FILTER_H
