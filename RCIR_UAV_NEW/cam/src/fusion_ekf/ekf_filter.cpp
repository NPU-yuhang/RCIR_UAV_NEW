#include "cam/fusion_ekf/ekf_filter.h"

const float PI2 = 2 * M_PI;

EKF_Fusion::EKF_Fusion() {}

void EKF_Fusion::Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
                        Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void EKF_Fusion::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  Eigen::MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void EKF_Fusion::Update(const Eigen::VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  Eigen::VectorXd z_pred = H_ * x_;

  Eigen::VectorXd y = z - z_pred;
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd PHt = P_ * Ht;
  Eigen::MatrixXd S = H_ * PHt + R_;
  //std::cout<<"S_radar: "<<S<<std::endl;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd K = PHt * Si;
  //std::cout<<"K_radar: "<<K<<std::endl;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  //std::cout<<"P_radar: "<<P_<<std::endl;

}

Eigen::VectorXd EKF_Fusion::camera_factor(const Eigen::VectorXd &x_state)
{
    Eigen::VectorXd x_r(4,1);
    x_r[0] = x_state[0];
    x_r[1] = x_state[1];
    x_r[2] = x_state[2];
    x_r[3] = 1;
    Eigen::VectorXd x_cam = T_*x_r;
    //std::cout<<"x: "<<x_cam[0]<<"y: "<<x_cam[1]<<"z: "<<x_cam[2]<<std::endl;
    float scale = x_cam[2];
    Eigen::VectorXd x_pix = (K_*x_cam)/scale;
    Eigen::VectorXd uv(2,1);
    uv[0] = x_pix[0];
    uv[1] = x_pix[1];
    return uv;
}

Eigen::MatrixXd EKF_Fusion::CalculateJacobian(const Eigen::VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
  */
  Eigen::MatrixXd Hj(2,5);
  Hj << 0,0,0,0,0,
        0,0,0,0,0;

  //recover state parameters
  Eigen::VectorXd x_r(4,1);
  x_r[0] = x_state[0];
  x_r[1] = x_state[1];
  x_r[2] = x_state[2];
  x_r[3] = 1;
  Eigen::VectorXd x_cam = T_*x_r;

  if(x_cam[2] < 0.0001)
  {
      std::cout << "Function CalculateJacobian() has Error: Division by Zero" << std::endl;
      return Hj;
  }

  //compute the Jacobian matrix
  Eigen::MatrixXd Hi(2,3);
  Hi << -(K_(0,0)/x_cam[2]), 0, K_(0,0)*x_cam[0]/(x_cam[2]*x_cam[2]),
        0,  -(K_(1,1)/x_cam[2]), K_(1,1)*x_cam[1]/(x_cam[2]*x_cam[2]);
  Hi = Hi*T_.block(0,0,3,3);
  Hj << Hi(0,0), Hi(0,1), Hi(0,2), 0, 0,
        Hi(1,0), Hi(1,1), Hi(1,2), 0, 0;
  return Hj;

}

void EKF_Fusion::UpdateEKF(const Eigen::VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */

  Eigen::VectorXd z_pred = camera_factor(x_);
  Eigen::VectorXd y = z - z_pred;
  // following is exact the same as in the function of KalmanFilter::Update()
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd PHt = P_ * Ht;
  Eigen::MatrixXd S = H_ * PHt + R_;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ - (K * y);
  long x_size = x_.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
