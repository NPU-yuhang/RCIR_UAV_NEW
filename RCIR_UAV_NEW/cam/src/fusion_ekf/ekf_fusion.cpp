#include "cam/fusion_ekf/ekf_fusion.h"

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_camera_ = Eigen::MatrixXd(2, 2);
  R_radar_ = Eigen::MatrixXd(4, 4);
  H_radar_ = Eigen::MatrixXd(4, 5);
  H_jacobian = Eigen::MatrixXd(2, 5);

  //measurement covariance matrix - radar
  R_radar_ << 0.1, 0, 0, 0,
              0, 0.1, 0, 0,
              0, 0, 0.01, 0,
              0, 0, 0, 0.01;

  R_camera_ << 10, 0,
               0, 10;
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  H_radar_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0,
              0, 0, 0, 1, 0,
              0, 0, 0, 0, 1;

  // initialize the kalman filter variables
  ekf_.P_ = Eigen::MatrixXd(5, 5);
  ekf_.P_ << 1, 0, 0, 0, 0,
             0, 1, 0, 0, 0,
             0, 0, 1, 0, 0,
             0, 0, 0, 1, 0,
             0, 0, 0, 0, 1;

  ekf_.F_ = Eigen::MatrixXd(5, 5);
  ekf_.F_ << 1, 0, 0, 1, 0,
             0, 1, 0, 0, 1,
             0, 0, 1, 0, 0,
             0, 0, 0, 1, 0,
             0, 0, 0, 0, 1;
  ekf_.K_ = Eigen::MatrixXd(4, 4);
  ekf_.K_ << 630.489651, 0, 400.735172, 0,
             0, 630.434369, 308.768174, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;

  float pitch = 91.5*(M_PI*1.0/180.0);
  float yaw = -1.0*(M_PI*1.0/180.0);
  float roll = 0*(M_PI*1.0/180.0);

  Eigen::Matrix3d Rx, Ry, Rz, R;
  Rx << 1,0,0,
        0,std::cos(pitch), -std::sin(pitch),
        0,std::sin(pitch), std::cos(pitch);
  Ry << std::cos(yaw),0,std::sin(yaw),
        0,1,0,
        -std::sin(yaw),0,std::cos(yaw);
  Rz << std::cos(roll),-std::sin(roll), 0,
        std::sin(roll),std::cos(roll),0,
        0,0,1;

  R = Rz*Ry*Rx;
  ekf_.T_ = Eigen::MatrixXd(4, 4);
  ekf_.T_ << R(0,0),R(0,1),R(0,2),0,
        R(1,0),R(1,1),R(1,2),0,
        R(2,0),R(2,1),R(2,2),0,
        0,0,0,1;
  // set measurement noises
  noise_ax = 0.09;
  noise_ay = 0.09;
  noise_az = 0.09;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    // first measurement
    ekf_.x_ = Eigen::VectorXd(5);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0,
                 measurement_pack.raw_measurements_[2], measurement_pack.raw_measurements_[3];  // x, y, vx, vy

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::CAMERA) {
      /**
      Initialize state.
      */
      Eigen::Vector4d x_pix;
      x_pix << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 1, 1;
      ekf_.x_ = ekf_.T_.inverse()*ekf_.K_.inverse()*x_pix;
      //ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0, 0; // x, y, vx, vy
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000000.0;  //  in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 3) = dt;
  ekf_.F_(1, 4) = dt;
  //set the process covariance matrix Q
  ekf_.Q_ = Eigen::MatrixXd(5, 5);
  ekf_.Q_ << dt_4/4*noise_ax,   0,                0,                dt_3/2*noise_ax,  0,
             0,                 dt_4/4*noise_ay,  0,                0,                dt_3/2*noise_ay,
             0,                 0,                dt_4/4*noise_az,  0,                0,
             dt_3/2*noise_ax,   0,                0,                dt_2*noise_ax,    0,
             0,                 dt_3/2*noise_ay,  0,                0,                dt_2*noise_ay;
  ekf_.Predict();
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::CAMERA) {
    // camera updates
    H_jacobian = ekf_.CalculateJacobian(ekf_.x_);
    ekf_.H_ = H_jacobian;
    ekf_.R_ = R_camera_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else {
    // Laser updates
    ekf_.H_ = H_radar_;
    ekf_.R_ = R_radar_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
//  cout << "x_ = " << ekf_.x_ << endl;
//  cout << "P_ = " << ekf_.P_ << endl;
}

