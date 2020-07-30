#ifndef EKF_FUSION_H
#define EKF_FUSION_H
//system
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include "cam/fusion_ekf/measurement_package.h"
#include "cam/fusion_ekf/ekf_filter.h"
#include <Eigen/Dense>

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();
  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  EKF_Fusion ekf_;

private:
  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  Eigen::MatrixXd R_camera_;    // camera measurement noise
  Eigen::MatrixXd R_radar_;    // radar measurement noise
  Eigen::MatrixXd H_radar_;    // measurement function for radar
  Eigen::MatrixXd H_jacobian;         // measurement function for camera

  float noise_ax;
  float noise_ay;
  float noise_az;
};

#endif // EKF_FUSION_H
