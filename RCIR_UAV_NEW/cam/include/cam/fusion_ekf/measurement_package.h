#ifndef MEASUREMENT_PACKAGE_H
#define MEASUREMENT_PACKAGE_H
#include "Eigen/Dense"

class MeasurementPackage {
public:
  long long timestamp_;

  enum SensorType{
    CAMERA,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;
};
#endif // MEASUREMENT_PACKAGE_H
