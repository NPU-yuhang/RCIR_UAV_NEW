#ifndef EKF_RADAR_CAMERA_FUSION_H
#define EKF_RADAR_CAMERA_FUSION_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "cam/fusion_ekf/ekf_fusion.h"
#include "cam/fusion_ekf/ground_truth_package.h"
#include "cam/fusion_ekf/measurement_package.h"
#include "cam/ekf_data.h"
#include "cam/ekf.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/LaserScan.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

MeasurementPackage radar_measure;
MeasurementPackage camera_measure;
FusionEKF ekf_obstacles;
std::ofstream location_out;
std::ofstream raw_data_out;

void ekf_measurement_callback(const cam::ekfConstPtr &ekf_m);

#endif // EKF_RADAR_CAMERA_FUSION_H
