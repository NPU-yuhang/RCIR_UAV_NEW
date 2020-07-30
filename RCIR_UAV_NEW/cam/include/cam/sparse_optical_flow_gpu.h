#ifndef SPARSE_OPTICAL_FLOW_GPU_H
#define SPARSE_OPTICAL_FLOW_GPU_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

cv::Mat cur_frame, prev_frame;
std::vector<cv::Point2f> cur_pts, prev_pts;
//std::vector<cv::KeyPoint> kp1_multi;
bool has_got_frame = false;

void image_callback(const sensor_msgs::ImageConstPtr &img);

#endif // SPARSE_OPTICAL_FLOW_GPU_H
