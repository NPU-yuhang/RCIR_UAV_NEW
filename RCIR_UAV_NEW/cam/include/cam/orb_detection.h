#ifndef ORB_DETECTION_H
#define ORB_DETECTION_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <iostream>
#include <ctype.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <cv.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cam/ClusterAnalysis.h"

using namespace std;
using namespace cv;

Mat cur_frame, pre_frame;
Mat Samples, Labels;
int clusterCount = 2;
Mat centers;

image_transport::Publisher pub_cluster_image;
cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >();

ros::Publisher cloud_pub;
pcl::PointCloud<pcl::PointXYZRGB> cloud;
sensor_msgs::PointCloud2 output;
ClusterAnalysis myClusterAnalysis;

bool has_got_first_frame = false;
bool has_got_cluster = false;

std::vector<KeyPoint> keypoints_1, keypoints_2;
std::vector<Point2f> kps_1, kps_2;
Mat descriptors_1, descriptors_2;
Ptr<ORB> orb = ORB::create();
std::vector<pair<int, Point3d>> center_pts;
int nearest_cluster;
std::vector<pair<int, pair<Point2f, Point2f>>> cluster_rects;
//GoodFeaturesToTrackDetector Detector(500);
std::map<int, int> kp_frames;
std::vector<pair<int, int>> point_cluster;
std::vector<pair<int, Point3d>> point_cluster_3d;

vector<uchar> status;
Size subPixWinSize(10,10), winSize(31,31);
TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);

void image_callback(const sensor_msgs::ImageConstPtr &img);

void pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector< DMatch > matches,
                            Mat& R, Mat& t );
void triangulation (
    const vector< KeyPoint >& keypoint_1,
    const vector< KeyPoint >& keypoint_2,
    const std::vector< DMatch >& matches,
    const Mat& R, const Mat& t,
    vector< Point3d >& points );
Point2f pixel2cam ( const Point2d& p, const Mat& K );

#endif // ORB_DETECTION_H
