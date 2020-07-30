#include "cam/sparse_optical_flow_gpu.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sparse_optical_flow_gpu");
    ros::NodeHandle nh;
    ros::start();
    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera_BGR", 1);
    img_sub.registerCallback(image_callback);
    ros::spin();
    ros::shutdown();
}

void image_callback(const sensor_msgs::ImageConstPtr &img)
{
  std::vector<cv::KeyPoint> kp1;
  cv::Ptr<cv::GFTTDetector> detector =cv::GFTTDetector::create(500, 0.01, 20);
  cur_frame = cv_bridge::toCvShare(img, "mono8")->image;
  detector->detect(cur_frame, kp1);
  std::vector<uchar> status;
  std::vector<float> error;
  cv::Mat img_show;
  cv::cvtColor(cur_frame, img_show, CV_GRAY2BGR);

  if(has_got_frame)
  {
    cv::calcOpticalFlowPyrLK(prev_frame, cur_frame, prev_pts, cur_pts, status, error, cv::Size(8, 8));

    for (int i = 0; i < cur_pts.size(); i++) {
        if (status[i]) {
            cv::circle(img_show, cur_pts[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img_show, prev_pts[i], cur_pts[i], cv::Scalar(0, 250, 0));
        }
    }
  }
  prev_pts.clear();
  for(auto &kp: kp1)
  {
    prev_pts.push_back(kp.pt);
  }

  prev_frame = cur_frame;
  has_got_frame = true;
  std::cout<<prev_pts.size()<<std::endl;
  cv::imshow("lalala", img_show);
  cv::waitKey(1);
}
