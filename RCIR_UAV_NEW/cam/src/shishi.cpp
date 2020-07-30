#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <cam/vision_msg.h>
#include <cam/vision_msgs.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

using namespace cv;
using namespace std;

int row, col;
int threshold_value = 15;
const string T_value = "threshold_value";
const string window_name = "yuchulihou";
Mat dstImage, edge, threshold_output;
int g_nKernelSize=20;//核大小
darknet_ros_msgs::BoundingBoxes obstacle;
ros::Publisher obstacle_pub;
image_transport::Publisher obstacle_image;
cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >();
cv::Rect rect1, rect2;

int clusterCount = 4;

void image_callback(const sensor_msgs::ImageConstPtr &img);
void detect(Mat img);
void Threshold_Demo( int, void* );
Mat K_means(Mat img);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "shishi");
    ros::NodeHandle nh;
    obstacle_pub = nh.advertise<darknet_ros_msgs::BoundingBoxes>("/obstacle/msg", 1);

    image_transport::ImageTransport it( nh );
    obstacle_image = it.advertise( "/obstacle/img", 1 );

    frame->encoding = sensor_msgs::image_encodings::BGR8;

    ros::start();
    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera_BGR", 1);
    img_sub.registerCallback(image_callback);
    ros::spin();
    ros::shutdown();
}

void image_callback(const sensor_msgs::ImageConstPtr &img)
{
  Mat resive_im = cv_bridge::toCvShare(img, "bgr8")->image;
  row = resive_im.rows;
  col = resive_im.cols;
  detect(resive_im);
}

void detect(Mat img)
{
  //
  Mat img_cluster = K_means(img);
  imshow("jullei", img_cluster);
  //Mat img_src = img;
  //cvtColor(img, img, CV_BGR2GRAY);
  //equalizeHist(img, img);
  //imshow("lalala", img);
  //GaussianBlur(img,dstImage,Size(5,5),0,0);
  //imshow("lblblb", dstImage);
  //Canny(dstImage, edge, 3, 9, 3);
  //imshow("bianyuan", edge);

//  namedWindow(window_name, WINDOW_AUTOSIZE);
//  createTrackbar( T_value, window_name, &threshold_value, 255, Threshold_Demo );
//  Threshold_Demo(0, 0);

//  Mat element_d = getStructuringElement(MORPH_RECT, Size(g_nKernelSize*2+1,g_nKernelSize*2+1),Point(g_nKernelSize,g_nKernelSize));
//  Mat element_e = getStructuringElement(MORPH_RECT, Size((g_nKernelSize-15)*2+1,(g_nKernelSize-15)*2+1),Point(g_nKernelSize-15,g_nKernelSize-15));

//  Mat fushi;
//  dilate(threshold_output, fushi, element_d);//进行腐蚀操作
//  erode(fushi, fushi, element_e);
  //imshow("fushi", fushi);//显示效果图

//  Canny(fushi, edge, 3, 9, 3);
//  //dilate(edge, edge, element_e2);
//  imshow("lclclc", edge);

//  addWeighted(polyPic, 0.5, img, 0.5, 0, img);

  waitKey(1);
}

void Threshold_Demo( int, void* )
{

    threshold( dstImage, threshold_output, threshold_value, 255, THRESH_BINARY );

    imshow( window_name, threshold_output );
}

Mat K_means(Mat img)
{
  Mat samples(img.cols*img.rows, 1, CV_32FC3);
  Mat labels(img.cols*img.rows, 1, CV_32SC1);

  uchar* p;
  int i, j, k=0;
  for(i=0; i < img.rows; i++)
  {
       p = img.ptr<uchar>(i);
       for(j=0; j< img.cols; j++)
       {
          samples.at<Vec3f>(k,0)[0] = float(p[j*3]);
          samples.at<Vec3f>(k,0)[1] = float(p[j*3+1]);
          samples.at<Vec3f>(k,0)[2] = float(p[j*3+2]);
          k++;
       }
  }
  Mat centers(clusterCount, 1, samples.type());
  kmeans(samples, clusterCount, labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);
  //我们已知有3个聚类，用不同的灰度层表示。
  Mat img1(img.rows, img.cols, CV_8UC1);
  float step=255/(clusterCount - 1);
  k=0;
  for(i=0; i < img1.rows; i++)
  {
      p = img1.ptr<uchar>(i);
      for(j=0; j< img1.cols; j++)
      {
          int tt = labels.at<int>(k, 0);
          k++;
          p[j] = 255 - tt*step;
      }
  }
  return img1;
}
