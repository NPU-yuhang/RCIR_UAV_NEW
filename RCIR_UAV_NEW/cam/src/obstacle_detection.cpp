#include "cam/obstacle_detection.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_detection");
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
  Mat img_src = img;
  cvtColor(img, img, CV_BGR2GRAY);
  equalizeHist(img, img);
  //imshow("lalala", img);
  GaussianBlur(img,dstImage,Size(5,5),0,0);
  //imshow("lblblb", dstImage);

  namedWindow(window_name, WINDOW_AUTOSIZE);
  createTrackbar( T_value, window_name, &threshold_value, 255, Threshold_Demo );
  Threshold_Demo(0, 0);

  Mat element_d = getStructuringElement(MORPH_RECT, Size(g_nKernelSize*2+1,g_nKernelSize*2+1),Point(g_nKernelSize,g_nKernelSize));
  Mat element_e = getStructuringElement(MORPH_RECT, Size((g_nKernelSize-15)*2+1,(g_nKernelSize-15)*2+1),Point(g_nKernelSize-15,g_nKernelSize-15));
  Mat element_e2 = getStructuringElement(MORPH_RECT, Size((g_nKernelSize-15)*2+1,(g_nKernelSize-15)*2+1),Point(g_nKernelSize-18,g_nKernelSize-18));

  Mat fushi;
  dilate(threshold_output, fushi, element_d);//进行腐蚀操作
  erode(fushi, fushi, element_e);
  //imshow("fushi", fushi);//显示效果图

  Canny(fushi, edge, 3, 9, 3);
  dilate(edge, edge, element_e2);
  //imshow("lclclc", edge);

  vector<vector<Point>> contours;    //储存轮廓
  vector<Vec4i> hierarchy;
  findContours(edge, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);    //获取轮廓
  Mat linePic = Mat::zeros(edge.rows, edge.cols, CV_8UC3);
  std::cout<<"contours.size: "<<contours.size()<<std::endl;

  if(contours.size()>1)
  {
    for (int index = 0; index < contours.size(); index++){
            drawContours(linePic, contours, index, Scalar(rand() & 255, rand() & 255, rand() & 255), 1, 8/*, hierarchy*/);
    }

    vector<vector<Point>> polyContours(contours.size()), sepolyContours(contours.size());
    int maxArea = 0;
    int semaxArea = 0;
    for (int index = 0; index < contours.size(); index++){
            if (contourArea(contours[index]) > contourArea(contours[maxArea]))
            {
                semaxArea = maxArea;
                maxArea = index;
            }
            approxPolyDP(contours[index], polyContours[index], 10, true);
            approxPolyDP(contours[index], sepolyContours[index], 10, true);
    }
    if (maxArea == 0 && contours.size() > 1)
    {
      semaxArea = 1;
      for (int index = 1; index < contours.size(); index++)
      {
        if (contourArea(contours[index]) > contourArea(contours[semaxArea]))
        {
            semaxArea = index;
        }
        approxPolyDP(contours[index], sepolyContours[index], 10, true);
      }
    }

    std::cout<<maxArea<<"---"<<semaxArea<<std::endl;
    rect1 = cv::boundingRect(polyContours[maxArea]);
    rect2 = cv::boundingRect(sepolyContours[semaxArea]);
    cv::rectangle(img_src, rect1, Scalar(0, 0, 255));
    cv::rectangle(img_src, rect2, Scalar(0, 0, 255));
    vector<Moments> mu(2);
    mu[0] = moments(polyContours[maxArea], false);
    mu[1] = moments(sepolyContours[semaxArea], false);

    ///  计算中心矩:
    vector<Point2f> mc(2);
    mc[0] = Point2f(mu[0].m10 / mu[0].m00, mu[0].m01 / mu[0].m00);
    mc[1] = Point2f(mu[1].m10 / mu[1].m00, mu[1].m01 / mu[1].m00);

    Mat polyPic = Mat::zeros(img.size(), CV_8UC3);
    circle(img_src, mc[0], 4, Scalar(255), -1, 8, 0);
    circle(img_src, mc[1], 4, Scalar(255), -1, 8, 0);

    obstacle.header.stamp = ros::Time::now();
    obstacle.bounding_boxes.resize(2);
    obstacle.bounding_boxes[0].xmin = rect1.x;
    obstacle.bounding_boxes[0].xmax = rect1.x+rect1.width;
    obstacle.bounding_boxes[0].ymin = rect1.y;
    obstacle.bounding_boxes[0].ymax = rect1.y+rect1.height;
    obstacle.bounding_boxes[0].rows = row;
    obstacle.bounding_boxes[0].cols = col;

    obstacle.bounding_boxes[1].xmin = rect2.x;
    obstacle.bounding_boxes[1].xmax = rect2.x+rect2.width;
    obstacle.bounding_boxes[1].ymin = rect2.y;
    obstacle.bounding_boxes[1].ymax = rect2.y+rect2.height;
    obstacle.bounding_boxes[1].rows = row;
    obstacle.bounding_boxes[1].cols = col;
    obstacle_pub.publish(obstacle);

    frame->image = img_src;
    frame->header.stamp = ros::Time::now();
    obstacle_image.publish(frame->toImageMsg());
    //imshow("lalala", img_src);
  }

//  addWeighted(polyPic, 0.5, img, 0.5, 0, img);

  waitKey(1);
}

void Threshold_Demo( int, void* )
{

    threshold( dstImage, threshold_output, threshold_value, 255, THRESH_BINARY );

    imshow( window_name, threshold_output );
}

void K_means()
{
  Mat samples(col*row, 1, CV_32FC3);
  Mat labels(col*row, 1, CV_32SC1);
}
