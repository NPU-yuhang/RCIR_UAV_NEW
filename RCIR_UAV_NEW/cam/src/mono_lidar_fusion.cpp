#include <cam/mono_lidar_fusion.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "caitu");
  ros::NodeHandle nh;

  if(argc != 2)
  {
      std::cerr << std::endl << "no config file" << std::endl;
      ros::shutdown();
      return 1;
  }
  fusion_param.readparam(argv[1]);
  setCameraParam(fusion_param);
  obstacle_info_pub = nh.advertise<cam::mono_radar_fusion>("/mono_radar/obstacle/msg", 10);
  ekf_measure_pub = nh.advertise<cam::ekf>("/ekf/measurement/msg", 10);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, my_radar::objects, darknet_ros_msgs::BoundingBoxes> topic_synchronizer;
  message_filters::Subscriber<sensor_msgs::Image>* img_sub;
  message_filters::Subscriber<my_radar::objects>* radar_sub;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>* obstacke_sub;

  message_filters::Synchronizer<topic_synchronizer>* sync;
  img_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, fusion_param.image_msg, 100);
  //scan_sub = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, "/scan", 100);
  radar_sub = new message_filters::Subscriber<my_radar::objects>(nh, fusion_param.radar_mod, 100);
  obstacke_sub = new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(nh, fusion_param.obsta_msg, 100);

  sync = new message_filters::Synchronizer<topic_synchronizer>(topic_synchronizer(10), *img_sub, *radar_sub, *obstacke_sub);
  sync->registerCallback(boost::bind(&FusionCallback, _1, _2, _3));

  ros::spin();
  ros::shutdown();
  return 0;
}

void FusionCallback(const sensor_msgs::ImageConstPtr &img, const my_radar::objectsConstPtr &radar, const darknet_ros_msgs::BoundingBoxesConstPtr &obstacle)
{
  std::vector<cv::Point> obsta;
  int obsta_count = obstacle->bounding_boxes.size();
  for(int i = 0; i<obsta_count; i++)
  {
    cv::Point P;
    //std::cout<<obstacle->bounding_boxes[i].ymax - obstacle->bounding_boxes[i].ymin<<std::endl;
    if((obstacle->bounding_boxes[i].ymax - obstacle->bounding_boxes[i].ymin)>100)
    {
      P.x = (obstacle->bounding_boxes[i].xmin+obstacle->bounding_boxes[i].xmax)/2;
      P.y = (obstacle->bounding_boxes[i].ymin+obstacle->bounding_boxes[i].ymax)/2;
      obsta.push_back(P);
    }
  }

  obsta_count = obsta.size();

  int count = radar->objs.size();
  if(count)
  {
    cv::Mat radar_data = cv::Mat::zeros(4,count,CV_32FC1);
    for(int i=0; i<count; i++)
    {
      if(radar->objs[i].Long < 7)
      {
        radar_data.at<float>(0,i) = -(radar->objs[i].Lat);
        radar_data.at<float>(1,i) = radar->objs[i].Long;
        radar_data.at<float>(2,i) = -(radar->objs[i].velo_Lat);
        radar_data.at<float>(3,i) = radar->objs[i].velo_Long;
      }
    }
    cv::Mat point = radar2image.TransformWRadarPoint2ImagePoint(radar_data.rowRange(0,2));
    cv::Mat p_camera = radar2image.TransformWRadar2Camera(radar_data);

    cv::Mat img_cam = cv_bridge::toCvShare(img, "bgr8")->image;
    std::vector<cv::Point> lidar_pts;
    //lidar_pts.resize(65);

    cv::Mat lidar_obsta = cv::Mat::zeros(obsta_count, 1, CV_32FC1);
    cv::Mat min_dis = cv::Mat::ones(obsta_count, 1, CV_32FC1);
    min_dis = 800*min_dis;

    for(int i = 0; i<count; i++)
    {
      cv::Point lidar_pt;
      //std::cout<<point.at<float>(0, i)<<"  "<<point.at<float>(1, i)<<"  "<<std::endl;
      if(point.at<float>(0, i) <= 800 && point.at<float>(0, i) >=1 && point.at<float>(1, i) <= 600 && point.at<float>(1, i) >=1)
      {
        lidar_pt.x = (int)point.at<float>(0, i);
        lidar_pt.y = (int)point.at<float>(1, i);
        cv::circle(img_cam, lidar_pt, 4, cv::Scalar(0, 0, 255));

        for(int j = 0; j<obsta_count; j++)
        {
          if(abs(lidar_pt.x - obsta[j].x) < min_dis.at<int>(j, 0))
          {
            lidar_obsta.at<int>(j, 0) = i;
            min_dis.at<int>(j, 0) = abs(lidar_pt.x-obsta[j].x);
          }
        }
      }
      lidar_pts.push_back(lidar_pt);
    }
    cam::mono_radar_fusion obs_info;
    obs_info.header.stamp = ros::Time::now();
    cam::ekf ekf_m;
    ekf_m.header.stamp = ros::Time::now();
    for(int i=0; i<obsta_count; i++)
    {
      std::string ss = std::to_string(p_camera.at<float>(0, lidar_obsta.at<int>(i, 0))) + ", " +
          std::to_string(p_camera.at<float>(1, lidar_obsta.at<int>(i, 0))) + ", " +
          std::to_string(p_camera.at<float>(2, lidar_obsta.at<int>(i, 0)));
      cv::putText(img_cam,ss,cv::Point(150, 40*(i+1)),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(255,23,0),2,4);
      cv::circle(img_cam, lidar_pts[lidar_obsta.at<int>(i, 0)], 6, cv::Scalar(0, 255, 0));
      cam::point obs_point;
      cam::rect obs_rect;
      obs_point.x = p_camera.at<float>(0, lidar_obsta.at<int>(i, 0));
      obs_point.y = p_camera.at<float>(1, lidar_obsta.at<int>(i, 0));
      obs_point.z = p_camera.at<float>(2, lidar_obsta.at<int>(i, 0));

//      double l = (obs_point.z * (obstacle->bounding_boxes[i].xmax - obstacle->bounding_boxes[i].xmin))/(camParam.fu);
//      //std::cout<<"l: "<<l<<std::endl;
//      obs_rect.xmin = obs_point.x - l/2;
//      obs_rect.xmax = obs_point.x + l/2;
//      obs_rect.ymin = obs_point.z;
//      obs_rect.ymax = obs_point.z + l;

      obs_info.points.push_back(obs_point);
      obs_info.rects.push_back(obs_rect);

      cam::ekf_data ekf;
      ekf.u = obsta[i].x;
      ekf.v = obsta[i].y;
      ekf.px = radar_data.at<float>(0, lidar_obsta.at<int>(i,0));
      ekf.py = radar_data.at<float>(1, lidar_obsta.at<int>(i,0));
      ekf.vx = radar_data.at<float>(2, lidar_obsta.at<int>(i,0));
      ekf.vy = radar_data.at<float>(3, lidar_obsta.at<int>(i,0));
      ekf_m.data.push_back(ekf);
    }

    obstacle_info_pub.publish(obs_info);
    ekf_measure_pub.publish(ekf_m);
    cv::imshow("lalala", img_cam);
    //std::cout<<point<<std::endl;
    cv::waitKey(1);
  }
}

void setCameraParam(param param_camera)
{
  camParam.fu = param_camera.camera_fx;
  camParam.fv = param_camera.camera_fy;
  camParam.cu = param_camera.camera_cx;
  camParam.cv = param_camera.camera_cy;
  camParam.pitch = 91.5*(CV_PI*1.0/180.0);
  camParam.yaw = -1.0*(CV_PI*1.0/180.0);
  camParam.roll = 0*(CV_PI*1.0/180.0);
  radar2image.set_param(camParam);
}
