#include <mymission/avoid_obsta_from_mono_radar.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_avoidance_task");
  ros::NodeHandle nh;

  if(argc != 2)
  {
      std::cerr << std::endl << "no config file" << std::endl;
      ros::shutdown();
      return 1;
  }

  vision_task_param.readparam(argv[1]);
  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  message_filters::Subscriber<cam::mono_radar_fusion> obstacle_infosub(nh, "/mono_radar/obstacle/msg", 10);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> vision_infosub(nh, vision_task_param.obsta_topic, 10);
  obstacle_infosub.registerCallback(obsta_info_callback);
  vision_infosub.registerCallback(vision_info_callback);

  square_mission.pubControl(nh);
  square_mission.setService(nh);
  bool obtain_control_result = square_mission.obtain_control();
  bool takeoff_result;
  //set path planning
  Artifical = artifical_potencial_field();
  square_mission.con_law.set_control_law(vision_task_param.GPS_nav_Kp, vision_task_param.GPS_nav_Ki,
                                         vision_task_param.GPS_nav_Kd, vision_task_param.GPS_nav_Ka,
                                         vision_task_param.GPS_nav_Kv);
  con_law_vision.set_control_law(vision_task_param.vision_nav_Kp, vision_task_param.vision_nav_Ki,
                                 vision_task_param.vision_nav_Kd, vision_task_param.vision_nav_Ka,
                                 vision_task_param.vision_nav_Kv);
  con_law_yaw.set_control_law(vision_task_param.yaw_nav_Kp, vision_task_param.yaw_nav_Ki,
                              vision_task_param.yaw_nav_Kd, vision_task_param.yaw_nav_Ka,
                              vision_task_param.yaw_nav_Kv);

  if (!square_mission.set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }
  square_mission.reset();
  if(square_mission.is_M100())
  {
    ROS_INFO("M100 taking off!");
    takeoff_result = square_mission.M100monitoredTakeoff();
  }
//    ROS_INFO("A3/N3 taking off!");
//    takeoff_result = monitoredTakeoff();

  if(takeoff_result)
  {
    time_s = ros::Time::now();
    square_mission.reset();
    square_mission.start_gps_location = square_mission.current_gps;
    square_mission.start_local_position = current_local_pos;
    start_yaw = square_mission.toEulerAngle(square_mission.current_atti).z * square_mission.rad2deg;
    std::cout<<"start yaw: "<<start_yaw<<std::endl;
    square_mission.setTarget(0, 0, current_local_pos.z, start_yaw);
    square_mission.state = 1;
    ROS_INFO("##### Start route %d ....", square_mission.state);
  }

  ros::spin();
  return 0;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  square_mission.current_atti = msg->quaternion;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;

  double current_yaw = square_mission.toEulerAngle(square_mission.current_atti).z * square_mission.rad2deg;
  square_mission.current_gps = *msg;
  //std::cout<<"current_yaw: "<<current_yaw<<std::endl;

  if(elapsed_time > ros::Duration(0.02))
  {
    if (square_mission.state == 0) {
    }

    if (square_mission.state == 1)
    {
      std::cout<<"start mode 1"<<std::endl;
      time_now = ros::Time::now().toSec() - time_s.toSec();
      if(time_now <= 15)
      {
        std::cout<<"wen ding jie duan"<<std::endl;
        square_mission.setTarget(0, 0, 4, start_yaw);
        square_mission.state = 1;
        square_mission.step();
      }
      if(time_now>15 && time_now<25 && is_get_data())
      {
        std::cout<<"dui zhun jie duan"<<std::endl;
        square_mission.setTarget(0, 0, 4.8, start_yaw);
        square_mission.state = 1;
        vision_navigation();
        //square_mission.state = 2;
      }
      if(time_now>15 && time_now<25 && !is_get_data())
      {
        std::cout<<"dui zhun wu shu ju jie duan"<<std::endl;
        square_mission.setTarget(current_local_pos.x, current_local_pos.y, 1.2, start_yaw);
        square_mission.state = 1;
        square_mission.step();
      }
      if(time_now>25)
      {
        std::cout<<"tiao zhuan jie duan"<<std::endl;
        vision_height = current_local_pos.z;
        square_mission.state = 2;
      }
    }

    if(square_mission.state == 2)
    {
      std::cout<<"start mode 2"<<std::endl;
      planning_path_time = ros::Time::now().toSec() - time_s.toSec() - time_now;
      std::cout<<"planning path time: "<<planning_path_time<<std::endl;

      if(!has_got_path && data_is_available())
      {
        get_path();
        std::cout<<"get path ok ----------------------"<<std::endl;
        square_mission.state = 3;
      }
      else
        square_mission.state = 2;
    }

    if (square_mission.state == 3)
    {
      std::cout<<"start mode 3"<<std::endl;
      visionok_time = ros::Time::now().toSec() - time_s.toSec() - time_now - planning_path_time;
      std::cout<<"visionok_time: "<<visionok_time<<" time_now: "<<time_now<<std::endl;

      float t_path = (Artifical.get_path_length())/0.8;
      if(visionok_time <= t_path && has_got_path)
      {
        int path_i = visionok_time*0.8/0.1 + 1;
        std::cout<<"path_i: "<<path_i<<"--path size: "<<Artifical.Goal.size()<<std::endl;
        float x = Artifical.Goal[path_i].x;
        float y = Artifical.Goal[path_i].y;
        square_mission.setTarget(x, y, vision_height, start_yaw);
        square_mission.state = 3;
        square_mission.step();
        std::cout<<"x: "<<current_local_pos.x<<"---y: "<<current_local_pos.y<<std::endl;
      }
      else if (visionok_time <= 20)
      {
        square_mission.setTarget(current_local_pos.x, current_local_pos.y, vision_height, start_yaw);
        square_mission.state = 3;
        square_mission.step();
      }
      else
        square_mission.state = 4;
    }

    if (square_mission.state ==4)
    {
      std::cout<<"start mode 3"<<std::endl;
      land_time = ros::Time::now().toSec() - time_s.toSec();
      if(square_mission.monitoredLand())
        square_mission.state = 0;
    }

  }
}

void obsta_info_callback(const cam::mono_radar_fusionConstPtr &obsta_info)
{
  int obs_num = obsta_info->points.size();
  obs_pts = cv::Mat::zeros(4, obs_num, CV_32FC1);
  for(int i = 0; i<obs_num; i++)
  {
    obs_pts.at<float>(0,i) = obsta_info->points[i].x;
    obs_pts.at<float>(1,i) = obsta_info->points[i].z;
    obs_pts.at<float>(2,i) = -obsta_info->points[i].y;
    obs_pts.at<float>(3,i) = 1;
  }

  if(!start_get_path)
  {
    cv::Mat obs_ground = body2ground(obs_pts);
    std::vector<cv::Point2f> obstacle_nodes;
    for(int i = 0; i<obs_num; i++)
    {
      cv::Point2f node;
      node.x = obs_ground.at<float>(0,i);
      node.y = obs_ground.at<float>(1,i);
      obstacle_nodes.push_back(node);
    }
    obstacle_nodes_s.push_back(obstacle_nodes);
    if(obstacle_nodes_s.size()>10)
    {
      obstacle_nodes_s.erase(obstacle_nodes_s.begin());
    }
  }

}

void vision_info_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& vi)
{
  int boxes_num = vi->bounding_boxes.size();
  //std::cout<<"boxes num: "<<boxes_num<<std::endl;
  vision_info.bounding_boxes.resize(boxes_num);
  for(int i=0; i<boxes_num; i++)
    vision_info.bounding_boxes[i] = vi->bounding_boxes[i];
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  square_mission.flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  square_mission.display_mode = msg->data;
}

float sum_vision(std_msgs::Float32MultiArray data)
{
  float ans = 0;
  for(int i = 0; i < data.data.size(); i++)
  {
    ans = ans + data.data[i];
  }
  return ans;
}

bool is_get_data()
{
  for(int i=0; i<obs_pts.cols; i++)
  {
    if(!obs_pts.at<float>(0,i) && !obs_pts.at<float>(1,i) && !obs_pts.at<float>(2,i))
      return false;
  }
  return true;
}

bool data_is_available()
{
    bool error_is_ok = false;
    float max_error_0x = 0;
    float max_error_0y = 0;
    float max_error_1x = 0;
    float max_error_1y = 0;
    std::cout<<obstacle_nodes_s.size()<<std::endl;
    if(obstacle_nodes_s.size() < 10)
      return false;
    for(int i=0; i<obstacle_nodes_s.size() - 1; i++)
    {
        std::vector<cv::Point2f> obs_point_j = obstacle_nodes_s[i+1];
        std::vector<cv::Point2f> obs_point_i = obstacle_nodes_s[i];
        float a = obs_point_j[0].x - obs_point_i[0].x;
        float b = obs_point_j[0].y - obs_point_i[0].y;
        float c = obs_point_j[1].x - obs_point_i[1].x;
        float d = obs_point_j[1].y - obs_point_i[1].y;
        if(std::abs(a) > std::abs(max_error_0x))
            max_error_0x = a;
        if(std::abs(b) > std::abs(max_error_0y))
            max_error_0y = b;
        if(std::abs(c) > std::abs(max_error_1x))
            max_error_1x = c;
        if(std::abs(d) > std::abs(max_error_1y))
            max_error_1y = d;
    }
//    std::cout<<"max_error_0x: "<<max_error_0x<<"--max_error_0y: "<<
//               max_error_0y<<"--max_error_1x: "<<max_error_1x<<"--max_error_1y: "<<
//               max_error_1y<<std::endl;
    if((std::abs(max_error_0x) < 0.5) && (std::abs(max_error_0y) < 0.5) &&
            (std::abs(max_error_1x) < 0.5) && (std::abs(max_error_1y) < 0.5))
        error_is_ok = true;
    std::cout<<"error is ok"<<std::endl;
    return error_is_ok;
}

void get_path()
{
    start_get_path = true;
    //obstacle_nodes_s.back()
    cv::Point2f StartP0 = cv::Point2f(current_local_pos.x, current_local_pos.y);
    Artifical.SetStartPoint(StartP0);
    std::cout<<"start point: "<<StartP0.x<<", "<<StartP0.y<<std::endl;
    std::vector<cv::Point2f> nodes;
    nodes.push_back(cv::Point2f(vision_task_param.goal_x, vision_task_param.goal_y));
    std::vector<cv::Point2f> obs = obstacle_nodes_s.back();
    for(int i=0; i<obs.size(); i++)
      nodes.push_back(obs[i]);
    std::cout<<"node: "<<nodes[0].x<<", "<<nodes[0].y<<"--"<<nodes[1].x<<", "<<nodes[1].y<<
               "--"<<nodes[2].x<<", "<<nodes[2].y<<std::endl;
    Artifical.SetObstacle(nodes);
    Artifical.path_planning();
    start_get_path = false;
    has_got_path = true;
}

void vision_navigation()
{
  float xcmd, ycmd, zcmd, yawcmd;
  double sum_y = 0;
  int num = vision_info.bounding_boxes.size();
  for(int i=0; i<num; i++)
    sum_y = vision_info.bounding_boxes[i].rows/2 - (vision_info.bounding_boxes[i].ymin+vision_info.bounding_boxes[i].ymax)/2 + sum_y;
  std::cout<<"num: "<<num<<"---sum_y: "<<sum_y<<std::endl;
  float zdesired = sum_y/num;
  float yawdesired = 0;
  double yawDesiredRad = square_mission.deg2rad * start_yaw;

  if(vs_info_counter < 5)
  {
    vs_data_y.data.push_back(zdesired);
    vs_data_yaw.data.push_back(yawdesired);
  }
  else {
    for(int i = 3; i >= 0; i--)
    {
      vs_data_y.data[i+1] = vs_data_y.data[i];
      vs_data_yaw.data[i+1] = vs_data_yaw.data[i];
    }
    vs_data_y.data[0] = zdesired;
    vs_data_yaw.data[0] = yawdesired;

    vs_data_y.data.resize(5);
    vs_data_yaw.data.resize(5);
  }
  vs_info_counter++;

  std::cout<<"zdesired: "<<zdesired<<std::endl;
  if(is_get_data())
    zcmd = con_law_vision.Kp * zdesired + con_law_vision.Ki * sum_vision(vs_data_y)
        + con_law_vision.Kd * (zdesired - vs_data_y.data[vs_data_y.data.size() - 2]);
  else zcmd = 0;
  //ycmd = -ycmd;

  yawcmd = con_law_yaw.Kp * yawdesired + con_law_yaw.Ki * sum_vision(vs_data_yaw)
      + con_law_yaw.Kd * (yawdesired - vs_data_yaw.data[vs_data_yaw.data.size() - 2]);

  xcmd = square_mission.target_offset_x;
  ycmd = square_mission.target_offset_y;

  std::cout<<"xcmd: "<<xcmd<<" ycmd: "<<ycmd<<" yawcmd: "<<yawcmd<<" zcmd: "<<zcmd<<std::endl;

  if(xcmd >= square_mission.lim_vel)
    xcmd = square_mission.lim_vel;
  if(xcmd <= -square_mission.lim_vel)
    xcmd = -square_mission.lim_vel;

  if(ycmd >= square_mission.lim_vel)
    ycmd = square_mission.lim_vel;
  if(ycmd <= -square_mission.lim_vel)
    ycmd = -square_mission.lim_vel;

  if(zcmd >= square_mission.lim_vel)
    zcmd = square_mission.lim_vel;
  if(zcmd <= -square_mission.lim_vel)
    zcmd = -square_mission.lim_vel;

  if(yawcmd >= 10)
    yawcmd = 10;
  if(yawcmd <= -10)
    yawcmd = -10;

  std::cout<<"vx: "<<xcmd<<" vy: "<<ycmd<<" vyaw: "<<yawcmd<<std::endl;

  sensor_msgs::Joy controlVelYawRate;
  uint8_t flag1 = (DJISDK::VERTICAL_VELOCITY  |
                  DJISDK::HORIZONTAL_VELOCITY |
                  DJISDK::YAW_ANGLE           |
                  DJISDK::HORIZONTAL_BODY     |
                  DJISDK::STABLE_ENABLE);

  controlVelYawRate.axes.push_back(xcmd);
  controlVelYawRate.axes.push_back(ycmd);
  controlVelYawRate.axes.push_back(zcmd);
  controlVelYawRate.axes.push_back(yawDesiredRad);
  controlVelYawRate.axes.push_back(flag1);
  square_mission.ctrlBrakePub.publish(controlVelYawRate);
}

cv::Mat body2ground(cv::Mat P_body)
{
  cv::Mat Trans;
  double d_pitch = square_mission.toEulerAngle(square_mission.current_atti).y;
  double d_yaw   = square_mission.toEulerAngle(square_mission.current_atti).z - C_PI/2;
  double d_roll  = square_mission.toEulerAngle(square_mission.current_atti).x;
  double d_x     = current_local_pos.x - square_mission.start_local_position.x;
  double d_y     = current_local_pos.y - square_mission.start_local_position.y;
  double d_z     = current_local_pos.z - square_mission.start_local_position.z;

  cv::Mat Rx = (cv::Mat_<float>(3,3)<<1,0,0,0,std::cos(d_pitch),
                -std::sin(d_pitch),0,std::sin(d_pitch),
                std::cos(d_pitch));
  cv::Mat Ry = (cv::Mat_<float>(3,3)<<std::cos(d_roll),0,std::sin(d_roll),
                0,1,0,-std::sin(d_roll),0,std::cos(d_roll));
  cv::Mat Rz = (cv::Mat_<float>(3,3)<<std::cos(d_yaw),-std::sin(d_yaw),
                0,std::sin(d_yaw),std::cos(d_yaw),0,0,0,1);
  cv::Mat R = Rx*Ry*Rz;

  Trans = (cv::Mat_<float>(4,4)<<R.at<float>(0,0),R.at<float>(0,1),R.at<float>(0,2),d_x,
          R.at<float>(1,0),R.at<float>(1,1),R.at<float>(1,2),d_y,
          R.at<float>(2,0),R.at<float>(2,1),R.at<float>(2,2),d_z,
          0,0,0,1);
  //std::cout<<"trans: "<<Trans<<std::endl;

  cv::Mat P_ground = Trans*P_body;
  return P_ground;
}
