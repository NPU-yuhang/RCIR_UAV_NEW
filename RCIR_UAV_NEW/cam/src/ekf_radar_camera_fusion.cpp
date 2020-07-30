#include "cam/ekf_radar_camera_fusion.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ekf_radar_camera_fusion");
  ros::NodeHandle nh;
  message_filters::Subscriber<cam::ekf> ekf_sub(nh, "/ekf/measurement/msg", 10);
  ekf_sub.registerCallback(ekf_measurement_callback);
  ros::spin();
  ros::shutdown();
  return 0;
}

void ekf_measurement_callback(const cam::ekfConstPtr &ekf_m)
{
  int obs_count = ekf_m->data.size();
  //std::cout<<"obs count: "<<obs_count<<std::endl;
  for(int i=0; i<obs_count; i++)
  {
    radar_measure.sensor_type_ = MeasurementPackage::RADAR;
    radar_measure.raw_measurements_ = Eigen::VectorXd(4);
    float px = ekf_m->data[i].px;
    float py = ekf_m->data[i].py;
    float vx = ekf_m->data[i].vx;
    float vy = ekf_m->data[i].vy;
    radar_measure.raw_measurements_ << px, py, vx, vy;
    radar_measure.timestamp_ = ekf_m->header.stamp.toNSec();
    ekf_obstacles.ProcessMeasurement(radar_measure);
    //std::cout<<"x_radar: "<<ekf_obstacles.ekf_.x_<<std::endl;
    camera_measure.sensor_type_ = MeasurementPackage::CAMERA;
    camera_measure.raw_measurements_ = Eigen::VectorXd(2);
    int u = ekf_m->data[i].u;
    int v = ekf_m->data[i].v;
    camera_measure.raw_measurements_ <<u, v;
    camera_measure.timestamp_ = ekf_m->header.stamp.toNSec();
    ekf_obstacles.ProcessMeasurement(camera_measure);
    //std::cout<<"x_cam: "<<ekf_obstacles.ekf_.x_<<std::endl;
  }
  location_out.open("/home/yuhang/catkin_m100/src/cam/output/ekf_out.txt", std::ios::out | std::ios::app);
  raw_data_out.open("/home/yuhang/catkin_m100/src/cam/output/raw_data_out.txt", std::ios::out | std::ios::app);
  std::string ekf_out;
  std::string raw_data;
  ekf_out = std::to_string(ekf_obstacles.ekf_.x_[0]) + " " +
      std::to_string(ekf_obstacles.ekf_.x_[1]) + " " +
      std::to_string(ekf_obstacles.ekf_.x_[2]) + " " +
      std::to_string(ekf_obstacles.ekf_.x_[3]) + " " +
      std::to_string(ekf_obstacles.ekf_.x_[4]) + " " +
      std::to_string(ekf_m->header.stamp.toNSec()) + "\n";
  raw_data = std::to_string(ekf_m->data[0].px) + " " +
      std::to_string(ekf_m->data[0].py) + " " +
      std::to_string(ekf_m->data[0].vx) + " " +
      std::to_string(ekf_m->data[0].vy) + " " +
      std::to_string(ekf_m->data[0].u) + " " +
      std::to_string(ekf_m->data[0].v) + " " +
      std::to_string(ekf_m->header.stamp.toNSec()) + "\n";
  location_out<<ekf_out;
  raw_data_out<<raw_data;
  location_out.close();
  raw_data_out.close();
}
