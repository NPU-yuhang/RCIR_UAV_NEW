#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>


#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include "dji_sdk/dji_sdk.h"

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    double yaw = toEulerAngle(msg->quaternion).z;
    std::cout<<"yaw: "<<yaw<<std::endl;
    std::cout<<"y: "<<toEulerAngle(msg->quaternion).y<<std::endl;
    std::cout<<"x: "<<toEulerAngle(msg->quaternion).x<<std::endl;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    double x = msg->point.x;
    double y = msg->point.y;
    std::cout<<"x: "<<x<<"---y: "<<y<<std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_flight_control_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);

  ros::spin();
  return 0;
}
