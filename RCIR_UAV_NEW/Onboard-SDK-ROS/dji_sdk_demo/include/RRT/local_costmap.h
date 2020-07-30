#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include "cam/point.h"
#include "cam/rect.h"
#include "cam/mono_radar_fusion.h"

typedef std::pair<double, double> points;

namespace RRT {

class LocalCostmap{

public:

    LocalCostmap(ros::NodeHandle &nh, ros::NodeHandle pnh);
    ~LocalCostmap();

    void pubOccGrid();

private:

    void setupCostmap();
    void createObstacles();
    std::vector<points> createRectangle(const double &height, const double &width, const points &center);
    std::vector<points> calcPointsBetween(const points &pt1, const points &pt2);
    void calcOccGrid();
    int calcGridLocation(const double &x, const double &y);
    points calcCartesianCoords(const int &location);
    void clearMap();    
    void ObstaclesCallback(const cam::mono_radar_fusionConstPtr &obsta);

    ros::Publisher occ_grid_pub;
    ros::Subscriber obstacle_sub;
    cam::mono_radar_fusion obsta_info;

    tf::TransformListener listener;
    nav_msgs::OccupancyGrid occ_grid;

    double m_local_costmap_res;
    double m_local_costmap_height;
    double m_local_costmap_width;
    int m_grid_array_length;

};

}
