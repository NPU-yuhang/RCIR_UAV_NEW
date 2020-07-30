#include <RRT/local_planner.h>

namespace RRT {

LocalPlanner::LocalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/local_planner/local_costmap", 10, &LocalPlanner::costmapCallback, this);
    goal_sub = nh.subscribe<geometry_msgs::Point>("/local_planner/goal", 10, &LocalPlanner::goalCallback, this);
    path_pub = nh.advertise<nav_msgs::Path>("/local_planner/local_path", 10);
    path_pub_1 = nh.advertise<nav_msgs::Path>("/local_planner/local_path_1", 10);
    path_pub_2 = nh.advertise<nav_msgs::Path>("/local_planner/local_path_2", 10);
    occ_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("RRT_tree", 10);
    setupCostmap(pnh);
    setupCSpace();
    getTreeParams(pnh);
    srand(time(NULL));
}

LocalPlanner::~LocalPlanner()
{
    ROS_INFO_STREAM("noooooooooooooooooooooooooooooooooo");
}

void LocalPlanner::planPath()
{
    clearVisited();
    clearTree();
    point current_pt_forward(m_local_costmap_height / 2 - 1, m_local_costmap_width / 2 - 1);
    point current_pt_backward(current_pt_forward.first + m_goal_pt.x / m_local_costmap_res, current_pt_forward.second + m_goal_pt.y / m_local_costmap_res);
    std::cout<<"current_pt_forward: "<<current_pt_forward.first<<std::endl;
    std::cout<<"current_pt_backward: "<<current_pt_backward.first<<std::endl;
    markVisited(current_pt_forward);
    markVisited(current_pt_backward);
    m_tree_forward.insert(std::pair<point, point>(current_pt_forward, current_pt_forward));
    m_tree_backward.insert(std::pair<point, point>(current_pt_backward, current_pt_backward));
    int count = 1;
    int viz_count = 1;
    while(true)
    {
        auto result = checkForConnection();
        if(result.first)
        {
            ROS_INFO_STREAM("path found!");
            calcOccGrid();
            calcPathMsg(result.second);
            break;
        }
        m_tree_forward = expandTree(m_tree_forward, count);
        m_tree_backward = expandTree(m_tree_backward, count);
        if(viz_count == 5)
        {
            calcOccGrid();
            viz_count = 1;
        }
        if(count == 10)
        {
            count = 1;
        }
        count++;
        viz_count++;
    }
}

std::map<point, point> LocalPlanner::expandTree(std::map<point, point> tree, const int &count)
{
    auto new_pt_and_neighbor = getNewPoint(tree, count);
    point new_pt = new_pt_and_neighbor.first;
    point nearest_node = new_pt_and_neighbor.second;
    while(checkCollision(new_pt))
    {
        new_pt_and_neighbor = getNewPoint(tree, 0);
        new_pt = new_pt_and_neighbor.first;
        nearest_node = new_pt_and_neighbor.second;
    }
    std::vector<point> neighbors = calcNeighborsInRadius(new_pt, nearest_node);
    point cheapest_pt = calcCheapestNeighbor(neighbors);
    std::vector<point> pts_bet = calcPointsBetween(new_pt, cheapest_pt);
    while(checkNewPointsForCollision(pts_bet))
    {
        new_pt_and_neighbor = getNewPoint(tree, 0);
        new_pt = new_pt_and_neighbor.first;
        nearest_node = new_pt_and_neighbor.second;
        neighbors = calcNeighborsInRadius(new_pt, nearest_node);
        cheapest_pt = calcCheapestNeighbor(neighbors);
        pts_bet = calcPointsBetween(new_pt, cheapest_pt);
    }
    tree.insert(std::pair<point, point>(new_pt, cheapest_pt));    
    auto result = checkNewPointsForConnection(pts_bet);
    if(std::get<0>(result))
    {        
        m_tree_forward.insert(std::pair<point, point>(std::get<1>(result), m_tree_forward[std::get<2>(result)]));
        m_tree_backward.insert(std::pair<point, point>(std::get<1>(result), m_tree_backward[std::get<3>(result)]));
    }
    for(auto pt : pts_bet)
    {
        markVisited(pt);
    }
    m_tree_counter++;
    return tree;
}

std::pair<point, point> LocalPlanner::getNewPoint(std::map<point, point> tree, const int &count)
{
  point pt = getRandomPoint();
  while(m_tree_forward.count(pt) || m_tree_backward.count(pt))
  {
      pt = getRandomPoint();
  }
  if(count == 10)
  {
      if(m_tree_counter == 0)
      {
          pt = point(m_local_costmap_height / 2 - 1 + m_goal_pt.x / m_local_costmap_res, m_local_costmap_width / 2 - 1 + m_goal_pt.y / m_local_costmap_res);
      }
      else
      {
          m_tree_counter = 0;
          pt = point(m_local_costmap_height / 2 - 1, m_local_costmap_width / 2 - 1);
      }
  }
  point nearest_node = calcNearestNode(pt, tree);
  point new_pt = interpolatePoint(pt, nearest_node);
  while((m_tree_forward.count(new_pt) || m_tree_backward.count(new_pt)))
  {
      pt = getRandomPoint();
      while(m_tree_forward.count(pt) || m_tree_backward.count(pt))
      {
          pt = getRandomPoint();
      }
      nearest_node = calcNearestNode(pt, tree);
      new_pt = interpolatePoint(pt, nearest_node);
  }
  return std::make_pair(new_pt, nearest_node);
}

void LocalPlanner::clearVisited()
{
    m_c_space_visited.clear();
    for(int i = 0; i < m_local_costmap_width; i++)
    {
        m_c_space_visited.push_back({});
        for(int j = 0; j < m_local_costmap_height; j++)
        {
            m_c_space_visited[i].push_back(0);
        }
    }
}

void LocalPlanner::clearTree()
{
    m_tree_forward.clear();
    m_tree_backward.clear();
}

point LocalPlanner::getRandomPoint()
{    
    int x = rand() % (m_c_space.size() - 1);
    int y = rand() % (m_c_space[0].size() - 1);
    return point(x, y);
}

point LocalPlanner::calcNearestNode(const point &pt, std::map<point, point> tree)
{
    point nearest_node;
    double lowest_distance = sqrt(pow(m_local_costmap_width, 2) + pow(m_local_costmap_height, 2));
    for(std::map<point, point>::iterator it = tree.begin(); it != tree.end(); it++)
    {
        double dx = pt.first - it->first.first;
        double dy = pt.second - it->first.second;
        double dist = sqrt(pow(dx, 2) + pow(dy, 2));
        if(dist < lowest_distance)
        {
            lowest_distance = dist;
            nearest_node = it->first;
        }
    }
    return nearest_node;
}

std::vector<point> LocalPlanner::calcNeighborsInRadius(const point &new_pt, const point &nearest_node)
{
    std::vector<point> neighbors;
    std::map<point, point> tree;
    if(m_tree_forward.count(nearest_node))
    {
        tree = m_tree_forward;
    }
    else if(m_tree_backward.count(nearest_node))
    {
        tree = m_tree_backward;
    }
    for(std::map<point, point>::iterator it = tree.begin(); it != tree.end(); it++)
    {
        double dx = it->first.first - new_pt.first;
        double dy = it->first.second - new_pt.second;
        double dist = sqrt(pow(dx, 2) + pow(dy, 2));
        if(dist <= m_search_radius)
        {
            neighbors.push_back(it->first);
        }
    }
    return neighbors;
}

point LocalPlanner::calcCheapestNeighbor(const std::vector<point> &points)
{
    point cheapest_pt;
    double lowest_cost = sqrt(pow(m_local_costmap_height, 2) + pow(m_local_costmap_res, 2));
    for(auto pt : points)
    {
        double cost = calcCost(pt);
        if(cost < lowest_cost)
        {
            cheapest_pt = pt;
        }
    }
    return cheapest_pt;
}

double LocalPlanner::calcCost(const point &pt)
{
    std::map<point, point> tree;
    point current_pt = pt;
    point last_pt = current_pt;
    point end_pt;
    double cost;
    if(m_tree_forward.count(pt))
    {
        tree = m_tree_forward;
        end_pt = point(m_local_costmap_height / 2 - 1, m_local_costmap_width / 2 - 1);
    }
    else if(m_tree_backward.count(pt))
    {
        tree = m_tree_backward;
        end_pt = point(m_local_costmap_height / 2 - 1 + m_goal_pt.x / m_local_costmap_res, m_local_costmap_width / 2 - 1 + m_goal_pt.y / m_local_costmap_res);
    }
    while(current_pt != end_pt)
    {
        double dx = current_pt.first - last_pt.first;
        double dy = current_pt.second - last_pt.second;
        double dist = sqrt(pow(dx, 2) + pow(dy, 2));
        cost += dist;
        last_pt = current_pt;
        current_pt = tree[current_pt];
    }
    return cost;
}

point LocalPlanner::interpolatePoint(const point &rand_pt, const point &nearest_node)
{
    double d_x = rand_pt.first - nearest_node.first;
    double d_y = rand_pt.second - nearest_node.second;
    double angle_to_pt = atan2(d_y, d_x);
    int x = nearest_node.first + m_max_branch_length * cos(angle_to_pt);
    int y = nearest_node.second + m_max_branch_length * sin(angle_to_pt);
    while(x > m_c_space.size() - 1 || y > m_c_space[0].size() - 1)
    {
        x -= cos(angle_to_pt);
        y -= sin(angle_to_pt);
    }
    point pt = std::make_pair(x, y);
    return pt;
}

std::vector<point> LocalPlanner::calcPointsBetween(const point &pt1, const point &pt2)
{
    std::vector<point> points_between;
    double d_x = pt2.first - pt1.first;
    double d_y = pt2.second - pt1.second;
    double dist_between = sqrt(pow(d_x, 2) + pow(d_y, 2));
    int num_pts_between = dist_between / m_local_costmap_res * 2;
    double angle_between = atan2(d_y, d_x);
    for(int i = 0; i < num_pts_between; i++)
    {
        int x = pt1.first + i / double(num_pts_between) * dist_between * cos(angle_between);
        int y = pt1.second + i / double(num_pts_between) * dist_between * sin(angle_between);
        point pt = std::make_pair(x, y);
        points_between.push_back(pt);
    }
    return points_between;
}

bool LocalPlanner::checkNewPointsForCollision(const std::vector<point> &points_between)
{
    for(auto pt : points_between)
    {
        if(checkCollision(pt))
        {
            return true;
        }
    }
    return false;
}

std::tuple<bool, point, point, point> LocalPlanner::checkNewPointsForConnection(const std::vector<point> &points_between)
{
    point connection_pt;
    for(auto pt : points_between)
    {
        std::vector<point> forward_neighbors;
        for(std::map<point, point>::iterator it = m_tree_forward.begin(); it != m_tree_forward.end(); it++)
        {
            double dx = it->first.first - pt.first;
            double dy = it->first.second - pt.second;
            double dist = sqrt(pow(dx, 2) + pow(dy, 2));
            if(dist <= m_search_radius)
            {
                forward_neighbors.push_back(it->first);
            }
        }
        std::vector<point> backward_neighbors;
        for(std::map<point, point>::iterator it = m_tree_backward.begin(); it != m_tree_backward.end(); it++)
        {
            double dx = it->first.first - pt.first;
            double dy = it->first.second - pt.second;
            double dist = sqrt(pow(dx, 2) + pow(dy, 2));
            if(dist <= m_search_radius)
            {
                backward_neighbors.push_back(it->first);
            }
        }
        std::vector<std::vector<point>> forward_line_pts;
        for(auto neighbor : forward_neighbors)
        {
            forward_line_pts.push_back(calcPointsBetween(neighbor, m_tree_forward[neighbor]));
        }
        std::vector<std::vector<point>> backward_line_pts;
        for(auto neighbor : backward_neighbors)
        {
            backward_line_pts.push_back(calcPointsBetween(neighbor, m_tree_backward[neighbor]));
        }
        for(auto segment : forward_line_pts)
        {
            for(auto pt_ : segment)
            {
                for(auto segment_ : backward_line_pts)
                {
                    for(auto pt__ : segment_)
                    {
                        if(pt_ == pt__)
                        {
                            return std::make_tuple(true, pt_, segment[0], segment_[0]);
                        }
                    }
                }
            }
        }
    }
    return std::make_tuple(false, connection_pt, point(0, 0), point(0, 0));
}

std::pair<bool, point> LocalPlanner::checkForConnection()
{
    point connection_pt;
    for(std::map<point, point>::iterator it = m_tree_backward.begin(); it != m_tree_backward.end(); it++)
    {
      if(m_tree_forward.count(it->first))
      {
          return std::make_pair(true, it->first);
      }
    }
    return std::make_pair(false, connection_pt);
}

void LocalPlanner::markVisitedPoints(const std::vector<point> &points)
{
    for(auto pt : points)
    {
        markVisited(pt);
    }
}

void LocalPlanner::markVisited(const point &pt)
{
    m_c_space_visited[pt.first][pt.second] = 1;
}

void LocalPlanner::setupCostmap(ros::NodeHandle &pnh)
{
    double costmap_height_meters;
    double costmap_width_meters;
    pnh.getParam("/local_costmap_node/local_costmap_res", m_local_costmap_res);
    pnh.getParam("/local_costmap_node/local_costmap_height", costmap_height_meters);
    pnh.getParam("/local_costmap_node/local_costmap_width", costmap_width_meters);
    m_local_costmap_height = costmap_height_meters / m_local_costmap_res;
    m_local_costmap_width = costmap_width_meters / m_local_costmap_res;
    //std::cout<<"m_local_costmap_height: "<<m_local_costmap_height<<"--"<<"m_local_costmap_width: "<<m_local_costmap_width<<std::endl;
}

void LocalPlanner::setupCSpace()
{
    for(int i = 0; i < m_local_costmap_width; i++)
    {
        m_c_space.push_back({});
        for(int j = 0; j < m_local_costmap_height; j++)
        {
            m_c_space[i].push_back(0);
        }
    }
}

void LocalPlanner::getTreeParams(ros::NodeHandle &pnh)
{
    double max_branch_length_meters;
    double search_radius;
    pnh.getParam("max_branch_length", max_branch_length_meters);
    pnh.getParam("search_radius", search_radius);
    m_max_branch_length = max_branch_length_meters / m_local_costmap_res;
    m_search_radius = search_radius / m_local_costmap_res;
}

void LocalPlanner::mapCostmapToCSpace(const nav_msgs::OccupancyGrid::ConstPtr &local_costmap)
{
    for(int i = 0; i < m_local_costmap_width; i++)
    {
        for(int j = 0; j < m_local_costmap_height; j++)
        {
            auto location = calcGridLocation(i, j);
            m_c_space[j][i] = local_costmap->data[location];
        }
    }
}

int LocalPlanner::calcGridLocation(const double &x, const double &y)
{
    int location = x + y * m_local_costmap_width - 1;
    return location;
}

point LocalPlanner::calcCartesianCoords(const point &pt)
{
    double x = (pt.first - (m_local_costmap_height / 2 - 1)) * m_local_costmap_res;
    double y = (pt.second - (m_local_costmap_width / 2 - 1)) * m_local_costmap_res;
    return point(x, y);
}

bool LocalPlanner::checkCollision(const point &pt)
{
    if(m_c_space[pt.first][pt.second] == 100)
    {
        return true;
    }
    return false;
}

void LocalPlanner::publishOccGrid(const nav_msgs::OccupancyGrid &grid)
{
    occ_grid_pub.publish(grid);
}

void LocalPlanner::calcOccGrid()
{
    nav_msgs::OccupancyGrid occ_grid;
    int m_grid_array_length = int(m_local_costmap_height * m_local_costmap_width);
    occ_grid.data.clear();
    for(int i = 0; i < m_grid_array_length; i++)
    {
        occ_grid.data.push_back(-1);
    }
    occ_grid.header.frame_id = "map";
    occ_grid.info.resolution = m_local_costmap_res;
    occ_grid.info.origin.position.x = -m_local_costmap_height / 10;
    occ_grid.info.origin.position.y = -m_local_costmap_width / 10;
    occ_grid.info.origin.position.z = 0;
    tf::Quaternion q = tf::createQuaternionFromRPY(0, M_PI, -M_PI / 2);
    occ_grid.info.origin.orientation.w = q.getW();
    occ_grid.info.origin.orientation.x = q.getX();
    occ_grid.info.origin.orientation.y = q.getY();
    occ_grid.info.origin.orientation.z = q.getZ();
    occ_grid.info.height = m_local_costmap_height;
    occ_grid.info.width = m_local_costmap_width;
    for(int i = 0; i < m_c_space_visited.size(); i++)
    {
        for(int j = 0; j < m_c_space_visited[0].size(); j++)
        {
            auto location = calcGridLocation(j, i);
            if(m_c_space_visited[i][j] == 1)
            {
                occ_grid.data[location] = 100;
            }
        }
    }
    publishOccGrid(occ_grid);
}

void LocalPlanner::calcPathMsg(const point &connection_pt)
{
    nav_msgs::Path path, path1, path2;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    path1.header.stamp = ros::Time::now();
    path1.header.frame_id = "map";
    path2.header.stamp = ros::Time::now();
    path2.header.frame_id = "map";
    std::vector<point> path_pts;
    std::vector<point> start_pts;
    std::vector<point> path_tailor;
    std::vector<point> path_catmull;
    point current_pt = connection_pt;
    point end_pt(m_local_costmap_height / 2 - 1, m_local_costmap_width / 2 - 1);
    while(current_pt != end_pt)
    {
        start_pts.push_back(current_pt);
        current_pt = m_tree_forward[current_pt];
    }
    for(int i = start_pts.size() - 1; i > 0; i--)
    {
        path_pts.push_back(start_pts[i]);
    }
    current_pt = m_tree_backward[connection_pt];
    end_pt = point(m_local_costmap_height / 2 - 1 + m_goal_pt.x / m_local_costmap_res, m_local_costmap_width / 2 - 1 + m_goal_pt.y / m_local_costmap_res);
    while(current_pt != end_pt)
    {
        path_pts.push_back(current_pt);
        current_pt = m_tree_backward[current_pt];
    }
    path_tailor = tailor_tree(path_pts);
    path_catmull = CatmullRomSpline(path_tailor);
    for(auto pt : path_tailor)
    {
        geometry_msgs::PoseStamped pose;
        point cartesian_pt = calcCartesianCoords(pt);
        pose.pose.position.x = cartesian_pt.first;
        pose.pose.position.y = cartesian_pt.second;
        path.poses.push_back(pose);
    }
    for(auto pt : path_pts)
    {
        geometry_msgs::PoseStamped pose;
        point cartesian_pt = calcCartesianCoords(pt);
        pose.pose.position.x = cartesian_pt.first;
        pose.pose.position.y = cartesian_pt.second;
        path1.poses.push_back(pose);
    }
    for(auto pt : path_catmull)
    {
        geometry_msgs::PoseStamped pose;
        point cartesian_pt = calcCartesianCoords(pt);
        pose.pose.position.x = cartesian_pt.first;
        pose.pose.position.y = cartesian_pt.second;
        path2.poses.push_back(pose);
    }
    publishPath(path, path1, path2);
}

void LocalPlanner::publishPath(const nav_msgs::Path &path, const nav_msgs::Path &path_1, const nav_msgs::Path &path_2)
{
    path_pub.publish(path);
    path_pub_1.publish(path_1);
    path_pub_2.publish(path_2);
}

void LocalPlanner::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    mapCostmapToCSpace(msg);
}

void LocalPlanner::goalCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    m_goal_pt = *msg;
    std::cout<<"plan path"<<std::endl;
    planPath();
    std::cout<<"plan path2"<<std::endl;
}

std::vector<point> LocalPlanner::tailor_tree(std::vector<point> path)
{
  std::vector<point> path_tailor;
  path_tailor.assign(path.begin(), path.end());
  std::cout<<"old size: "<<path_tailor.size()<<std::endl;
  for(int i=0; i<path.size(); i++)
  {
    for(int j=i+2; j<path.size();)
    {
      std::vector<point> pts_bet = calcPointsBetween(path[i], path[j]);
      if(checkNewPointsForCollision(pts_bet))
      {
        j++;
        break;
      }
      else
        path.erase(path.begin()+(j-1));
    }
  }
  std::cout<<"new size: "<<path.size()<<std::endl;
  return path;
}

std::vector<point> LocalPlanner::CatmullRomSpline(std::vector<point> path)
{
  std::vector<point> spline_path;
  cv::Mat Catmull;
  int numControlPoints = path.size()+4;
  int numSamplePerSpline = 100;
  cv::Mat controlPoints(numControlPoints, 2, CV_32FC1);
  controlPoints.at<float>(0,0) = m_local_costmap_width/2-1;//path[0].first - 1;
  controlPoints.at<float>(0,1) = m_local_costmap_height/2-1;//path[0].second - 1;
  controlPoints.at<float>(1,0) = m_local_costmap_width/2;
  controlPoints.at<float>(1,1) = m_local_costmap_height/2;
  controlPoints.at<float>(numControlPoints-2,0) = m_local_costmap_width/2 + m_goal_pt.x/m_local_costmap_res;
  controlPoints.at<float>(numControlPoints-2,1) = m_local_costmap_height/2 + m_goal_pt.y/m_local_costmap_res;
  controlPoints.at<float>(numControlPoints-1,0) = m_local_costmap_width/2 + m_goal_pt.x/m_local_costmap_res - 1;//path[numControlPoints-3].first + 1;
  controlPoints.at<float>(numControlPoints-1,1) = m_local_costmap_height/2 + m_goal_pt.y/m_local_costmap_res - 1;//path[numControlPoints-3].second + 1;
  for(int i=2; i<numControlPoints-2; i++)
  {
    controlPoints.at<float>(i,0) = path[i-2].first;
    controlPoints.at<float>(i,1) = path[i-2].second;
  }
  std::cout<<"controlPoints: "<<controlPoints<<std::endl;
  if(numControlPoints > 4)
  {
    int numSplines = numControlPoints - 3;
    Catmull = cv::Mat::zeros(numSamplePerSpline*numSplines, 2, CV_32FC1);
    for(int i=0; i<numSplines; i++)
    {
      cv::Mat spline = getSpline(controlPoints.rowRange(i,i+4), numSamplePerSpline);
      int stopIndex = (i+1)*numSamplePerSpline - 1;
      int startIndex = stopIndex - numSamplePerSpline + 1;
      spline.copyTo(Catmull.rowRange(startIndex, stopIndex+1));
    }
  }
  else if(numControlPoints = 4)
    Catmull = getSpline(controlPoints, numSamplePerSpline);
  std::cout<<"Catmull: "<<Catmull<<std::endl;
  spline_path = mat2point(Catmull);
  return spline_path;
}

cv::Mat LocalPlanner::getSpline(cv::Mat point, int num)
{
  cv::Mat P0, P1, P2, P3;
  point.row(0).copyTo(P0);
  point.row(1).copyTo(P1);
  point.row(2).copyTo(P2);
  point.row(3).copyTo(P3);
  std::cout<<"P0: "<<P0<<std::endl;
  std::cout<<"P1: "<<P1<<std::endl;
  std::cout<<"P2: "<<P2<<std::endl;
  std::cout<<"P3: "<<P3<<std::endl;
  double t0 = 0;
  double t1 = get_t(t0, P0, P1);
  double t2 = get_t(t1, P1, P2);
  double t3 = get_t(t2, P2, P3);
  cv::Mat t = cv::Mat::zeros(num, 1, CV_32FC1);
  for(int i=0; i<num; i++)
    t.at<float>(i,0) = t1+i*(t2-t1)/(num-1);
  std::cout<<t1<<std::endl;
  cv::Mat A1 = (t1-t)/(t1-t0)*P0 + (t-t0)/(t1-t0)*P1;
  cv::Mat A2 = (t2-t)/(t2-t1)*P1 + (t-t1)/(t2-t1)*P2;
  cv::Mat A3 = (t3-t)/(t3-t2)*P2 + (t-t2)/(t3-t2)*P3;
  //std::cout<<"A1: "<<A1<<std::endl;
  cv::Mat a1 = (t2-t)/(t2-t0);
  cv::Mat a2 = (t-t0)/(t2-t0);
  cv::Mat a3 = (t3-t)/(t3-t1);
  cv::Mat a4 = (t-t1)/(t3-t1);
  cv::Mat a5 = (t2-t)/(t2-t1);
  cv::Mat a6 = (t-t1)/(t2-t1);

  cv::Mat b1, b2, b3, b4, b5, b6;
  b1 = cv::Mat::zeros(num, 2, CV_32FC1);
  b2 = cv::Mat::zeros(num, 2, CV_32FC1);
  b3 = cv::Mat::zeros(num, 2, CV_32FC1);
  b4 = cv::Mat::zeros(num, 2, CV_32FC1);
  b5 = cv::Mat::zeros(num, 2, CV_32FC1);
  b6 = cv::Mat::zeros(num, 2, CV_32FC1);
  cv::Mat bb;
  a1.copyTo(bb);

  a1.copyTo(b1.col(0));
  a1.copyTo(b1.col(1));
  a2.copyTo(b2.col(0));
  a2.copyTo(b2.col(1));
  a3.copyTo(b3.col(0));
  a3.copyTo(b3.col(1));
  a4.copyTo(b4.col(0));
  a4.copyTo(b4.col(1));
  a5.copyTo(b5.col(0));
  a5.copyTo(b5.col(1));
  a6.copyTo(b6.col(0));
  a6.copyTo(b6.col(1));
  cv::Mat B1 = b1.mul(A1)+b2.mul(A2);
  cv::Mat B2 = b3.mul(A2)+b4.mul(A3);
  cv::Mat C = b5.mul(B1) + b6.mul(B2);

  return C;
}

double LocalPlanner::get_t(double ti, cv::Mat Pi, cv::Mat Pj)
{
  double t;
  double alpha = 0.5;
  t = std::pow(std::sqrt(std::pow(Pj.at<float>(0,0) - Pi.at<float>(0,0), 2) +
      std::pow(Pj.at<float>(0,1) - Pi.at<float>(0,1), 2)), alpha) + ti;
  return t;
}

std::vector<point> LocalPlanner::mat2point(cv::Mat catmull_spline)
{
  std::vector<point> path;
  for(int i = 0; i<catmull_spline.rows; i++)
  {
    point p;
    p.first = catmull_spline.at<float>(i,0);
    p.second = catmull_spline.at<float>(i,1);
    path.push_back(p);
  }
  return path;
}

}
