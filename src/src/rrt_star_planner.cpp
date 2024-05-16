#include "rrt_planner/rrt_planner.h"

namespace rrt_planner
{

RRTPlanner::RRTPlanner(ros::NodeHandle * node)
: nh_(node),
  private_nh_("~"),
  map_received_(false),
  init_pose_received_(false),
  goal_received_(false),
  max_iterations(10000),
  step_size(1.0),
  goal_bias(0.1)
{
  // Get map and path topics from parameter server
  std::string map_topic, path_topic;
  private_nh_.param<std::string>("map_topic", map_topic, "/map");
  private_nh_.param<std::string>("path_topic", path_topic, "/path");

  // Get RRT parameters from parameter server
  private_nh_.param<int>("max_iterations", max_iterations, 10000);
  private_nh_.param<double>("step_size", step_size, 1.0);
  private_nh_.param<double>("goal_bias", goal_bias, 0.1);

  // Subscribe to map topic
  map_sub_ = nh_->subscribe<const nav_msgs::OccupancyGrid::Ptr &>(
    map_topic, 1, &RRTPlanner::mapCallback, this);

  // Subscribe to initial pose topic that is published by RViz
  init_pose_sub_ = nh_->subscribe<const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &>(
    "/initialpose", 1, &RRTPlanner::initPoseCallback, this);

  // Subscribe to goal topic that is published by RViz
  goal_sub_ = nh_->subscribe<const geometry_msgs::PoseStamped::ConstPtr &>(
    "/move_base_simple/goal", 1, &RRTPlanner::goalCallback, this);

  // Advertise topic where calculated path is going to be published
  path_pub_ = nh_->advertise<nav_msgs::Path>(path_topic, 1, true);

  // Initialize marker publishers
  init_pose_marker_pub_ = nh_->advertise<visualization_msgs::Marker>("initial_pose_marker", 1);
  goal_marker_pub_ = nh_->advertise<visualization_msgs::Marker>("goal_marker", 1);

  // This loops until the node is running, will exit when the node is killed
  while (ros::ok()) {
    // if map, initial pose, and goal have been received
    // build the map image, draw initial pose and goal, and plan
    if (map_received_ && init_pose_received_ && goal_received_) {
      buildMapImage();
      drawGoalInitPose();
      plan();
    } else {
      if (map_received_) {
        displayMapImage();
      }
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
  }
}

void RRTPlanner::mapCallback(const nav_msgs::OccupancyGrid::Ptr & msg)
{
  map_grid_ = msg;

  // Build and display the map image
  buildMapImage();
  displayMapImage();

  // Reset these values for a new planning iteration
  map_received_ = true;
  init_pose_received_ = false;
  goal_received_ = false;

  ROS_INFO("Map obtained successfully. Please provide initial pose and goal through RViz.");
}

void RRTPlanner::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
  if (init_pose_received_) {
    buildMapImage();
  }

  // Convert msg to Point2D
  poseToPoint(init_pose_, msg->pose.pose);

  // Reject the initial pose if the given point is occupied in the map
  if (!isPointUnoccupied(init_pose_)) {
    init_pose_received_ = false;
    ROS_WARN(
      "The initial pose specified is on or too close to an obstacle please specify another point");
  } else {
    init_pose_received_ = true;
    drawGoalInitPose();
    ROS_INFO("Initial pose obtained successfully.");
    std::cout << "Initial Pose x: " << init_pose_.x() << " m & Initial Pose y: " << init_pose_.y() << " m" << std::endl;
  }

  displayMapImage();
}

void RRTPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
  if (goal_received_) {
    buildMapImage();
  }

  // Convert msg to Point2D
  poseToPoint(goal_, msg->pose);

  // Reject the goal pose if the given point is occupied in the map
  if (!isPointUnoccupied(goal_)) {
    goal_received_ = false;
    ROS_WARN("The goal specified is on or too close to an obstacle please specify another point");
  } else {
    goal_received_ = true;
    drawGoalInitPose();
    ROS_INFO("Goal obtained successfully.");
    std::cout << "Goal Pose x: " << goal_.x() << " m & Goal Pose y: " << goal_.y() << " m" << std::endl;
  }

  displayMapImage();
}

void RRTPlanner::drawGoalInitPose()
{
  if (goal_received_) {
    drawCircle(goal_, 3, cv::Scalar(0, 255, 0));

    // Create and publish goal marker
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = map_grid_->header.frame_id;
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.ns = "goal_marker";
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose.position.x = goal_.y() * map_grid_->info.resolution;
    goal_marker.pose.position.y = goal_.x() * map_grid_->info.resolution;
    goal_marker.pose.position.z = 0.0;
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 0.75;
    goal_marker.scale.y = 0.75;
    goal_marker.scale.z = 0.75;
    goal_marker.color.r = 0.0f;
    goal_marker.color.g = 1.0f;
    goal_marker.color.b = 0.0f;
    goal_marker.color.a = 1.0f;
    goal_marker_pub_.publish(goal_marker);

  }
  if (init_pose_received_) {
    drawCircle(init_pose_, 3, cv::Scalar(255, 0, 0));

    // Create and publish initial pose marker
    visualization_msgs::Marker init_pose_marker;
    init_pose_marker.header.frame_id = map_grid_->header.frame_id;
    init_pose_marker.header.stamp = ros::Time::now();
    init_pose_marker.ns = "init_pose_marker";
    init_pose_marker.id = 1;
    init_pose_marker.type = visualization_msgs::Marker::SPHERE;
    init_pose_marker.action = visualization_msgs::Marker::ADD;
    init_pose_marker.pose.position.x = init_pose_.y() * map_grid_->info.resolution;
    init_pose_marker.pose.position.y = init_pose_.x() * map_grid_->info.resolution;
    init_pose_marker.pose.position.z = 0;
    init_pose_marker.pose.orientation.w = 1.0;
    init_pose_marker.scale.x = 0.75;
    init_pose_marker.scale.y = 0.75;
    init_pose_marker.scale.z = 0.75;
    init_pose_marker.color.r = 1.0f;
    init_pose_marker.color.g = 0.0f;
    init_pose_marker.color.b = 0.0f;
    init_pose_marker.color.a = 1.0f;
    init_pose_marker_pub_.publish(init_pose_marker);
  }
}

void RRTPlanner::plan()
{
  // Reset these values so planning only happens once for a
  // given pair of initial pose and goal points
  goal_received_ = false;
  init_pose_received_ = false;

  std::vector<Point2D> tree;
  std::map<int, int> parent_map;
  std::vector<double> cost;

  tree.push_back(init_pose_);
  cost.push_back(0.0);

  bool goal_reached = false;

  // RRT algorithm
  for(int i = 0; i < max_iterations; ++i){

    // Generate a random point
    Point2D rand_point;
    if((double)rand() / RAND_MAX < goal_bias){
      rand_point = goal_;
    } else {
      rand_point = Point2D(rand() % map_grid_->info.height, rand() % map_grid_->info.width);
    }

    int nearest_index = 0;
    double min_distance = (tree[nearest_index] - rand_point).norm();
    for(size_t j = 0; j < tree.size(); ++j) {
      double distance = (tree[j] - rand_point).norm();
      if (distance < min_distance) {
        min_distance = distance;
        nearest_index = j;
      }
    }

    Point2D nearest_point = tree[nearest_index];

    // Extend the tree towards the random point
    Point2D direction = (rand_point - nearest_point).normalized();
    Point2D new_point = nearest_point + direction * step_size;

    if(isPointUnoccupied(new_point) && isLineUnoccupied(nearest_point, new_point)) {
      // Check if new_point is not already in the tree
      bool point_exists = false;
      for(const auto& point : tree) {
        if ((point - new_point).norm() < 1e-6) {
          point_exists = true;
          break;
        }
      }
      if (!point_exists) {
        tree.push_back(new_point);
        parent_map[tree.size() - 1] = nearest_index;
        cost.push_back(cost[nearest_index] + costFromNode(nearest_point, new_point));
        drawLine(nearest_point, new_point, cv::Scalar(0,0,255),1);

        rewire(new_point, tree, parent_map, cost);

        if((new_point - goal_).norm() < step_size) {
          // Goal Reached
          goal_reached = true;
          ROS_INFO("GOAL REACHED!");
          std::vector<Point2D> path;
          int current_index = tree.size() - 1;
          while(tree[current_index] != init_pose_) {
            path.push_back(tree[current_index]);
            current_index = parent_map[current_index];
          }
          path.push_back(init_pose_);
          std::reverse(path.begin(), path.end());
          calculated_path_ = path;
          publishPath();
          ROS_INFO("Path found successfully!");
          return;
        }
      }
    }
  }
  if(!goal_reached) {
  ROS_WARN("Failed to find a path to the goal within the allowed iterations!");
  }
}

void RRTPlanner::publishPath()
{
  // Create new Path msg
  nav_msgs::Path path;
  path.header.frame_id = map_grid_->header.frame_id;
  path.header.stamp = ros::Time::now();

  for(const auto& point : calculated_path_) {
    geometry_msgs::PoseStamped pose = pointToPose(point);
    path.poses.push_back(pose);
  }

  // Publish the calculated path
  path_pub_.publish(path);

  displayMapImage();
}

bool RRTPlanner::isPointUnoccupied(const Point2D & p)
{
  int index = toIndex(p.x(), p.y());
  if(index < 0 || index >= map_grid_->data.size()) {
    return false;
  }

  return map_grid_->data[index] == 0;
}

bool RRTPlanner::isLineUnoccupied(const Point2D & p1, const Point2D & p2)
{
  // Implement Bresenham's line algorithm to check if a line between two points occupied or not
  int x0 = p1.x(), y0 = p1.y();
  int x1 = p2.x(), y1 = p2.y();
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy, e2;

  while(true) {
    if(!isPointUnoccupied(Point2D(x0, y0))) return false;
    if(x0 == x1 && y0 == y1) break;
    e2 = 2 * err;
    if(e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if(e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }

  return true;
}

double RRTPlanner::costToNode(const Point2D &p, const std::map<int, int> &parent_map, const std::vector<Point2D> &tree) {
  double total_cost = 0.0;
  auto it = std::find(tree.begin(), tree.end(), p);
  if (it == tree.end()) return std::numeric_limits<double>::infinity();
  auto current_index = std::distance(tree.begin(), it);
  while(current_index != 0) {
    if (parent_map.find(current_index) == parent_map.end()) break;
    total_cost += costFromNode(tree[current_index], tree[parent_map.at(current_index)]);
    current_index = parent_map.at(current_index);
  }
  return total_cost;
}

double RRTPlanner::costFromNode(const Point2D &p1, const Point2D &p2) {
  return (p1 - p2).norm();
}

void RRTPlanner::rewire(const Point2D &new_point, std::vector<Point2D> &tree, std::map<int, int> &parent_map, std::vector<double> &cost) {
  const double search_radius = step_size;
  int new_point_index = std::find(tree.begin(), tree.end(), new_point) - tree.begin();
  
  for(int i = 0; i < tree.size(); i++) {
    if (i == new_point_index) continue;
    if ((tree[i] - new_point).norm() <= search_radius && isLineUnoccupied(tree[i], new_point)) {
      double new_cost = cost[new_point_index] + costFromNode(new_point, tree[i]);
      if (new_cost < cost[i]) {
        // Check for cycle creation
        int parent_check_index = new_point_index;
        bool creates_cycle = false;
        while (parent_map.find(parent_check_index) != parent_map.end()) {
          if (parent_map[parent_check_index] == i) {
            creates_cycle = true;
            break;
          }
          parent_check_index = parent_map[parent_check_index];
        }
        if (creates_cycle) {
          ROS_WARN("Cycle detected! Skipping rewire for node %d with new parent %d", i, new_point_index);
          continue;
        }

        parent_map[i] = new_point_index;
        cost[i] = new_cost;
        drawLine(new_point, tree[i], cv::Scalar(0, 255, 0), 1);
        // ROS_INFO("Rewiring node %d with new parent %d and cost %.2f", i, new_point_index, new_cost);
      }
    }
  }
}

void RRTPlanner::buildMapImage()
{
  // Create a new opencv matrix with the same height and width as the received map
  map_ = std::unique_ptr<cv::Mat>(new cv::Mat(map_grid_->info.height,
                                              map_grid_->info.width,
                                              CV_8UC3,
                                              cv::Scalar::all(255)));

  // Fill the opencv matrix pixels with the map points
  for (int i = 0; i < map_grid_->info.height; i++) {
    for (int j = 0; j < map_grid_->info.width; j++) {
      if (map_grid_->data[toIndex(i, j)]) {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(0, 0, 0);
      } else {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(255, 255, 255);
      }
    }
  }
}

void RRTPlanner::displayMapImage(int delay)
{
  cv::imshow("Output", *map_);
  cv::waitKey(delay);
}

void RRTPlanner::drawCircle(Point2D & p, int radius, const cv::Scalar & color)
{
  cv::circle(
    *map_,
    cv::Point(p.y(), map_grid_->info.height - p.x() - 1),
    radius,
    color,
    -1);
}

void RRTPlanner::drawLine(const Point2D & p1, const Point2D & p2, const cv::Scalar & color, int thickness)
{
  cv::line(
    *map_,
    cv::Point(p2.y(), map_grid_->info.height - p2.x() - 1),
    cv::Point(p1.y(), map_grid_->info.height - p1.x() - 1),
    color,
    thickness);
}

inline geometry_msgs::PoseStamped RRTPlanner::pointToPose(const Point2D & p)
{
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = p.y() * map_grid_->info.resolution;
  pose.pose.position.y = p.x() * map_grid_->info.resolution;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = map_grid_->header.frame_id;
  return pose;
}

inline void RRTPlanner::poseToPoint(Point2D & p, const geometry_msgs::Pose & pose)
{
  p.x(pose.position.y / map_grid_->info.resolution);
  p.y(pose.position.x / map_grid_->info.resolution);
}

inline int RRTPlanner::toIndex(int x, int y)
{
  return x * map_grid_->info.width + y;
}

}  // namespace rrt_planner
