#include <chrono>
#include <memory>
#include <cmath>
#include "costmap_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  odom_sub_ = 
  this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, 
    std::bind(&CostmapNode::odomCallback, this, std::placeholders::_1)
  );
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, 
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1)
  );
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10);
  }
 
void CostmapNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_orientation_ = msg->pose.pose.orientation;
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Step 1: Initialize costmap
  costmap_.initializeCostmap();

  // Step 2: Convert LaserScan to grid and mark obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
      int x_grid, y_grid;
      // Calculate grid coordinates
      costmap_.convertToGrid(range, angle, robot_x_, robot_y_, robot_orientation_, x_grid, y_grid);
      // Mark obstacles
      if (x_grid >= 0 && x_grid < costmap_.grid_length_ && y_grid >= 0 && y_grid < costmap_.grid_length_) {
        costmap_.grid_[x_grid][y_grid] = 100;
      }
    }
  }

  // Step 3: Inflate obstacles
  costmap_.inflateObstacles();

  // Step 4: Publish costmap
  nav_msgs::msg::OccupancyGrid grid_msg;

  // Header
  grid_msg.header.stamp = this->now();
  grid_msg.header.frame_id = "sim_world"; 

  // Info
  grid_msg.info.resolution = costmap_.resolution_;
  grid_msg.info.width = costmap_.grid_length_;
  grid_msg.info.height = costmap_.grid_length_;
  grid_msg.info.origin.position.x = -15; // set as needed
  grid_msg.info.origin.position.y = -15; // set as needed
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  // Data (flatten 2D grid to 1D)
  grid_msg.data.resize(costmap_.grid_length_ * costmap_.grid_length_);
  for (int y = 0; y < costmap_.grid_length_; ++y) {
      for (int x = 0; x < costmap_.grid_length_; ++x) {
          grid_msg.data[y * costmap_.grid_length_ + x] = costmap_.grid_[x][y];
      }
  }

  costmap_pub_->publish(grid_msg);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}