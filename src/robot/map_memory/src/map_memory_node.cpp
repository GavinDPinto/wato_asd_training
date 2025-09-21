#include <chrono>
#include <memory>
#include "map_memory_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  
  // Map Publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Odom Subscription
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  // Costmap subscription
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MapMemoryNode::timerCallback, this)
  );
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  
  last_costmap_ = *msg;
  have_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

  double x_current = msg->pose.pose.position.x;
  double y_current = msg->pose.pose.position.y;

  if (first_odom_) {
    last_x_ = x_current;
    last_y_ = y_current;
    first_odom_ = false;
    return;
  }

  double distance = std::sqrt(
    std::pow(x_current - last_x_, 2) +
    std::pow(y_current - last_y_, 2)
  );

  RCLCPP_INFO(this->get_logger(), "Distance since last update: %f", distance);

  if (distance >= 1.5 && have_costmap_) {
    if (!map_memory_.getGlobalMap().data.size()) {
      map_memory_.initializeGlobalMap(last_costmap_, -15.0, -15.0); // or set origin as needed
    }
    map_memory_.mergeCostmap(last_costmap_);
    last_x_ = x_current;
    last_y_ = y_current;
    have_costmap_ = false; // Only merge once per movement
    should_update_map_ = true;
    RCLCPP_INFO(this->get_logger(), "Merged costmap into global map.");
  }


}

void MapMemoryNode::timerCallback() {
  RCLCPP_INFO(this->get_logger(), "Timer callback triggered");
  if (map_memory_.getGlobalMap().data.size() && should_update_map_) {
      map_pub_->publish(map_memory_.getGlobalMap());
      should_update_map_ = false;
      RCLCPP_INFO(this->get_logger(), "Published global map!");
  } else {
      RCLCPP_WARN(this->get_logger(), "Global map is empty, not publishing");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
