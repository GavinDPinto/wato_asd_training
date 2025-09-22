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
  if (!first_costmap_received_) {
    first_costmap_received_ = true;
    first_costmap_time_ = this->now();
  }
  if (!map_memory_.getGlobalMap().data.size()) {
    map_memory_.initializeGlobalMap(last_costmap_, -15.0, -15.0); // or set origin as needed
  }

  map_memory_.decayMemory();

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

  if (distance >= 1.5 && have_costmap_) { // ensures a memory map is published on initialization
    map_memory_.mergeCostmap(last_costmap_);
    last_x_ = x_current;
    last_y_ = y_current;
    should_update_map_ = true;
    RCLCPP_INFO(this->get_logger(), "Merged costmap into global map.");
  }
}

void MapMemoryNode::timerCallback() {
  RCLCPP_INFO(this->get_logger(), "Timer callback triggered");
  if (!first_costmap_received_) return;

  // Wait for 500 ms after first costmap
  if ((this->now() - first_costmap_time_).seconds() * 1000.0 < initial_publish_delay_ms_) {
    return;
  }

  // Now safe to publish
  if (should_update_map_ && map_memory_.getGlobalMap().data.size() > 0) {
    map_memory_.mergeCostmap(last_costmap_);
    have_costmap_ = false; // Only merge once per movement
    map_pub_->publish(map_memory_.getGlobalMap());
    should_update_map_ = false;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
