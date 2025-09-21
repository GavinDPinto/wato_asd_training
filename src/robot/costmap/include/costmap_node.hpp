#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    // void publishMessage();

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void odomCallback(const
    nav_msgs::msg::Odometry::SharedPtr msg);
 
  private:
    double robot_x_, robot_y_;
    geometry_msgs::msg::Quaternion robot_orientation_;
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};
 
#endif 