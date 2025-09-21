#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <optional>

namespace robot
{

class ControlCore {
public:
    explicit ControlCore(const rclcpp::Logger& logger);

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
        const nav_msgs::msg::Path& path,
        const nav_msgs::msg::Odometry& odom,
        double lookahead_distance);

    geometry_msgs::msg::Twist computeVelocity(
        const nav_msgs::msg::Odometry& odom,
        const geometry_msgs::msg::PoseStamped& target,
        double linear_speed);

    double computeDistance(
        const geometry_msgs::msg::Point& a,
        const geometry_msgs::msg::Point& b);

    double extractYaw(const geometry_msgs::msg::Quaternion& q);

private:
    rclcpp::Logger logger_;
};

}

#endif 
