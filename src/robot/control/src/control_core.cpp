#include "control_core.hpp"
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
    : logger_(logger) {}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path& path,
    const nav_msgs::msg::Odometry& odom,
    double lookahead_distance)
{
    const auto& robot_pos = odom.pose.pose.position;
    for (const auto& pose_stamped : path.poses) {
        double dist = computeDistance(robot_pos, pose_stamped.pose.position);
        if (dist >= lookahead_distance) {
            return pose_stamped;
        }
    }
    // If no point is far enough, return the last point (goal)
    if (!path.poses.empty())
        return path.poses.back();
    return std::nullopt;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const nav_msgs::msg::Odometry& odom,
    const geometry_msgs::msg::PoseStamped& target,
    double linear_speed)
{
    geometry_msgs::msg::Twist cmd;
    const auto& robot_pos = odom.pose.pose.position;
    double robot_yaw = extractYaw(odom.pose.pose.orientation);

    double dx = target.pose.position.x - robot_pos.x;
    double dy = target.pose.position.y - robot_pos.y;

    double target_angle = std::atan2(dy, dx);
    double alpha = target_angle - robot_yaw;

    // Normalize alpha to [-pi, pi]
    while (alpha > M_PI) alpha -= 2 * M_PI;
    while (alpha < -M_PI) alpha += 2 * M_PI;

    cmd.linear.x = linear_speed;
    cmd.angular.z = 2.0 * linear_speed * std::sin(alpha) / std::hypot(dx, dy);

    return cmd;
}

double ControlCore::computeDistance(
    const geometry_msgs::msg::Point& a,
    const geometry_msgs::msg::Point& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q)
{
    // Standard ROS quaternion to yaw
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}
