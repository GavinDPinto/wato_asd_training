#include "control_node.hpp"

ControlNode::ControlNode()
    : Node("control"), control_(robot::ControlCore(this->get_logger()))
{
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ControlNode::timerCallback, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    current_path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = msg;
}

void ControlNode::timerCallback() {
    if (!current_path_ || !current_odom_ || current_path_->poses.empty())
        return;

    auto lookahead = control_.findLookaheadPoint(*current_path_, *current_odom_, lookahead_distance_);
    if (!lookahead) {
        // Stop if no lookahead point found (goal reached or path invalid)
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        return;
    }

    geometry_msgs::msg::Twist cmd = control_.computeVelocity(*current_odom_, *lookahead, linear_speed_);
    // Stop if at goal
    if (control_.computeDistance(
            current_odom_->pose.pose.position, 
            current_path_->poses.back().pose.position) < goal_tolerance_) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
    }
    cmd_vel_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
