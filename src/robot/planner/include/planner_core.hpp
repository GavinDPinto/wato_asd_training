#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_core.hpp"

namespace robot
{

class PlannerCore {
public:
    PlannerCore(const rclcpp::Logger& logger);

    // Add this method:
    bool planAStar(
        const nav_msgs::msg::OccupancyGrid& grid,
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::PointStamped& goal,
        nav_msgs::msg::Path& path);

private:
    rclcpp::Logger logger_;
};

}  

#endif  
