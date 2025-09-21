#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeCostmap();
    void convertToGrid(double range, double angle, double robot_x, double robot_y, geometry_msgs::msg::Quaternion orientation, int &x_grid, int &y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();


  private:
    rclcpp::Logger logger_;
  
  public:
    const static int grid_length_ = 300;
    constexpr static double resolution_ = 0.1;
    int grid_[grid_length_][grid_length_];

};

}  

#endif  