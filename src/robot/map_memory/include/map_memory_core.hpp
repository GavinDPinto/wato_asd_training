#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    void initializeGlobalMap(const nav_msgs::msg::OccupancyGrid & costmap, double origin_x, double origin_y);
    void mergeCostmap(const nav_msgs::msg::OccupancyGrid & costmap);
    const nav_msgs::msg::OccupancyGrid &getGlobalMap() const { 
      return global_map_; 
    }

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
    bool global_map_initialized_ = false;
};

}  

#endif  
