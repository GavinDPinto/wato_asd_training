#include "map_memory_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : logger_(logger) {}

void MapMemoryCore::initializeGlobalMap(const nav_msgs::msg::OccupancyGrid & costmap, double origin_x, double origin_y) {
  global_map_ = costmap;
  global_map_.header.frame_id = "sim_world";
  global_map_.info.origin.position.x = origin_x;
  global_map_.info.origin.position.y = origin_y;
  global_map_initialized_ = true;
}

void MapMemoryCore::mergeCostmap(const nav_msgs::msg::OccupancyGrid & costmap) {
    if (!global_map_initialized_) return;

    int width = costmap.info.width;
    int height = costmap.info.height;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;

            global_map_.data[idx] = 
                std::max(global_map_.data[idx], costmap.data[idx]); 
        }
    }
}

void MapMemoryCore::decayMemory() {
    if (!global_map_initialized_) return;

    for (auto &cell : global_map_.data) {
        if (cell > 0 && cell <= 100) {
            cell = std::max(0, static_cast<int>(cell - decay_rate_));
        }
    }
}

} 
