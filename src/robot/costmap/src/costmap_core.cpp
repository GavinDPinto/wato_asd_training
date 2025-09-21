#include "costmap_core.hpp"
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initializeCostmap() {
    
    for (int i = 0; i < grid_length_; ++i) {
        for (int j = 0; j < grid_length_; ++j) {
            grid_[i][j] = 0;
        }
    }
    
}

void CostmapCore::convertToGrid(double range, double angle, double robot_x, double robot_y, geometry_msgs::msg::Quaternion orientation, int &x_grid, int &y_grid) {
    // coordinates relative to robot
    double robot_rel_x = range * std::cos(angle);
    double robot_rel_y = range * std::sin(angle);

    double x = orientation.x;
    double y = orientation.y;
    double z = orientation.z;
    double w = orientation.w;
    // calculate yaw
    double robot_angle = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    // coordinates relative to field if robot was at origin
    double orig_field_rel_x = std::cos(robot_angle) * robot_rel_x - std::sin(robot_angle) * robot_rel_y;
    double orig_field_rel_y = std::sin(robot_angle) * robot_rel_x + std::cos(robot_angle) * robot_rel_y;

    // field relative coordinates
    double field_rel_x = orig_field_rel_x + robot_x;
    double field_rel_y = orig_field_rel_y + robot_y;

    // convert to array (grid) indices
    x_grid = (int) (field_rel_x / resolution_ + grid_length_ / 2);
    y_grid = (int) (field_rel_y / resolution_ + grid_length_ / 2);  
}

void CostmapCore::inflateObstacles() {
    const double inflation_radius = 1.5; // meters
    const int max_cost = 99;
    int inflation_cells = static_cast<int>(inflation_radius / resolution_);

    // Copy of the grid to avoid inflating already inflated cells
    int temp_grid[grid_length_][grid_length_];
    for (int i = 0; i < grid_length_; ++i)
        for (int j = 0; j < grid_length_; ++j)
            temp_grid[i][j] = grid_[i][j];

    for (int x = 0; x < grid_length_; ++x) {
        for (int y = 0; y < grid_length_; ++y) {
            if (grid_[x][y] == 100) { // Obstacle cell
                for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= grid_length_ || ny >= grid_length_)
                            continue;
                        double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                        if (distance > inflation_radius || distance == 0.0)
                            continue;
                        int cost = static_cast<int>(max_cost * (1.0 - (distance / inflation_radius)));
                        if (cost > temp_grid[nx][ny]) {
                            temp_grid[nx][ny] = cost;
                        }
                    }
                }
            }
        }
    }

    // Copy back to the main grid
    for (int i = 0; i < grid_length_; ++i)
        for (int j = 0; j < grid_length_; ++j)
            grid_[i][j] = temp_grid[i][j];
}


}