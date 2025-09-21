#include "planner_core.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>

namespace robot
{

struct GridIndex {
    int x, y;
    bool operator==(const GridIndex& other) const { return x == other.x && y == other.y; }
    bool operator<(const GridIndex& other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }
};

struct GridIndexHash {
    std::size_t operator()(const GridIndex& idx) const {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

bool PlannerCore::planAStar(
    const nav_msgs::msg::OccupancyGrid& grid,
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::PointStamped& goal,
    nav_msgs::msg::Path& path)
{
    // Convert world to grid coordinates
    auto worldToGrid = [&](double wx, double wy, int& gx, int& gy) {
        gx = static_cast<int>((wx - grid.info.origin.position.x) / grid.info.resolution);
        gy = static_cast<int>((wy - grid.info.origin.position.y) / grid.info.resolution);
    };

    int width = grid.info.width;
    int height = grid.info.height;

    int start_x, start_y, goal_x, goal_y;
    worldToGrid(start.position.x, start.position.y, start_x, start_y);
    worldToGrid(goal.point.x, goal.point.y, goal_x, goal_y);

    auto isValid = [&](int x, int y) {
        return x >= 0 && y >= 0 && x < static_cast<int>(width) && y < static_cast<int>(height) &&
               grid.data[y * width + x] >= 0 && grid.data[y * width + x] < 30; // min cost to be considered an obstacle
    };

    auto heuristic = [&](int x, int y) {
        return std::hypot(goal_x - x, goal_y - y);
    };

    using PQElem = std::pair<double, GridIndex>;
    std::priority_queue<PQElem, std::vector<PQElem>, std::greater<PQElem>> open;
    std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
    std::unordered_map<GridIndex, double, GridIndexHash> cost_so_far;

    GridIndex start_idx{start_x, start_y};
    GridIndex goal_idx{goal_x, goal_y};
    open.emplace(0.0, start_idx);
    came_from[start_idx] = start_idx;
    cost_so_far[start_idx] = 0.0;

    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    bool found = false;
    while (!open.empty()) {
        auto [current_cost, current] = open.top();
        open.pop();

        if (current.x == goal_x && current.y == goal_y) {
            found = true;
            break;
        }

        for (int dir = 0; dir < 8; ++dir) {
            int nx = current.x + dx[dir];
            int ny = current.y + dy[dir];
            GridIndex neighbor{nx, ny};
            if (!isValid(nx, ny)) continue;
            double new_cost = cost_so_far[current] + ((dir % 2 == 0) ? 1.0 : std::sqrt(2.0));
            if (!cost_so_far.count(neighbor) || new_cost < cost_so_far[neighbor]) {
                cost_so_far[neighbor] = new_cost;
                double priority = new_cost + heuristic(nx, ny);
                open.emplace(priority, neighbor);
                came_from[neighbor] = current;
            }
        }
    }

    if (!found) {
        RCLCPP_WARN(logger_, "A* failed: No path found.");
        return false;
    }

    // Reconstruct path
    std::vector<GridIndex> grid_path;
    GridIndex current{goal_x, goal_y};
    while (!(current == start_idx)) {
        grid_path.push_back(current);
        current = came_from[current];
    }
    grid_path.push_back(start_idx);
    std::reverse(grid_path.begin(), grid_path.end());

    // Convert grid path to world coordinates and fill nav_msgs::msg::Path
    path.poses.clear();
    for (const auto& idx : grid_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = grid.header.frame_id;
        pose.pose.position.x = grid.info.origin.position.x + (idx.x + 0.5) * grid.info.resolution;
        pose.pose.position.y = grid.info.origin.position.y + (idx.y + 0.5) * grid.info.resolution;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    return true;
}

} 
