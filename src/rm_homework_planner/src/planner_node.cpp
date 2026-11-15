#include <memory>
#include <optional>
#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <limits>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using Grid = nav_msgs::msg::OccupancyGrid;

struct Cell{
    int x, y;
};

std::optional<Cell> worldToCell(const Grid& map, double x, double y){
    const auto& origin = map.info.origin.position;
    const double gx = std::floor((x - origin.x) / map.info.resolution);
    const double gy = std::floor((y - origin.y) / map.info.resolution);

    if(gx < 0 || gx >= map.info.width || gy < 0 || gy >= map.info.height){
        return std::nullopt;
    }

    return Cell{static_cast<int>(gx), static_cast<int>(gy)};
}

std::vector<Cell> aStar(const Grid& map, const Cell& start, const Cell& goal){
    const int width = map.info.width;
    const int height = map.info.height;

    auto id = [width](const Cell& cell) { return cell.y * width + cell.x; };
    auto h = [goal](const Cell& cell) { return 100 * std::abs(cell.x - goal.x) + 100 * std::abs(cell.y - goal.y); };
    auto blocked = [&map](int i) {return map.data[i] < 0 || map.data[i] >= 99;};

    const int source = id(start);
    const int target = id(goal);
    if(blocked(source) || blocked(target)){
        return {};
    }

    const int INF = std::numeric_limits<int>::max();
    std::vector<int> best(width * height, INF);
    std::vector<int> parent(width * height, -1);

    using Entry = std::tuple<int, int, int>; // f, g, id
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;
    best[source] = 0;
    open.emplace(h(start), 0, source);

    const std::vector<Cell> directions{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    while(!open.empty()){
        auto [f, g, current] = open.top();
        open.pop();

        if(current == target){
            std::vector<Cell> path;
            for(int i = current; i != -1; i = parent[i]){
                path.push_back(Cell{i % width, i / width});
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        if(g > best[current]){
            continue;
        }

        const Cell cell{current % width, current / width};
        for(const auto& dir : directions){
            Cell neighbor{cell.x + dir.x, cell.y + dir.y};
            if(neighbor.x < 0 || neighbor.x >= width || neighbor.y < 0 || neighbor.y >= height){
                continue;
            }

            int neighbor_id = id(neighbor);
            if(blocked(neighbor_id)){
                continue;
            }

            int new_g = g + 100 + map.data[neighbor_id];
            if(new_g < best[neighbor_id]){
                best[neighbor_id] = new_g;
                parent[neighbor_id] = current;
                open.emplace(new_g + h(neighbor), new_g, neighbor_id);
            }
        }
    }

    return {};
}

class HomeworkPlanner : public rclcpp::Node {
    using Click = geometry_msgs::msg::PointStamped;
    using Path = nav_msgs::msg::Path;

public:
    HomeworkPlanner() : Node("HomeworkPlannner"){
        path_pub_ = create_publisher<Path>("/homework_path", 10);

        click_sub_ = create_subscription<Click>("/clicked_point", 10, 
        [this](Click::ConstSharedPtr msg){
            onClick(*msg);
        });

        map_sub_ = create_subscription<Grid>("/costmap/costmap", rclcpp::QoS(1).transient_local(),
        [this](Grid::ConstSharedPtr msg){
            map_ = msg;
        });

        RCLCPP_INFO(get_logger(), "Waiting for two clicks from RViz");
    }

private:
    void onClick(const Click& click)
    {
        if(!map_){
            RCLCPP_WARN(get_logger(), "No map received yet");
            return;
        }

        auto cell = worldToCell(*map_, click.point.x, click.point.y);
        if(!cell){
            RCLCPP_WARN(get_logger(), "Clicked point is outside the map");
            return;
        }

        if(!start_){
            start_ = click;
            RCLCPP_INFO(get_logger(), "Start point set at (%.2f, %.2f)", click.point.x, click.point.y);
        } else {
            auto goal_cell = worldToCell(*map_, click.point.x, click.point.y);
            if(!goal_cell){
                RCLCPP_WARN(get_logger(), "Goal point is outside the map");
                return;
            }

            auto path_cells = aStar(*map_, *worldToCell(*map_, start_->point.x, start_->point.y), *goal_cell);
            if(path_cells.empty()){
                RCLCPP_WARN(get_logger(), "No path found between the two points");
                start_.reset();
                return;
            }

            Path path_msg;
            path_msg.header.stamp = this->now();
            path_msg.header.frame_id = map_->header.frame_id;

            for(const auto& cell : path_cells){
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_msg.header;
                pose.pose.position.x = map_->info.origin.position.x + (cell.x + 0.5) * map_->info.resolution;
                pose.pose.position.y = map_->info.origin.position.y + (cell.y + 0.5) * map_->info.resolution;
                pose.pose.orientation.w = 1.0; // No rotation
                path_msg.poses.push_back(pose);
            }

            path_pub_->publish(path_msg);
            RCLCPP_INFO(get_logger(), "Path published with %zu points", path_msg.poses.size());
            start_.reset();
        }
    }

    std::optional<Click> start_;
    rclcpp::Publisher<Path>::SharedPtr path_pub_;
    rclcpp::Subscription<Click>::SharedPtr click_sub_;
    rclcpp::Subscription<Grid>::SharedPtr map_sub_;
    Grid::ConstSharedPtr map_;

};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HomeworkPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
