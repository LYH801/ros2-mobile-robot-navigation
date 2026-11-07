#include "astar_planner/astar_planner.hpp"

#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace astar_planner
{

struct GridIndex {
    int x;
    int y;
    bool operator==(const GridIndex& other) const {
        return x == other.x && y == other.y;
    }
};

struct Point2D {
    double x, y;
};

void AStarPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    global_frame_ = costmap_ros_->getGlobalFrameID();

    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
    node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);
}

void AStarPlanner::cleanup() {}
void AStarPlanner::activate() {}
void AStarPlanner::deactivate() {}

static double heuristic(const GridIndex& a, const GridIndex& b)
{
    double dx = static_cast<double>(a.x - b.x);
    double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

nav_msgs::msg::Path AStarPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = node_->now();

    unsigned int mx_start, my_start, mx_goal, my_goal;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
        RCLCPP_WARN(node_->get_logger(), "Start out of map bounds");
        return path;
    }
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
        RCLCPP_WARN(node_->get_logger(), "Goal out of map bounds");
        return path;
    }

    int width = costmap_->getSizeInCellsX();
    int height = costmap_->getSizeInCellsY();

    auto idx = [width](const GridIndex& p) { return p.y * width + p.x; };

    // 【修改1：修正 allow_unknown 逻辑】
    auto is_obstacle = [&](int x, int y) {
        if (x < 0 || x >= width || y < 0 || y >= height) return true;
        unsigned char c = costmap_->getCost(x, y);
        if (allow_unknown_) {
            return c >= 253; // 253为内切，254为致命；允许未知但不可穿致命障碍
        }
        return c >= 128; // 不允许未知，则非空闲即障碍
    };

    auto move_cost = [&](const GridIndex& a, const GridIndex& b) {
        int dx = std::abs(a.x - b.x);
        int dy = std::abs(a.y - b.y);
        double base = (dx == 1 && dy == 1) ? 1.414 : 1.0;
        unsigned char c = costmap_->getCost(b.x, b.y);
        return base * (1.0 + c / 100.0);
    };

    auto get_neighbors = [&](const GridIndex& node) {
        std::vector<GridIndex> result;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                GridIndex n{node.x + dx, node.y + dy};
                if (is_obstacle(n.x, n.y)) continue;
                if (dx != 0 && dy != 0) {
                    if (is_obstacle(node.x + dx, node.y) || is_obstacle(node.x, node.y + dy)) {
                        continue; // 禁止穿角
                    }
                }
                result.push_back(n);
            }
        }
        return result;
    };

    GridIndex start_idx{static_cast<int>(mx_start), static_cast<int>(my_start)};
    GridIndex goal_idx{static_cast<int>(mx_goal), static_cast<int>(my_goal)};

    struct Node { GridIndex index; double g; double f; };
    auto cmp = [](const Node& a, const Node& b) { return a.f > b.f; };

    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);
    std::unordered_map<int, double> g_score;
    std::unordered_map<int, GridIndex> parent;
    std::vector<bool> closed(width * height, false);

    open.push({start_idx, 0.0, heuristic(start_idx, goal_idx)});
    g_score[idx(start_idx)] = 0.0;

    bool found = false;
    while (!open.empty()) {
        Node current = open.top(); open.pop();
        if (closed[idx(current.index)]) continue;
        closed[idx(current.index)] = true;

        if (current.index == goal_idx) { found = true; break; }

        for (const auto& neighbor : get_neighbors(current.index)) {
            if (closed[idx(neighbor)]) continue;
            double tentative_g = current.g + move_cost(current.index, neighbor);
            auto it = g_score.find(idx(neighbor));
            if (it == g_score.end() || tentative_g < it->second) {
                g_score[idx(neighbor)] = tentative_g;
                parent[idx(neighbor)] = current.index;
                open.push({neighbor, tentative_g, tentative_g + heuristic(neighbor, goal_idx)});
            }
        }
    }

    if (!found) {
        RCLCPP_WARN(node_->get_logger(), "A* failed to find a path");
        return path;
    }

    std::vector<GridIndex> reverse_path;
    GridIndex current = goal_idx;
    while (!(current == start_idx)) {
        reverse_path.push_back(current);
        auto it = parent.find(idx(current));
        if (it == parent.end()) break;
        current = it->second;
    }
    reverse_path.push_back(start_idx);
    std::reverse(reverse_path.begin(), reverse_path.end());

    const int DOWNSAMPLE = 3;
    std::vector<GridIndex> sampled;
    for (size_t i = 0; i < reverse_path.size(); i += DOWNSAMPLE) sampled.push_back(reverse_path[i]);
    if (sampled.empty() || !(sampled.back() == reverse_path.back())) sampled.push_back(reverse_path.back());

    std::vector<Point2D> pts;
    for (const auto& p : sampled) {
        double wx, wy; costmap_->mapToWorld(p.x, p.y, wx, wy);
        pts.push_back({wx, wy});
    }

    // 【修改2：贝塞尔平滑加入碰撞复检】
    const int CURVE_STEPS = 5;
    std::vector<Point2D> smooth_pts;
    if (pts.size() < 3) {
        smooth_pts = pts;
    } else {
        smooth_pts.push_back(pts[0]);
        for (size_t i = 1; i + 1 < pts.size(); ++i) {
            Point2D P0 = pts[i-1], P1 = pts[i], P2 = pts[i+1];
            Point2D C0 = {(P0.x + P1.x) / 2.0, (P0.y + P1.y) / 2.0};
            Point2D C1 = {(P1.x + P2.x) / 2.0, (P1.y + P2.y) / 2.0};
            
            bool point_safe = true;
            for (int k = 0; k <= CURVE_STEPS; ++k) {
                double t = (double)k / CURVE_STEPS;
                double mt = 1.0 - t;
                Point2D B;
                B.x = mt*mt*C0.x + 2*mt*t*P1.x + t*t*C1.x;
                B.y = mt*mt*C0.y + 2*mt*t*P1.y + t*t*C1.y;
                
                unsigned int bx, by;
                if (costmap_->worldToMap(B.x, B.y, bx, by)) {
                    if (costmap_->getCost(bx, by) >= 253) { point_safe = false; break; } // 撞墙保护
                }
                smooth_pts.push_back(B);
            }
            if (!point_safe) {
                // 如果平滑切进障碍，清空这轮平滑点，回退使用原始 P1
                smooth_pts.resize(smooth_pts.size() - (CURVE_STEPS + 1));
                smooth_pts.push_back(P1);
            }
        }
        smooth_pts.push_back(pts.back());
    }

    for (size_t i = 0; i < smooth_pts.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = global_frame_;
        pose.header.stamp = node_->now();
        pose.pose.position.x = smooth_pts[i].x;
        pose.pose.position.y = smooth_pts[i].y;
        pose.pose.position.z = 0.0;

        double yaw = 0.0;
        if (i + 1 < smooth_pts.size()) yaw = std::atan2(smooth_pts[i+1].y - smooth_pts[i].y, smooth_pts[i+1].x - smooth_pts[i].x);
        else if (i > 0) yaw = std::atan2(smooth_pts[i].y - smooth_pts[i-1].y, smooth_pts[i].x - smooth_pts[i-1].x);

        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        path.poses.push_back(pose);
    }

    if (!path.poses.empty()) {
        path.poses.back().pose.position.x = goal.pose.position.x;
        path.poses.back().pose.position.y = goal.pose.position.y;
    }

    RCLCPP_INFO(node_->get_logger(), "A* path size: %zu", path.poses.size());
    return path;
}
}  // namespace astar_planner
PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, nav2_core::GlobalPlanner)
