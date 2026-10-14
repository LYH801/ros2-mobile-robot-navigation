#include "pursuit_controller/pursuit_controller.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace pursuit_controller
{

void PursuitController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    global_frame_ = costmap_ros_->getGlobalFrameID();

    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.4));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".max_lookahead_dist", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".max_linear_speed", rclcpp::ParameterValue(0.3));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".min_linear_speed", rclcpp::ParameterValue(0.05));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".max_angular_speed", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".goal_tolerance", rclcpp::ParameterValue(0.15));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".use_velocity_scaled_lookahead",
        rclcpp::ParameterValue(true));

    node_->get_parameter(name_ + ".min_lookahead_dist", min_lookahead_dist_);
    node_->get_parameter(name_ + ".max_lookahead_dist", max_lookahead_dist_);
    node_->get_parameter(name_ + ".lookahead_time", lookahead_time_);
    node_->get_parameter(name_ + ".max_linear_speed", max_linear_speed_);
    node_->get_parameter(name_ + ".min_linear_speed", min_linear_speed_);
    node_->get_parameter(name_ + ".max_angular_speed", max_angular_speed_);
    node_->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);
    node_->get_parameter(name_ + ".use_velocity_scaled_lookahead",
                         use_velocity_scaled_lookahead_);

    RCLCPP_INFO(node_->get_logger(), "PursuitController configured");
}

void PursuitController::cleanup()
{
    RCLCPP_INFO(node_->get_logger(), "PursuitController cleanup");
}

void PursuitController::activate()
{
    RCLCPP_INFO(node_->get_logger(), "PursuitController activated");
}

void PursuitController::deactivate()
{
    RCLCPP_INFO(node_->get_logger(), "PursuitController deactivated");
}

void PursuitController::setPlan(const nav_msgs::msg::Path& path)
{
    global_plan_ = path;
}

void PursuitController::setSpeedLimit(
    const double& speed_limit,
    const bool& percentage)
{
    if (percentage) {
        max_linear_speed_ = 0.3 * speed_limit / 100.0;
    } else {
        max_linear_speed_ = speed_limit;
    }
}

static inline double quaternionToYaw(
    const geometry_msgs::msg::Quaternion& q)
{
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::TwistStamped PursuitController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& /*velocity*/,
    nav2_core::GoalChecker* /*goal_checker*/)
{
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = "base_link";
    cmd_vel.header.stamp = node_->now();

    if (global_plan_.poses.empty() || global_plan_.poses.size() < 2) {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
    }
    
    // 检查 pose 是否有效（frame_id 必须是 map 或 global_frame）
    if (pose.header.frame_id != global_frame_) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "Pursuit: pose frame_id is '%s', expected '%s'. Stopping.",
            pose.header.frame_id.c_str(), global_frame_.c_str());
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
    }

    double robot_x = pose.pose.position.x;
    double robot_y = pose.pose.position.y;
    double robot_yaw = quaternionToYaw(pose.pose.orientation);

    // ========== 步骤 1：找路径上离机器人最近的点 ==========
    size_t closest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
        double dx = global_plan_.poses[i].pose.position.x - robot_x;
        double dy = global_plan_.poses[i].pose.position.y - robot_y;
        double d_sq = dx * dx + dy * dy;
        if (d_sq < min_dist_sq) {
            min_dist_sq = d_sq;
            closest_idx = i;
        }
    }

    // ========== 步骤 2：从最近点向前找预瞄点 ==========
    double lookahead = min_lookahead_dist_;
    if (use_velocity_scaled_lookahead_) {
        lookahead = max_linear_speed_ * lookahead_time_;
        lookahead = std::clamp(lookahead,
                               min_lookahead_dist_, max_lookahead_dist_);
    }

    double accumulated = 0.0;
    double lookahead_x = global_plan_.poses[closest_idx].pose.position.x;
    double lookahead_y = global_plan_.poses[closest_idx].pose.position.y;
    bool found = false;

    for (size_t i = closest_idx + 1; i < global_plan_.poses.size(); ++i) {
        double px0 = global_plan_.poses[i-1].pose.position.x;
        double py0 = global_plan_.poses[i-1].pose.position.y;
        double px1 = global_plan_.poses[i].pose.position.x;
        double py1 = global_plan_.poses[i].pose.position.y;
        double dx = px1 - px0;
        double dy = py1 - py0;
        double seg_len = std::sqrt(dx * dx + dy * dy);
        accumulated += seg_len;

        if (accumulated >= lookahead) {
            lookahead_x = px1;
            lookahead_y = py1;
            found = true;
            break;
        }
    }

    if (!found) {
        // 路径剩余部分不足 lookahead，用终点
        lookahead_x = global_plan_.poses.back().pose.position.x;
        lookahead_y = global_plan_.poses.back().pose.position.y;
    }

    // ========== 步骤 3：把预瞄点转到机器人坐标系 ==========
    double dx = lookahead_x - robot_x;
    double dy = lookahead_y - robot_y;

    double cos_yaw = std::cos(-robot_yaw);
    double sin_yaw = std::sin(-robot_yaw);

    double local_x = dx * cos_yaw - dy * sin_yaw;
    double local_y = dx * sin_yaw + dy * cos_yaw;

    // 预瞄点在身后，原地转（选最近的方向）
    if (local_x < 0.0) {
        cmd_vel.twist.linear.x = 0.0;
        double omega = (local_y > 0.0 ? 1.0 : -1.0) * max_angular_speed_ * 0.5;
        cmd_vel.twist.angular.z = omega;
        return cmd_vel;
    }

    // ========== 步骤 4：Pure Pursuit 曲率计算 ==========
    double Ld_sq = local_x * local_x + local_y * local_y;
    double Ld = std::sqrt(Ld_sq);
    if (Ld < 1e-3) Ld = 1e-3;

    double kappa = 2.0 * local_y / Ld_sq;
    double omega = max_linear_speed_ * kappa;

    // 限制角速度
    omega = std::clamp(omega, -max_angular_speed_, max_angular_speed_);

    // ========== 步骤 5：接近目标时减速 ==========
    double goal_x = global_plan_.poses.back().pose.position.x;
    double goal_y = global_plan_.poses.back().pose.position.y;
    double dist_to_goal = std::sqrt(
        (goal_x - robot_x) * (goal_x - robot_x) +
        (goal_y - robot_y) * (goal_y - robot_y));

    double v = max_linear_speed_;
    if (dist_to_goal < 0.5) {
        v = std::max(min_linear_speed_,
                     max_linear_speed_ * dist_to_goal / 0.5);
    }

    // 角速度过大时降线速度
    if (std::abs(omega) > 0.5 * max_angular_speed_) {
        v *= 0.5;
    }

    cmd_vel.twist.linear.x = v;
    cmd_vel.twist.angular.z = omega;

    return cmd_vel;
}

}  // namespace pursuit_controller

PLUGINLIB_EXPORT_CLASS(pursuit_controller::PursuitController, nav2_core::Controller)
