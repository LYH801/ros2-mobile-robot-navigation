#ifndef PURSUIT_CONTROLLER_HPP
#define PURSUIT_CONTROLLER_HPP

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace pursuit_controller
{

class PursuitController : public nav2_core::Controller
{
public:
    PursuitController() = default;
    ~PursuitController() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    void setPlan(const nav_msgs::msg::Path& path) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::Twist& velocity,
        nav2_core::GoalChecker* goal_checker) override;

    void setSpeedLimit(
        const double& speed_limit,
        const bool& percentage) override;

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::string global_frame_;
    std::string name_;

    nav_msgs::msg::Path global_plan_;

    // 参数
    double lookahead_dist_;
    double min_lookahead_dist_;
    double max_lookahead_dist_;
    double lookahead_time_;
    double max_linear_speed_;
    double min_linear_speed_;
    double max_angular_speed_;
    double goal_tolerance_;
    bool use_velocity_scaled_lookahead_;
};

}  // namespace pursuit_controller

#endif
