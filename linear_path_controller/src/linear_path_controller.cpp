#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"

#include "tf2/utils.h"
#include "tf2_ros/buffer.h"

#include "pluginlib/class_list_macros.hpp"

namespace linear_path_controller
{

class LinearPathController : public nav2_core::Controller
{
public:
  LinearPathController() = default;
  ~LinearPathController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("Failed to lock node");
    }

    tf_ = tf;
    costmap_ros_ = costmap_ros;
    clock_ = node->get_clock();

    nav2_util::declare_parameter_if_not_declared(
      node, name + ".desired_linear_vel", rclcpp::ParameterValue(0.5));

    nav2_util::declare_parameter_if_not_declared(
      node, name + ".max_angular_vel", rclcpp::ParameterValue(1.0));

    nav2_util::declare_parameter_if_not_declared(
      node, name + ".k1", rclcpp::ParameterValue(1.0));

    nav2_util::declare_parameter_if_not_declared(
      node, name + ".k2", rclcpp::ParameterValue(1.0));

    desired_linear_vel_ = node->get_parameter(name + ".desired_linear_vel").as_double();
    node->get_parameter(name + ".max_angular_vel", max_angular_vel_);
    node->get_parameter(name + ".k1", k1_);
    node->get_parameter(name + ".k2", k2_);
  }

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    global_plan_ = path;
  }

  void setSpeedLimit(const double &, const bool &) override {}

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist&,
    nav2_core::GoalChecker*) override
  {
    geometry_msgs::msg::TwistStamped cmd_vel;

    if (global_plan_.poses.size() < 2) {
      return cmd_vel;
    }

    // ===== 経路（直線） =====
    auto start = global_plan_.poses.front().pose.position;
    auto goal  = global_plan_.poses.back().pose.position;

    double dx = goal.x - start.x;
    double dy = goal.y - start.y;

    double path_theta = atan2(dy, dx);

    // ===== 現在状態 =====
    double x = pose.pose.position.x;
    double y_pos = pose.pose.position.y;
    double yaw = tf2::getYaw(pose.pose.orientation);

    // ===== 姿勢誤差 =====
    double theta = yaw - path_theta;
    theta = atan2(sin(theta), cos(theta));

    // ===== 横ずれ誤差（直線距離） =====
    double A = dy;
    double B = -dx;
    double C = dx * start.y - dy * start.x;

    double y = (A * x + B * y_pos + C) / sqrt(A*A + B*B);

    // ===== 前後判定（ここが重要） =====
    double direction = (cos(theta) >= 0.0) ? 1.0 : -1.0;

    // 後ろ向きなら角度補正（安定化のため）
    if (direction < 0.0) {
      theta = atan2(sin(theta + M_PI), cos(theta + M_PI));
    }

    // ===== 制御則 =====
    double linear_vel = direction * desired_linear_vel_;
    double angular_vel = -k1_ * theta - k2_ * y;

    // ===== 角速度制限 =====
    angular_vel = std::max(
      -fabs(max_angular_vel_),
      std::min(angular_vel, fabs(max_angular_vel_)));

    // ===== 出力 =====
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();

    cmd_vel.twist.linear.x = linear_vel;
    cmd_vel.twist.angular.z = angular_vel;

    return cmd_vel;
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  nav_msgs::msg::Path global_plan_;

  double desired_linear_vel_;
  double max_angular_vel_;

  double k1_;
  double k2_;

  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace linear_path_controller


PLUGINLIB_EXPORT_CLASS(
  linear_path_controller::LinearPathController,
  nav2_core::Controller)