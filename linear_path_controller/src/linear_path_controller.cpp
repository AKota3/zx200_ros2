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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace linear_path_controller
{

class LinearPathController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();

    tf_ = tf;
    costmap_ros_ = costmap_ros;
    clock_ = node->get_clock();
    logger_ = node->get_logger();

    nav2_util::declare_parameter_if_not_declared(
      node, name + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".max_angular_vel", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".k1", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".k2", rclcpp::ParameterValue(2.0));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".goal_tolerance", rclcpp::ParameterValue(0.2));

    node->get_parameter(name + ".desired_linear_vel", desired_linear_vel_);
    node->get_parameter(name + ".max_angular_vel", max_angular_vel_);
    node->get_parameter(name + ".k1", k1_);
    node->get_parameter(name + ".k2", k2_);
    node->get_parameter(name + ".goal_tolerance", goal_tolerance_);
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
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = pose.header.frame_id;

    if (global_plan_.poses.size() < 2) {
      return cmd_vel;
    }

    // ===== TF変換 =====
    nav_msgs::msg::Path transformed_plan;
    std::string target_frame = costmap_ros_->getBaseFrameID();

    for (auto & p : global_plan_.poses) {
      geometry_msgs::msg::PoseStamped tf_pose;

      try {
        tf_pose = tf_->transform(
          p,
          target_frame,
          tf2::durationFromSec(0.2)
        );
        transformed_plan.poses.push_back(tf_pose);

      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(logger_,
          "TF failed: %s (from %s to %s)",
          ex.what(),
          p.header.frame_id.c_str(),
          target_frame.c_str());
        return cmd_vel;
      }
    }

    if (transformed_plan.poses.empty()) {
      return cmd_vel;
    }

    // ===== 経路情報 =====
    auto start = transformed_plan.poses.front().pose.position;
    auto goal  = transformed_plan.poses.back().pose.position;

    double dx = goal.x - start.x;
    double dy = goal.y - start.y;
    double path_theta = atan2(dy, dx);

    double yaw = tf2::getYaw(pose.pose.orientation);
    double goal_dist = hypot(goal.x, goal.y);

    // ===== ゴール停止 =====
    if (goal_dist < goal_tolerance_) {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      return cmd_vel;
    }

    // ===== 横ずれ（重要修正）=====
    double y = goal.y;

    // ===== 進行方向判定 =====
    double goal_theta = atan2(goal.y, goal.x);
    double heading_error = yaw - goal_theta;
    heading_error = atan2(sin(heading_error), cos(heading_error));

    double c = cos(heading_error);
    double threshold = 0.2;

    if (c > threshold) direction_ = 1.0;
    else if (c < -threshold) direction_ = -1.0;

    // ===== 有効yaw（後退対応）=====
    double effective_yaw = yaw;

    if (direction_ < 0.0) {
      effective_yaw = atan2(sin(yaw + M_PI), cos(yaw + M_PI));
    }

    // ===== 角度誤差 =====
    double theta = effective_yaw - path_theta;
    theta = atan2(sin(theta), cos(theta));

    // ===== 速度生成 =====
    double speed_scale = std::min(1.0, goal_dist);
    double linear_vel = direction_ * desired_linear_vel_ * speed_scale;

    double angular_vel = direction_*(k1_ * theta + k2_ * y);

    angular_vel = std::max(
      -fabs(max_angular_vel_),
      std::min(angular_vel, fabs(max_angular_vel_)));

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
  double goal_tolerance_;

  double direction_ = 1.0;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("LinearPathController")};
};

}  // namespace linear_path_controller

PLUGINLIB_EXPORT_CLASS(
  linear_path_controller::LinearPathController,
  nav2_core::Controller)