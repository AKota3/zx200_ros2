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

    if (global_plan_.poses.empty()) {
      return cmd_vel;
    }

    std::string target_frame = costmap_ros_->getBaseFrameID();

    // ===== goal transform =====
    geometry_msgs::msg::PoseStamped goal_tf;

    try {
      auto goal_pose = global_plan_.poses.back();

      if (goal_pose.header.frame_id.empty()) {
        RCLCPP_ERROR(logger_, "Goal frame_id is EMPTY!");
        return cmd_vel;
      }

      goal_tf = tf_->transform(goal_pose, target_frame);

    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(logger_, "Goal TF failed: %s", ex.what());
      return cmd_vel;
    }

    // ===== 自分のpose transform =====
    geometry_msgs::msg::PoseStamped pose_tf;

    try {
      if (pose.header.frame_id.empty()) {
        RCLCPP_ERROR(logger_, "Pose frame_id is EMPTY!");
        return cmd_vel;
      }

      pose_tf = tf_->transform(pose, target_frame);

    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(logger_, "Pose TF failed: %s", ex.what());
      return cmd_vel;
    }

    // ===== 状態取得 =====
    double x = goal_tf.pose.position.x;
    double y = goal_tf.pose.position.y;

    double yaw = tf2::getYaw(pose_tf.pose.orientation);

    double goal_dist = hypot(x, y);

    // ===== ゴール判定 =====
    if (goal_dist < goal_tolerance_) {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      return cmd_vel;
    }

    // ===== 経路方向 =====
    double path_theta = atan2(y, x);

    // ===== 進行方向判定 =====
    double heading_error = yaw - path_theta;
    heading_error = atan2(sin(heading_error), cos(heading_error));

    double c = cos(heading_error);
    double threshold = 0.2;

    if (c > threshold) direction_ = 1.0;
    else if (c < -threshold) direction_ = -1.0;

    // ===== 有効yaw =====
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
    double angular_vel = direction_ * (k1_ * theta + k2_ * y);

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