
#include "nav2_core/goal_checker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cmath>

namespace goal_checker
{

class YGoalChecker : public nav2_core::GoalChecker
{
public:
  YGoalChecker()
  : tolerance_(0.3),
    initialized_(false),
    start_x_(0.0),
    start_y_(0.0),
    goal_x_(0.0),
    goal_y_(0.0),
    dx_(0.0),
    dy_(0.0),
    length_(0.0)
  {}

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/) override
  {
    node_ = parent.lock();
    name_ = name;

    node_->declare_parameter(name_ + ".tolerance", 0.3);
    node_->get_parameter(name_ + ".tolerance", tolerance_);
  }

  void reset() override
  {
    initialized_ = false;

    RCLCPP_INFO(
      node_->get_logger(),
      "YGoalChecker reset");
  }

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist &) override
  {
    double current_x = query_pose.position.x;
    double current_y = query_pose.position.y;

    double new_goal_x = goal_pose.position.x;
    double new_goal_y = goal_pose.position.y;

    // ゴールが変更されたか確認
    bool goal_changed =
      initialized_ &&
      (std::hypot(
        new_goal_x - goal_x_,
        new_goal_y - goal_y_) > 0.01);

    // 初回またはゴール変更時に再初期化
    if (!initialized_ || goal_changed)
    {
      start_x_ = current_x;
      start_y_ = current_y;

      goal_x_ = new_goal_x;
      goal_y_ = new_goal_y;

      dx_ = goal_x_ - start_x_;
      dy_ = goal_y_ - start_y_;

      length_ = std::hypot(dx_, dy_);

      initialized_ = true;

      RCLCPP_INFO(
        node_->get_logger(),
        "YGoalChecker initialized: "
        "start=(%.3f, %.3f), goal=(%.3f, %.3f), "
        "distance=%.3f",
        start_x_, start_y_,
        goal_x_, goal_y_, length_);

      return false;
    }

    if (length_ < 1e-6)
    {
      return true;
    }

    // 現在位置の変位
    double current_dx = current_x - start_x_;
    double current_dy = current_y - start_y_;

    // ゴール方向への進行距離
    double progress =
      (current_dx * dx_ + current_dy * dy_) / length_;

    // 横方向のズレ
    double cross_track =
      (dx_ * current_dy - dy_ * current_dx) / length_;

    // ゴールまでの残り距離
    double remaining = length_ - progress;

    // ゴール手前 tolerance 以内、または通過後
    bool longitudinal_ok =
      remaining <= tolerance_;

    // 横方向のズレ
    bool lateral_ok =
      std::abs(cross_track) <= tolerance_;

    bool reached = longitudinal_ok && lateral_ok;

    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "GoalChecker: current=(%.3f, %.3f), "
      "progress=%.3f, remaining=%.3f, "
      "cross_track=%.3f, reached=%s",
      current_x, current_y,
      progress, remaining,
      cross_track,
      reached ? "true" : "false");

    return reached;
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tol,
    geometry_msgs::msg::Twist & vel_tol) override
  {
    pose_tol.position.x = tolerance_;
    pose_tol.position.y = tolerance_;
    pose_tol.position.z = 0.0;

    vel_tol.linear.x = 0.0;
    vel_tol.angular.z = 0.0;

    return true;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;

  double tolerance_;

  bool initialized_;

  double start_x_;
  double start_y_;

  double goal_x_;
  double goal_y_;

  double dx_;
  double dy_;

  double length_;
};

}  // namespace goal_checker

PLUGINLIB_EXPORT_CLASS(
  goal_checker::YGoalChecker,
  nav2_core::GoalChecker)
