#include "nav2_core/goal_checker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace goal_checker
{

class YGoalChecker : public nav2_core::GoalChecker
{
public:
  YGoalChecker() = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent.lock();
    name_ = name;

    node_->declare_parameter(name_ + ".y_tolerance", 0.0);
    node_->get_parameter(name_ + ".y_tolerance", y_tolerance_);
  }

  void reset() override {}

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist &) override
  {
    return query_pose.position.y >= goal_pose.position.y - y_tolerance_;
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tol,
    geometry_msgs::msg::Twist & vel_tol) override
  {
    pose_tol.position.x = y_tolerance_;
    pose_tol.position.y = y_tolerance_;
    pose_tol.position.z = 0.0;

    vel_tol.linear.x = 0.0;
    vel_tol.angular.z = 0.0;

    return true;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  double y_tolerance_;
};

}  // namespace goal_checker

PLUGINLIB_EXPORT_CLASS(goal_checker::YGoalChecker, nav2_core::GoalChecker)