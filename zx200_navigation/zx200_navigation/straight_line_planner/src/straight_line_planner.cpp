#include "straight_line_planner/straight_line_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace straight_line_planner
{

void StraightLinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
  auto node = parent.lock();
  logger_ = node->get_logger();

  node->declare_parameter(name + ".interpolation_resolution", 0.05);
  node->get_parameter(name + ".interpolation_resolution", interpolation_resolution_);

  global_frame_ = "map";

  RCLCPP_INFO(logger_, "StraightLinePlanner configured");
}

nav_msgs::msg::Path StraightLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = global_frame_;

  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;
  double distance = std::hypot(dx, dy);

  int steps = std::max(1, static_cast<int>(distance / interpolation_resolution_));

  for (int i = 0; i <= steps; ++i)
  {
    double t = static_cast<double>(i) / steps;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;

    pose.pose.position.x = start.pose.position.x + t * dx;
    pose.pose.position.y = start.pose.position.y + t * dy;
    pose.pose.position.z = 0.0;

    pose.pose.orientation = start.pose.orientation;

    path.poses.push_back(pose);
  }

  return path;
}

}  // namespace straight_line_planner

PLUGINLIB_EXPORT_CLASS(
  straight_line_planner::StraightLinePlanner,
  nav2_core::GlobalPlanner)