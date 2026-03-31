#include "straight_line_planner/straight_line_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace straight_line_planner
{

void StraightLinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
  node_ = parent.lock();
  logger_ = node_->get_logger();

  tf_ = tf;

  node_->declare_parameter(name + ".interpolation_resolution", 0.05);
  node_->get_parameter(name + ".interpolation_resolution", interpolation_resolution_);

  global_frame_ = "map";

  RCLCPP_INFO(logger_, "StraightLinePlanner configured");
}

nav_msgs::msg::Path StraightLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;

  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;

  geometry_msgs::msg::PoseStamped start_tf = start;
  geometry_msgs::msg::PoseStamped goal_tf = goal;

  // ===== TFでmapに変換 =====
  try {
    if (start.header.frame_id != global_frame_) {
      tf_->transform(start, start_tf, global_frame_);
    }
    if (goal.header.frame_id != global_frame_) {
      tf_->transform(goal, goal_tf, global_frame_);
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "TF transform failed: %s", ex.what());
    return path;
  }

  double dx = goal_tf.pose.position.x - start_tf.pose.position.x;
  double dy = goal_tf.pose.position.y - start_tf.pose.position.y;
  double distance = std::hypot(dx, dy);

  int steps = std::max(1, static_cast<int>(distance / interpolation_resolution_));

  // ===== 進行方向の向き（重要）=====
  double yaw = std::atan2(dy, dx);
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);

  for (int i = 0; i <= steps; ++i)
  {
    double t = static_cast<double>(i) / steps;

    geometry_msgs::msg::PoseStamped pose;

    //pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;

    pose.pose.position.x = start_tf.pose.position.x + t * dx;
    pose.pose.position.y = start_tf.pose.position.y + t * dy;
    pose.pose.position.z = 0.0;

    // ★ここが超重要（振動防止）
    pose.pose.orientation = tf2::toMsg(q);

    path.poses.push_back(pose);
  }

  return path;
}

}  // namespace straight_line_planner

PLUGINLIB_EXPORT_CLASS(
  straight_line_planner::StraightLinePlanner,
  nav2_core::GlobalPlanner)