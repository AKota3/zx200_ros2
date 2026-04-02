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

  node_->declare_parameter(name + ".goal_update_threshold", 0.01);
  node_->get_parameter(name + ".goal_update_threshold", goal_update_threshold_);

  global_frame_ = "map";

  has_last_goal_ = false;

  RCLCPP_INFO(logger_, "StraightLinePlanner configured");
}

nav_msgs::msg::Path StraightLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,  // ← 実際は使わない
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;

  geometry_msgs::msg::PoseStamped start_tf;
  geometry_msgs::msg::PoseStamped goal_tf;

  // ===== goalをTF変換 =====
  try {
    if (goal.header.frame_id != global_frame_) {
      tf_->transform(goal, goal_tf, global_frame_);
    } else {
      goal_tf = goal;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Goal TF failed: %s", ex.what());
    return path;
  }

  // ===== startの決定 =====
  if (has_last_goal_) {
    start_tf = last_goal_;   // ★前回ゴールを使用
  } else {
    // 初回のみ現在位置
    try {
      if (start.header.frame_id != global_frame_) {
        tf_->transform(start, start_tf, global_frame_);
      } else {
        start_tf = start;
      }
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "Start TF failed: %s", ex.what());
      return path;
    }
  }

  // ===== 直線生成 =====
  double dx = goal_tf.pose.position.x - start_tf.pose.position.x;
  double dy = goal_tf.pose.position.y - start_tf.pose.position.y;
  double distance = std::hypot(dx, dy);

  int steps = std::max(1, static_cast<int>(distance / interpolation_resolution_));

  double yaw = std::atan2(dy, dx);
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);

  for (int i = 0; i <= steps; ++i)
  {
    double t = static_cast<double>(i) / steps;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;

    pose.pose.position.x = start_tf.pose.position.x + t * dx;
    pose.pose.position.y = start_tf.pose.position.y + t * dy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf2::toMsg(q);

    path.poses.push_back(pose);
  }

  // ===== 安定化（重要）=====
  // 最初の点を現在位置にする
  if (!path.poses.empty()) {
    geometry_msgs::msg::PoseStamped current_tf;

    try {
      if (start.header.frame_id != global_frame_) {
        tf_->transform(start, current_tf, global_frame_);
      } else {
        current_tf = start;
      }
      path.poses.front() = current_tf;
    } catch (...) {}
  }

  // ===== goal更新判定 =====
  if (!has_last_goal_) {
    last_goal_ = goal_tf;
    has_last_goal_ = true;
  } else {
    double d = std::hypot(
      goal_tf.pose.position.x - last_goal_.pose.position.x,
      goal_tf.pose.position.y - last_goal_.pose.position.y
    );

    if (d > goal_update_threshold_) {
      last_goal_ = goal_tf;
    }
  }

  return path;
}

}  // namespace straight_line_planner

PLUGINLIB_EXPORT_CLASS(
  straight_line_planner::StraightLinePlanner,
  nav2_core::GlobalPlanner)