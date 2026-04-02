#ifndef STRAIGHT_LINE_PLANNER_HPP_
#define STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

namespace straight_line_planner
{

class StraightLinePlanner : public nav2_core::GlobalPlanner
{
public:
  StraightLinePlanner() = default;
  ~StraightLinePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // ===== 基本 =====
  rclcpp::Logger logger_{rclcpp::get_logger("StraightLinePlanner")};
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::string global_frame_;
  double interpolation_resolution_{0.05};

  // ===== ★追加（今回のキモ）=====
  geometry_msgs::msg::PoseStamped last_goal_;  // 前回ゴール
  bool has_last_goal_{false};                  // 初回判定
  double goal_update_threshold_{0.01};         // ゴール更新閾値
};

}  // namespace straight_line_planner

#endif