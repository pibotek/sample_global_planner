#include "sample_global_planner/planner_node.h"
#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(sample_global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace sample_global_planner {
GlobalPlanner::GlobalPlanner() {
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan) {
  ROS_INFO("make plan start:[%f %f], goal:[%f %f]", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

  plan.clear();

  float yaw = atan2(goal.pose.position.y - start.pose.position.y, goal.pose.position.x - start.pose.position.x);

  int id = 0;
  int n = 0;
  float goal_distance = sqrt(pow((start.pose.position.x - goal.pose.position.x), 2) + pow((start.pose.position.y - goal.pose.position.y), 2));

  float delta = 0.1;
  while (n * delta < goal_distance) {
    geometry_msgs::PoseStamped pose = goal;

    pose.pose.position.x = (n * delta) * cos(yaw) + start.pose.position.x;
    pose.pose.position.y = (n * delta) * sin(yaw) + start.pose.position.y;
    ++n;
    plan.push_back(pose);
  }

  plan.push_back(goal);

  publishPlan(plan);
  return !plan.empty();
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  ros::NodeHandle private_nh("~/" + name);
  plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
  frame_id_ = costmap_ros->getGlobalFrameID();
}

void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());

  gui_path.header.frame_id = frame_id_;
  gui_path.header.stamp = ros::Time::now();

  for (unsigned int i = 0; i < path.size(); i++) {
    gui_path.poses[i] = path[i];
  }

  plan_pub_.publish(gui_path);
}
}  // namespace sample_global_planner
