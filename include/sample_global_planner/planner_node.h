#ifndef SAMPLE_GLOBAL_PLANNER_H_
#define SAMPLE_GLOBAL_PLANNER_H_

#include <costmap_2d/costmap_2d.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <vector>

namespace sample_global_planner {
class GlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  GlobalPlanner();

  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

 private:
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

 private:
  ros::Publisher plan_pub_;
  std::string frame_id_;
};
}  // namespace sample_global_planner

#endif