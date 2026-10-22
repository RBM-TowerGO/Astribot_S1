#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace astribot_nav_bringup
{
// Humble lattice endpoints use discrete heading bins. Preserve the user's
// precise final pose after checking the short connection to that endpoint.
class ExactGoalLattice : public nav2_smac_planner::SmacPlannerLattice
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    costmap_ros_ = costmap_ros;
    nav2_smac_planner::SmacPlannerLattice::configure(parent, name, tf, costmap_ros);
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    auto plan = nav2_smac_planner::SmacPlannerLattice::createPlan(start, goal);
    if (plan.poses.empty()) {return plan;}
    auto * map = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
    const auto footprint = costmap_ros_->getRobotFootprint();
    const auto & last = plan.poses.back().pose;
    const double dx = goal.pose.position.x - last.position.x;
    const double dy = goal.pose.position.y - last.position.y;
    const double yaw = tf2::getYaw(last.orientation);
    const double delta = tf2::getYaw(goal.pose.orientation) - yaw;
    const double dyaw = std::atan2(std::sin(delta), std::cos(delta));
    // The lattice tolerance is a local endpoint connection, never a shortcut
    // across an unfinished route. Reject unexpectedly distant endpoints.
    if (std::hypot(dx, dy) > .3) {
      throw nav2_core::PlannerException("Lattice endpoint is too far from exact goal");
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(std::max(
      std::hypot(dx, dy) / .01, std::abs(dyaw) / .01))));
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      const double x = last.position.x + t * dx, y = last.position.y + t * dy;
      unsigned int mx, my;
      if (!map->worldToMap(x, y, mx, my) ||
        map->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        throw nav2_core::PlannerException("Exact goal connection enters obstacle or unknown");
      }
      const double cost = checker.footprintCostAtPose(x, y, yaw + t * dyaw, footprint);
      if (cost < 0. || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        throw nav2_core::PlannerException("Exact goal rotation intersects obstacle or unknown");
      }
    }
    plan.poses.push_back(goal);
    return plan;
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};
}  // namespace astribot_nav_bringup

PLUGINLIB_EXPORT_CLASS(astribot_nav_bringup::ExactGoalLattice, nav2_core::GlobalPlanner)
