#include "straight_route.hpp"
#include "route_collision.hpp"
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
    auto node=parent.lock();
    const auto key=name+".prefer_straight_route";
    if(!node->has_parameter(key))node->declare_parameter<bool>(key,true);
    prefer_straight_=node->get_parameter(key).as_bool();
    costmap_ros_ = costmap_ros;
    nav2_smac_planner::SmacPlannerLattice::configure(parent, name, tf, costmap_ros);
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    auto * map = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
    const auto footprint = costmap_ros_->getRobotFootprint();
    auto free = [&](towergo_route::Pose p) {
      unsigned int mx,my;
      if (!map->worldToMap(p.x,p.y,mx,my) || map->getCost(mx,my)>=nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) return false;
      const double cost=checker.footprintCostAtPose(p.x,p.y,p.yaw,footprint);
      return cost>=0. && cost<nav2_costmap_2d::LETHAL_OBSTACLE && towergo_route::footprint_free(map,footprint,p);
    };
    const towergo_route::Pose from{start.pose.position.x,start.pose.position.y,tf2::getYaw(start.pose.orientation)};
    const towergo_route::Pose to{goal.pose.position.x,goal.pose.position.y,tf2::getYaw(goal.pose.orientation)};
    auto straight=prefer_straight_?towergo_route::straight(from,to,free):std::vector<towergo_route::Pose>{};
    if (!straight.empty()) {
      nav_msgs::msg::Path path;path.header=goal.header;
      for (auto p:straight) {
        auto pose=goal;pose.pose.position.x=p.x;pose.pose.position.y=p.y;
        tf2::Quaternion q;q.setRPY(0.,0.,p.yaw);pose.pose.orientation=tf2::toMsg(q);
        path.poses.push_back(pose);
      }
      return path;
    }
    // Obstacle detours still use the footprint-aware lattice, but the user's
    // terminal yaw is excluded from translation planning.
    auto arrival=goal;
    tf2::Quaternion arrival_q;
    arrival_q.setRPY(0.,0.,std::atan2(to.y-from.y,to.x-from.x));
    arrival.pose.orientation=tf2::toMsg(arrival_q);
    lock.unlock();
    auto plan = nav2_smac_planner::SmacPlannerLattice::createPlan(start, arrival);
    if (plan.poses.empty()) {return plan;}
    lock.lock();
    const auto & last = plan.poses.back().pose;
    const double dx = goal.pose.position.x - last.position.x;
    const double dy = goal.pose.position.y - last.position.y;
    const double yaw = tf2::getYaw(last.orientation);
    const double delta = tf2::getYaw(arrival.pose.orientation) - yaw;
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
    const towergo_route::Pose arrived{to.x,to.y,tf2::getYaw(arrival.pose.orientation)};
    if (!towergo_route::segment(arrived,to,free)) {
      throw nav2_core::PlannerException("Final in-place rotation intersects obstacle or unknown");
    }
    plan.poses.push_back(arrival);
    plan.poses.push_back(goal);
    // Recheck every interpolated pose after lattice/postprocessing and exact
    // endpoint stitching, including obstacles enclosed by the full rectangle.
    towergo_route::Pose previous=from;
    size_t sample_index=0;
    for(const auto & sample:plan.poses){
      towergo_route::Pose current{sample.pose.position.x,sample.pose.position.y,tf2::getYaw(sample.pose.orientation)};
      towergo_route::Pose rejected{};
      auto audited_free=[&](towergo_route::Pose p){if(!free(p)){rejected=p;return false;}return true;};
      if(!towergo_route::segment(previous,current,audited_free)) {
        unsigned int rx=0,ry=0;const bool in_map=map->worldToMap(rejected.x,rejected.y,rx,ry);
        RCLCPP_WARN(_node.lock()->get_logger(),"Rejected center cost=%u full_footprint_free=%d",in_map?map->getCost(rx,ry):255,towergo_route::footprint_free(map,footprint,rejected));
        RCLCPP_WARN(_node.lock()->get_logger(),"Clearance rejected sample %zu/%zu at (%.6f, %.6f, %.6f), from (%.6f, %.6f, %.6f)",sample_index,plan.poses.size(),rejected.x,rejected.y,rejected.yaw,from.x,from.y,from.yaw);
        throw nav2_core::PlannerException("Planned route violates full footprint clearance");
      }
      previous=current;++sample_index;
    }
    return plan;
  }

private:
  bool prefer_straight_=true;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};
}  // namespace astribot_nav_bringup

PLUGINLIB_EXPORT_CLASS(astribot_nav_bringup::ExactGoalLattice, nav2_core::GlobalPlanner)
