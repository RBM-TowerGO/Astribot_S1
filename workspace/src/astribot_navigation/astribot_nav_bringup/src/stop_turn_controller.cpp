#include "stop_turn.hpp"
#include "route_collision.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/string.hpp"
#include <nlohmann/json.hpp>
#include <mutex>
#include <atomic>
#include <ctime>
namespace towergo {
inline double monotonic(){timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}
class Feedback {
 std::mutex mutex;double sent=-1e20,still_since=-1;bool stationary=false,valid=false;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
 public:
 void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node){
  sub=node->create_subscription<std_msgs::msg::String>("executor_status",10,[this](std_msgs::msg::String::ConstSharedPtr msg){
   std::lock_guard<std::mutex> guard(mutex);double now=monotonic();
   try{auto d=nlohmann::json::parse(msg->data);double stamp=d.at("sent").get<double>();
    if(stamp<=sent||now-stamp<0||now-stamp>=.3)return;
    bool gap=stamp-sent>=.3;sent=stamp;
    valid=d.value("rights",false)&&d.at("fault").is_null()&&!d.value("holding",false)&&!d.at("task").is_null();
    auto command=d.at("command");stationary=valid&&d.value("stationary",false)&&std::abs(command[0].get<double>())<1e-6&&std::abs(command[1].get<double>())<1e-6;
    if(!stationary)still_since=-1;else if(gap||still_since<0)still_since=now;
   }catch(...){valid=false;stationary=false;still_since=-1;}
  });
 }
 std::pair<bool,bool> state(){std::lock_guard<std::mutex> guard(mutex);double now=monotonic();
  return {valid&&now-sent<.3,stationary&&now-sent<.3&&still_since>=0&&now-still_since>=.2};}
};
class StopTurnController:public nav2_core::Controller {
 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> cm;std::shared_ptr<tf2_ros::Buffer> tf;
 rclcpp_lifecycle::LifecycleNode::SharedPtr node;std::string name,frame;Feedback feedback;
 towergo_route::StopTurn follower;std::mutex mutex;std::atomic<double> speed{.05},limit{.3};double last=0;
 rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status;
 rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters;
 public:
 void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,std::string id,std::shared_ptr<tf2_ros::Buffer> buffer,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)override{
  node=parent.lock();name=id;tf=buffer;cm=costmap;feedback.initialize(node);
  for(auto kv:std::vector<std::pair<std::string,double>>{{"desired_linear_vel",.05},{"regulated_linear_scaling_min_speed",.02},{"min_approach_linear_velocity",.015}})
    if(!node->has_parameter(name+"."+kv.first))node->declare_parameter(name+"."+kv.first,kv.second);
  speed=node->get_parameter(name+".desired_linear_vel").as_double();
  parameters=node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& ps){
    rcl_interfaces::msg::SetParametersResult result;result.successful=true;
    for(auto p:ps)if(p.get_name()==name+".desired_linear_vel"){
      double v=p.as_double();if(!std::isfinite(v)||v<=0||v>.3){result.successful=false;result.reason="Speed must be in (0, 0.30] m/s";return result;}}
    for(auto p:ps)if(p.get_name()==name+".desired_linear_vel")speed=p.as_double();return result;});
  status=node->create_publisher<std_msgs::msg::String>("route_execution",10);
 }
 void cleanup()override{parameters.reset();status.reset();cm.reset();tf.reset();node.reset();}
 void activate()override{status->on_activate();}void deactivate()override{status->on_deactivate();}
 void setPlan(const nav_msgs::msg::Path& path)override{
  std::lock_guard<std::mutex> guard(mutex);std::vector<towergo_route::Pose> points;const bool same_frame=frame==path.header.frame_id;frame=path.header.frame_id;
  for(auto p:path.poses){if(p.header.frame_id!=frame)throw nav2_core::PlannerException("Mixed path frames");
    towergo_route::Pose q{p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)};
    if(!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.yaw))throw nav2_core::PlannerException("Invalid path pose");
    if(!points.empty()&&std::hypot(q.x-points.back().x,q.y-points.back().y)<1e-6)points.back()=q;else points.push_back(q);}
  const double now=monotonic();
  if(same_frame){if(!follower.replan(points,now))last=now;}
  else {follower.plan(points,now);last=now;}
 }
 void setSpeedLimit(const double& value,const bool& percent)override{limit=value==0?.3:std::clamp(percent?speed.load()*value/100.:value,0.,.3);}
 geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,const geometry_msgs::msg::Twist&,nav2_core::GoalChecker*)override{
  std::lock_guard<std::mutex> guard(mutex);geometry_msgs::msg::TwistStamped out;out.header.stamp=node->now();out.header.frame_id=cm->getBaseFrameID();
  auto ready=feedback.state();if(!ready.first)throw nav2_core::PlannerException("Missing fresh executor feedback/control permission");
  geometry_msgs::msg::PoseStamped world,local;
  try{tf->transform(pose,world,frame,tf2::durationFromSec(.1));tf->transform(pose,local,cm->getGlobalFrameID(),tf2::durationFromSec(.1));}
  catch(const tf2::TransformException&e){throw nav2_core::PlannerException(e.what());}
  towergo_route::Pose p{world.pose.position.x,world.pose.position.y,tf2::getYaw(world.pose.orientation)};
  double now=monotonic(),dt=now-last;last=now;auto command=follower.tick(p,ready.second,now,dt,std::min(speed.load(),limit.load()));
  auto *map=cm->getCostmap();std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> maplock(*map->getMutex());auto footprint=cm->getRobotFootprint();
  towergo_route::Pose a{local.pose.position.x,local.pose.position.y,tf2::getYaw(local.pose.orientation)},b=a;
  if(command.w!=0){double target=follower.stage==towergo_route::StopTurn::FINAL_TURN?follower.points.back().yaw:follower.heading;
    b.yaw+=towergo_route::angle(target-p.yaw);}
  if(command.v!=0){double distance=.1+command.v*1.5+command.v*command.v/.2;b.x+=distance*cos(a.yaw);b.y+=distance*sin(a.yaw);}
  if(!towergo_route::segment(a,b,[&](auto q){return towergo_route::footprint_free(map,footprint,q);}))throw nav2_core::PlannerException("Straight motion or in-place rotation intersects occupied/unknown map");
  out.twist.linear.x=command.v;out.twist.angular.z=command.w;
  std_msgs::msg::String state;state.data=nlohmann::json{{"stage",follower.label()},{"segment",follower.next},{"corners",follower.points.size()},{"stationary_confirmed",ready.second},{"command",{command.v,command.w}}}.dump();status->publish(state);
  return out;
 }
};
class StopTurnGoalChecker:public nav2_core::GoalChecker {
 Feedback feedback;
 public:
 void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,const std::string&,const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)override{feedback.initialize(parent.lock());}
 void reset()override{}
 bool isGoalReached(const geometry_msgs::msg::Pose& p,const geometry_msgs::msg::Pose& goal,const geometry_msgs::msg::Twist&)override{
  auto state=feedback.state();return state.first&&state.second&&std::hypot(p.position.x-goal.position.x,p.position.y-goal.position.y)<=.15&&std::abs(towergo_route::angle(tf2::getYaw(p.orientation)-tf2::getYaw(goal.orientation)))<=.08;
 }
 bool getTolerances(geometry_msgs::msg::Pose& p,geometry_msgs::msg::Twist& v)override{
  p.position.x=p.position.y=.15;tf2::Quaternion q;q.setRPY(0,0,.08);p.orientation=tf2::toMsg(q);v.linear.x=v.linear.y=.005;v.angular.z=.01;return true;
 }
};
}
PLUGINLIB_EXPORT_CLASS(towergo::StopTurnController,nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(towergo::StopTurnGoalChecker,nav2_core::GoalChecker)
