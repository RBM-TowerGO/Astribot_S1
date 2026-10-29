#include "stop_turn.hpp"
#include "route_collision.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <queue>
#include <chrono>
namespace towergo {
class StopTurnPlanner:public nav2_core::GlobalPlanner {
 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> cm;
 public:
 void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,std::string,std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> c)override{cm=c;}
 void cleanup()override{cm.reset();}void activate()override{}void deactivate()override{}
 nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,const geometry_msgs::msg::PoseStamped& goal)override{
  using namespace towergo_route;auto *map=cm->getCostmap();std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
  if(start.header.frame_id!=cm->getGlobalFrameID()||goal.header.frame_id!=cm->getGlobalFrameID())throw nav2_core::PlannerException("Unexpected planning frame");
  Pose from{start.pose.position.x,start.pose.position.y,tf2::getYaw(start.pose.orientation)},to{goal.pose.position.x,goal.pose.position.y,tf2::getYaw(goal.pose.orientation)};
  auto footprint=cm->getRobotFootprint();nav2_costmap_2d::padFootprint(footprint,.05);auto free=[&](Pose p){return footprint_free(map,footprint,p);};
  auto edge=[&](Pose a,Pose b){double h=std::hypot(b.x-a.x,b.y-a.y)<1e-8?a.yaw:atan2(b.y-a.y,b.x-a.x);
    Pose aa{a.x,a.y,h},bb{b.x,b.y,h};return segment(a,aa,free)&&segment(aa,bb,free)&&segment(bb,b,free);};
  std::vector<Pose> corners;
  if(edge(from,to))corners={from,to};
  else {
    unsigned int sx,sy,gx,gy;
    if(!map->worldToMap(from.x,from.y,sx,sy)||!map->worldToMap(to.x,to.y,gx,gy))throw nav2_core::PlannerException("Route endpoints outside map");
    int w=map->getSizeInCellsX(),h=map->getSizeInCellsY(),n=w*h,source=sy*w+sx,target=gy*w+gx;
    if(source==target)throw nav2_core::PlannerException("No safe in-place turn at endpoint");
    std::vector<double> distance(n,1e30);std::vector<int> previous(n,-1);std::vector<int8_t> cache(n,-1);
    double radius=0,outer=0;nav2_costmap_2d::calculateMinAndMaxDistances(footprint,radius,outer);
    // The 2-D guide uses only the inscribed radius. It is not executable yet:
    // simplify() checks the full rectangle on every straight edge and actual turn.
    // Requiring the circumscribed disk at every cell incorrectly blocks aisles.
    int cells=std::ceil(radius/map->getResolution()+.7072);std::vector<std::pair<int,int>> disk;
    for(int y=-cells;y<=cells;++y)for(int x=-cells;x<=cells;++x)if(std::hypot(x,y)*map->getResolution()<=radius+.7072*map->getResolution())disk.emplace_back(x,y);
    auto clear=[&](int id){if(id==source||id==target)return true;if(cache[id]>=0)return cache[id]!=0;
      int x=id%w,y=id/w;bool ok=true;for(auto d:disk){int xx=x+d.first,yy=y+d.second;
        if(xx<0||yy<0||xx>=w||yy>=h||map->getCost(xx,yy)>=nav2_costmap_2d::LETHAL_OBSTACLE){ok=false;break;}}
      cache[id]=ok;return ok;};
    using Item=std::pair<double,int>;std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
    auto heuristic=[&](int id){return std::hypot(id%w-int(gx),id/w-int(gy));};
    distance[source]=0;queue.emplace(heuristic(source),source);bool found=false;
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);int iterations=0;
    while(!queue.empty()){
      auto [score,id]=queue.top();queue.pop();if(score>distance[id]+heuristic(id)+1e-8)continue;
      if(id==target){found=true;break;}
      if((++iterations%1024)==0&&std::chrono::steady_clock::now()>deadline)throw nav2_core::PlannerException("Stop-turn route search timed out");
      int x=id%w,y=id/w;
      for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){if(!dx&&!dy)continue;
        int xx=x+dx,yy=y+dy;if(xx<0||yy<0||xx>=w||yy>=h)continue;int next=yy*w+xx;
        if(!clear(next)||(dx&&dy&&(!clear(y*w+xx)||!clear(yy*w+x))))continue;
        double cost=distance[id]+std::hypot(dx,dy)*(1.+map->getCost(xx,yy)/252.);
        if(cost<distance[next]){distance[next]=cost;previous[next]=id;queue.emplace(cost+heuristic(next),next);}
      }
    }
    if(!found)throw nav2_core::PlannerException("No route with safe in-place turning space");
    std::vector<Pose> guide;for(int at=target;at!=source;at=previous[at]){double x,y;map->mapToWorld(at%w,at/w,x,y);guide.push_back({x,y,0});}
    std::reverse(guide.begin(),guide.end());guide.insert(guide.begin(),from);guide.back()=to;
    try{corners=simplify(guide,edge);}catch(const std::exception&e){throw nav2_core::PlannerException(e.what());}
  }
  // Sparse vertices are intentional: each one is a mandatory stopped corner.
  nav_msgs::msg::Path result;result.header=goal.header;
  for(auto p:corners){auto pose=goal;pose.pose.position.x=p.x;pose.pose.position.y=p.y;
    tf2::Quaternion q;q.setRPY(0,0,p.yaw);pose.pose.orientation=tf2::toMsg(q);result.poses.push_back(pose);}
  return result;
 }
};
}
PLUGINLIB_EXPORT_CLASS(towergo::StopTurnPlanner,nav2_core::GlobalPlanner)
