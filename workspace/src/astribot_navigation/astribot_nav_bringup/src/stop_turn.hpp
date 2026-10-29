#pragma once
#include "straight_route.hpp"
#include <stdexcept>
#include <string>
namespace towergo_route {
struct Command {double v=0,w=0;};
// Waypoints are explicit corners, never a smoothed curve. All transitions
// between translation and rotation pass through zero + SDK stopped evidence.
class StopTurn {
 public:
  std::vector<Pose> points;Pose anchor{0,0,0};size_t next=1;
  enum Stage {HOLD_TURN,TURN,HOLD_DRIVE,DRIVE,HOLD_CORNER,FINAL_TURN,HOLD_DONE,DONE};
  Stage stage=HOLD_TURN;double since=0,heading=0,last_v=0,last_w=0,deviation_since=-1;
  const double xy=.05,yaw=.01,final_xy=.15,final_yaw=.08;
  void plan(std::vector<Pose> p,double now) {
    if(p.empty())throw std::runtime_error("Empty stop-turn path");
    points=std::move(p);next=points.size()>1?1:0;set(HOLD_TURN,now);
  }
  // Replanning the same next edge must not restart its stop/turn/acceleration
  // stages. A new edge or destination is a real route change and starts safely.
  bool replan(std::vector<Pose> p,double now) {
    if(p.empty())throw std::runtime_error("Empty stop-turn path");
    const bool same_goal=!points.empty() &&
      std::hypot(p.back().x-points.back().x,p.back().y-points.back().y)<1e-6 &&
      std::abs(angle(p.back().yaw-points.back().yaw))<1e-6;
    const bool same_edge=p.size()>=2 && next<points.size() &&
      std::hypot(p[1].x-points[next].x,p[1].y-points[next].y)<1e-6;
    if(same_goal && same_edge && stage!=DONE) {
      points=std::move(p);next=1;return true;
    }
    plan(std::move(p),now);return false;
  }
  void set(Stage s,double now){stage=s;since=now;last_v=last_w=0;deviation_since=-1;}
  const char *label() const {
    switch(stage){case HOLD_TURN:return "stopping_before_turn";case TURN:return "turning";
      case HOLD_DRIVE:return "stopping_before_straight";case DRIVE:return "straight";
      case HOLD_CORNER:return "stopping_at_corner";case FINAL_TURN:return "final_turn";
      case HOLD_DONE:return "stopping_at_goal";default:return "done";}
  }
  double turn(double error,double dt) {
    double desired=std::clamp(error*.7,-.1,.1),step=.1*std::clamp(dt,0.,.05);
    last_w=std::clamp(desired,last_w-step,last_w+step);return last_w;
  }
  Command tick(Pose p,bool stopped,double now,double dt,double maximum) {
    if(points.empty())throw std::runtime_error("No stop-turn path");
    auto target=points[next];double distance=std::hypot(target.x-p.x,target.y-p.y);
    const bool settled=stopped&&now-since>=.25;
    const double arrival=next+1==points.size()?final_xy:xy;
    switch(stage){
      case HOLD_TURN:
        if(settled){
          if(distance<=arrival){set(next+1==points.size()?FINAL_TURN:HOLD_CORNER,now);}
          else {anchor=p;heading=std::atan2(target.y-p.y,target.x-p.x);set(TURN,now);}
        }return {};
      case TURN: {
        double e=angle(heading-p.yaw);
        if(std::abs(e)<=yaw){set(HOLD_DRIVE,now);return {};}
        return {0,turn(e,dt)};}
      case HOLD_DRIVE:
        if(settled)set(DRIVE,now);return {};
      case DRIVE: {
        if(distance<=arrival){set(HOLD_CORNER,now);return {};}
        const double aim=std::atan2(target.y-p.y,target.x-p.x);
        // Correct drift by stopping and realigning; never bend a moving segment.
        const double cross=std::abs((p.x-anchor.x)*std::sin(heading)-(p.y-anchor.y)*std::cos(heading));
        const double heading_error=std::abs(angle(heading-p.yaw));
        const bool deviated=cross>.03 || heading_error>.08 ||
          (std::abs(angle(aim-p.yaw))>.08 && std::abs(distance*std::sin(aim-p.yaw))>.03);
        // Reject large deviations immediately. Small measurement excursions
        // must persist for 0.3 s before stopping to realign. Every command still
        // passes the current footprint and swept collision check in the plugin.
        const bool severe=cross>.10 || heading_error>.25 || std::cos(aim-p.yaw)<=0;
        if(deviated){if(deviation_since<0)deviation_since=now;}
        else deviation_since=-1;
        if(severe || (deviation_since>=0 && now-deviation_since>=.3)){set(HOLD_TURN,now);return {};}
        double v=std::min({maximum,.5*std::max(0.,distance-arrival*.5),last_v+.1*std::clamp(dt,0.,.05)});
        last_v=v;return {v,0};}
      case HOLD_CORNER:
        if(settled){if(distance>arrival){set(HOLD_TURN,now);}
          else if(next+1<points.size()){++next;set(HOLD_TURN,now);}
          else set(FINAL_TURN,now);}
        return {};
      case FINAL_TURN: {
        if(distance>arrival){set(HOLD_TURN,now);return {};}
        double e=angle(points.back().yaw-p.yaw);
        if(std::abs(e)<=final_yaw){set(HOLD_DONE,now);return {};}
        return {0,turn(e,dt)};}
      case HOLD_DONE:
        if(settled){if(distance>arrival)set(HOLD_TURN,now);
          else if(std::abs(angle(points.back().yaw-p.yaw))>final_yaw)set(FINAL_TURN,now);
          else set(DONE,now);}
        return {};
      case DONE:return {};
    }return {};
  }
};
// Greedy visibility reduction; the callback validates rotation, translation,
// and endpoint rotation for each proposed edge with the actual footprint.
template<class Edge>
std::vector<Pose> simplify(const std::vector<Pose>& guide,Edge edge) {
  if(guide.size()<2)return guide;
  std::vector<Pose> result{guide.front()};size_t at=0;
  while(at+1<guide.size()){
    size_t chosen=at;
    for(size_t j=guide.size()-1;j>at;--j){
      Pose target=guide[j];double heading=std::atan2(target.y-result.back().y,target.x-result.back().x);
      if(j+1<guide.size())target.yaw=heading;
      if(edge(result.back(),target)){chosen=j;result.push_back(target);break;}
    }
    if(chosen==at)throw std::runtime_error("No collision-free straight edge and in-place turn");
    at=chosen;
  }return result;
}
}
