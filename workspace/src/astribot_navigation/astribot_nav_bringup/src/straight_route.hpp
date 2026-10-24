#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
namespace towergo_route {
struct Pose {double x,y,yaw;};
inline double angle(double a) {return std::atan2(std::sin(a),std::cos(a));}
template<class Free>
bool segment(Pose a, Pose b, Free free) {
  const double turn=angle(b.yaw-a.yaw);
  const int n=std::max(1,static_cast<int>(std::ceil(std::max(std::hypot(b.x-a.x,b.y-a.y)/.01,std::abs(turn)/.01))));
  for(int i=0;i<=n;++i){double t=double(i)/n;
    if(!free(Pose{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.yaw+t*turn})) return false;
  }
  return true;
}
template<class Free>
std::vector<Pose> straight(Pose start, Pose goal, Free free) {
  const double d=std::hypot(goal.x-start.x,goal.y-start.y);
  const double heading=d<1e-6?start.yaw:std::atan2(goal.y-start.y,goal.x-start.x);
  const Pose a{start.x,start.y,heading},b{goal.x,goal.y,heading};
  if(!segment(start,a,free)||!segment(a,b,free)||!segment(b,goal,free))return {};
  std::vector<Pose> route;
  const int n=std::max(1,static_cast<int>(std::ceil(d/.025)));
  for(int i=0;i<=n;++i){double t=double(i)/n;route.push_back({start.x+t*(goal.x-start.x),start.y+t*(goal.y-start.y),heading});}
  // RPP uses the terminal orientation for in-place goal rotation.
  route.back()=goal;
  return route;
}
}
