#pragma once
#include "straight_route.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
namespace towergo_route {
inline bool footprint_free(nav2_costmap_2d::Costmap2D *map,
    const std::vector<geometry_msgs::msg::Point>& footprint,Pose p) {
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(map);
  double cost=checker.footprintCostAtPose(p.x,p.y,p.yaw,footprint);
  if(cost<0||cost>=nav2_costmap_2d::LETHAL_OBSTACLE)return false;
  std::vector<std::pair<double,double>> poly;double c=cos(p.yaw),s=sin(p.yaw);
  unsigned int minx=map->getSizeInCellsX(),miny=map->getSizeInCellsY(),maxx=0,maxy=0;
  for(auto q:footprint){double x=p.x+c*q.x-s*q.y,y=p.y+s*q.x+c*q.y;unsigned int mx,my;
    if(!map->worldToMap(x,y,mx,my))return false;
    minx=std::min(minx,mx);miny=std::min(miny,my);maxx=std::max(maxx,mx);maxy=std::max(maxy,my);poly.emplace_back(x,y);}
  if(poly.size()<3)return false;
  // Boundary rasterization alone misses occupied cells enclosed by the robot.
  for(unsigned int y=miny;y<=maxy;++y)for(unsigned int x=minx;x<=maxx;++x){
    if(map->getCost(x,y)<nav2_costmap_2d::LETHAL_OBSTACLE)continue;
    double wx,wy;map->mapToWorld(x,y,wx,wy);bool inside=false;
    for(size_t i=0,j=poly.size()-1;i<poly.size();j=i++){
      auto a=poly[i],b=poly[j];
      if((a.second>wy)!=(b.second>wy) && wx<(b.first-a.first)*(wy-a.second)/(b.second-a.second)+a.first)inside=!inside;
    }if(inside)return false;
  }return true;
}
}
