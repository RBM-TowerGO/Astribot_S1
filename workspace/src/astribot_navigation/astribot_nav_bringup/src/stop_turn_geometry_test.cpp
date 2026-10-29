#include "route_collision.hpp"
#include <cassert>
using namespace towergo_route;
int main(){
 nav2_costmap_2d::Costmap2D map(100,100,.05,-2.5,-2.5,0);
 std::vector<geometry_msgs::msg::Point> fp;
 for(auto xy:std::vector<std::pair<double,double>>{{.6,.2},{.6,-.2},{-.6,-.2},{-.6,.2}}){geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;fp.push_back(p);}
 auto free=[&](Pose p){return footprint_free(&map,fp,p);};
 assert(free({0,0,0}));
 auto set=[&](double x,double y,unsigned char cost){unsigned int mx,my;assert(map.worldToMap(x,y,mx,my));map.setCost(mx,my,cost);};
 set(.1,.1,254);assert(!free({0,0,0})); // enclosed obstacle, not merely perimeter
 set(.1,.1,0);set(.1,.1,255);assert(!free({0,0,0}));set(.1,.1,0);
 set(0,.5,254);assert(free({0,0,0}));assert(!segment(Pose{0,0,0},Pose{0,0,1.57},free));
 set(0,.5,0);assert(segment(Pose{0,0,0},Pose{0,0,1.57},free));
 assert(!free({2.3,0,0})); // unknown outside map is never traversable
 set(1,0,254);assert(!segment(Pose{0,0,0},Pose{1.5,0,0},free));
 map.resetMap(0,0,100,100);
 for(int x=0;x<100;++x){double wx,wy;map.mapToWorld(x,50,wx,wy);set(wx,.3,254);set(wx,-.3,254);}
 assert(segment(Pose{-1,0,0},Pose{1,0,0},free)); // narrow aisle is valid straight
 assert(!segment(Pose{0,0,0},Pose{0,0,1.57},free)); // but cannot rotate here

}
