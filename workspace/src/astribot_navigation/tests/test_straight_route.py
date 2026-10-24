"""Compile and exercise production route geometry without ROS or hardware."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
def test_straight_route_and_rotation_clearance(tmp_path):
    source=tmp_path/'test.cpp'
    source.write_text(r'''
#include "straight_route.hpp"
#include <cassert>
using namespace towergo_route;
int main(){
 auto clear=[](Pose){return true;};
 auto p=straight(Pose{0,0,1.4},Pose{2,0,-1.2},clear);
 assert(p.size()>50 && p.back().yaw==-1.2);
 for(size_t i=0;i+1<p.size();++i)assert(p[i].y==0 && p[i].yaw==0);
 // Final yaw cannot curve a clear translational route.
 auto q=straight(Pose{0,0,1.4},Pose{2,0,2.4},clear);
 assert(q.size()==p.size());
 for(size_t i=0;i<p.size();++i)assert(p[i].x==q[i].x && p[i].y==q[i].y);
 auto blocked=[](Pose a){return !(a.x>.95 && a.x<1.05);};
 assert(straight(Pose{0,0,0},Pose{2,0,0},blocked).empty());
 auto turn_blocked=[](Pose a){return !(a.x>1.99 && a.yaw>.4 && a.yaw<.6);};
 assert(straight(Pose{0,0,0},Pose{2,0,1.2},turn_blocked).empty());
 auto start_blocked=[](Pose a){return !(a.x<.01 && a.yaw>.4 && a.yaw<.6);};
 assert(straight(Pose{0,0,1.2},Pose{2,0,0},start_blocked).empty());
 auto at_goal=straight(Pose{2,3,0},Pose{2,3,1.},clear);
 for(auto a:at_goal)assert(a.x==2 && a.y==3);
 auto wrap=[](Pose a){return std::abs(a.yaw)>3.;};
 assert(segment(Pose{0,0,3.1},Pose{0,0,-3.1},wrap));
}
''')
    binary=tmp_path/'test'
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I'+str(ROOT/'astribot_nav_bringup/src'),str(source),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
