"""Execute production state machine and verify generated hardware configuration."""
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))

def test_stopped_corners(tmp_path):
    p=tmp_path/'test.cpp';p.write_text(r'''
#include "stop_turn.hpp"
#include <cassert>
#include <iostream>
using namespace towergo_route;
int main(){
 StopTurn f;f.plan({{0,0,1.}, {1,0,0}, {1,1,3.}},0);
 Pose p{0,0,1.};double idle=0;int rotations=0,straight=0;auto old=f.stage;
 for(int i=1;i<20000&&f.stage!=StopTurn::DONE;++i){double now=i*.02;
  bool stopped=idle>=.2;
  auto cmd=f.tick(p,stopped,now,.02,.3);
  assert(cmd.v==0||cmd.w==0);assert(cmd.v>=0&&cmd.v<=.3&&std::abs(cmd.w)<=.1);
  if(cmd.w!=0){++rotations;assert(f.stage==StopTurn::TURN||f.stage==StopTurn::FINAL_TURN);}
  if(cmd.v!=0){++straight;assert(f.stage==StopTurn::DRIVE);}
  if(old!=f.stage&&(f.stage==StopTurn::TURN||f.stage==StopTurn::DRIVE||f.stage==StopTurn::FINAL_TURN))assert(stopped);
  old=f.stage;
  p.x+=cmd.v*.02*cos(p.yaw);p.y+=cmd.v*.02*sin(p.yaw);p.yaw=angle(p.yaw+cmd.w*.02);
  if(cmd.v==0&&cmd.w==0)idle+=.02;else idle=0;
 }
 assert(f.stage==StopTurn::DONE&&rotations>0&&straight>0);
 assert(std::hypot(p.x-1,p.y-1)<=f.final_xy&&std::abs(angle(p.yaw-3.))<=f.final_yaw);
 // A timer alone, without SDK stationary evidence, never releases a hold.
 f.plan({{0,0,0},{1,0,0}},0);
 for(int i=1;i<100;++i){auto c=f.tick({0,0,0},false,i,1,.3);assert(c.v==0&&c.w==0);}
 assert(f.stage==StopTurn::HOLD_TURN);
 // New plan discards the old corner and starts with a fresh stationary hold.
 f.plan({{0,0,0},{0,1,0}},100);assert(f.next==1&&f.stage==StopTurn::HOLD_TURN);
 // Same position + different final heading contains rotation only.
 f.plan({{0,0,1.}},0);p={0,0,0};idle=1;
 for(int i=1;i<2000&&f.stage!=StopTurn::DONE;++i){auto c=f.tick(p,idle>=.2,i*.02,.02,.3);assert(c.v==0);p.yaw+=c.w*.02;if(c.w==0)idle+=.02;else idle=0;}
 assert(f.stage==StopTurn::DONE);
 // Drift correction cannot generate a moving arc.
 f.plan({{0,0,0},{1,0,0}},0);f.stage=StopTurn::DRIVE;
 auto c=f.tick({.3,.1,.5},false,1,.02,.3);assert(c.v==0&&c.w==0&&f.stage==StopTurn::HOLD_TURN);
 // A long segment also detects lateral drift before its far endpoint.
 f.plan({{0,0,0},{20,0,0}},0);f.stage=StopTurn::DRIVE;f.heading=0;f.anchor={0,0,0};
 c=f.tick({3,.04,0},false,1,.02,.3);assert(c.v>0&&c.w==0&&f.stage==StopTurn::DRIVE);
 c=f.tick({3,.00,0},false,1.1,.02,.3);assert(c.v>0&&f.deviation_since<0);
 c=f.tick({3,.04,0},false,1.2,.02,.3);assert(c.v>0);
 c=f.tick({3,.04,0},false,1.51,.02,.3);assert(c.v==0&&c.w==0&&f.stage==StopTurn::HOLD_TURN);
 // A fresh plan cannot inherit the old deviation timer.
 f.plan({{0,0,0},{20,0,0}},2);f.stage=StopTurn::DRIVE;f.heading=0;f.anchor={0,0,0};
 c=f.tick({3,.04,0},false,2.01,.02,.3);assert(c.v>0);
 // Large cross-track error remains an immediate stop.
 c=f.tick({3,.11,0},false,2.02,.02,.3);assert(c.v==0&&f.stage==StopTurn::HOLD_TURN);
 // Repeated compatible replans keep speed/stage/anchor and do not insert holds.
 f.plan({{0,0,0},{20,0,0}},3);f.stage=StopTurn::DRIVE;f.heading=0;f.anchor={0,0,0};f.last_v=.1;
 for(int i=0;i<100;++i){
  double x=3+i*.01;assert(f.replan({{x,0,0},{20,0,0}},3+i*.1));
  c=f.tick({x,0,0},false,3+i*.1,.05,.1);
  assert(c.v>.099&&c.w==0&&f.stage==StopTurn::DRIVE&&f.anchor.x==0);
 }
 // A changed next corner or goal cannot inherit a moving stage.
 assert(!f.replan({{4,0,0},{4,2,0},{20,0,0}},14));assert(f.stage==StopTurn::HOLD_TURN);
 f.stage=StopTurn::DRIVE;
 assert(!f.replan({{4,0,0},{4,2,0},{20,0,1}},15));assert(f.stage==StopTurn::HOLD_TURN);
 // Visibility simplification keeps a mandatory corner around a blocked diagonal.
 auto route=simplify(std::vector<Pose>{{0,0,0},{1,0,0},{1,1,1.}},[](Pose a,Pose b){return a.x==b.x||a.y==b.y;});assert(route.size()==3);
 auto direct=simplify(route,[](Pose,Pose){return true;});assert(direct.size()==2&&direct.back().yaw==1.);
 std::cout<<"stop-turn state machine passed\n";
}
''')
    subprocess.run(['g++','-std=c++17','-I'+str(ROOT/'astribot_nav_bringup/src'),str(p),'-o',str(tmp_path/'test')],check=True)
    subprocess.run([str(tmp_path/'test')],check=True)

def test_real_profile_selects_continuous_planner_controller_goal_checker():
    import yaml
    from real_navigation.session import nav_params
    config=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
    p=nav_params(config,'/tmp/map.yaml',False)['towergo']['nav']
    assert p['planner_server']['ros__parameters']['GridBased']['plugin']=='astribot_nav_bringup/ExactGoalLattice'
    controller=p['controller_server']['ros__parameters']
    assert controller['FollowPath']['plugin']=='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    assert controller['FollowPath']['rotate_to_heading_min_angle']==.7
    assert controller['FollowPath']['use_collision_detection'] is True
    assert controller['goal_checker']['plugin']=='nav2_controller::SimpleGoalChecker'
    assert p['local_costmap']['local_costmap']['ros__parameters']['plugins']==['static_layer','inflation_layer']
    assert p['bt_navigator']['ros__parameters']['default_server_timeout']==1000
    assert p['bt_navigator']['ros__parameters']['bt_loop_duration']==50
    tree=(ROOT/'scripts/real_navigation/navigation.xml').read_text()
    assert '<PipelineSequence ' in tree and '<RateController hz="1.0">' in tree and '<Wait ' not in tree
