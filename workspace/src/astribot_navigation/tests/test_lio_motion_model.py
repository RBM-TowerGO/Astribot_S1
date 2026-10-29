"""Compile the actual model math and test transforms, continuity and reported case."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]

def test_planar_lio_motion(tmp_path):
    source=tmp_path/'test.cpp'
    source.write_text(r'''
#include "lio_planar.hpp"
#include <cassert>
#include <iostream>
using namespace towergo;
bool near(double a,double b){return std::abs(a-b)<1e-9;}
int main(){
  // Same odometry step applied to different particle headings; no fake rotations.
  auto d=lio_step(M_PI/2,0,1,.2,0,0,0,0,0);
  assert(near(d.x,1)&&near(d.y,0));
  double x=0,y=0,a=0;apply_step(x,y,a,d);assert(near(x,1)&&near(y,0)&&near(a,.2));
  x=0;y=0;a=M_PI/2;apply_step(x,y,a,d);assert(near(x,0)&&near(y,1));
  for(double dx:{-1.,0.,1.})for(double dy:{-1.,0.,1.}) {
    auto p=lio_step(.6,dx,dy,.1,0,0,0,0,0);x=2;y=3;a=.6;
    apply_step(x,y,a,p);assert(near(x,2+dx)&&near(y,3+dy)&&near(a,.7));
  }
  assert(near(lio_step(0,0,0,-2*M_PI+.1,0,0,0,0,0).yaw,.1));
  auto z=lio_step(0,0,0,0,.05,.05,.05,.05,.05);assert(z.sx==0&&z.sy==0&&z.sa==0);
  // Evidence: 1.1995 cm, 2.8979 deg, 83.1883 deg translation direction.
  double dist=.0119950198,theta=2.8979*M_PI/180,angle=83.1883*M_PI/180;
  auto e=lio_step(0,dist*cos(angle),dist*sin(angle),theta,.05,.05,.05,.05,.05);
  assert(e.sx<.012&&e.sy<.012&&e.sa<.012);
  auto before=lio_step(0,0,.009999,theta,.05,.05,.05,.05,.05);
  auto after=lio_step(0,0,.010001,theta,.05,.05,.05,.05,.05);
  assert(std::abs(before.sx-after.sx)<1e-6);
  // A genuine large translation still injects correspondingly large uncertainty.
  assert(lio_step(0,1,0,0,.05,.05,.05,.05,.05).sx>.2);
  std::cout<<"evidence sigma translation="<<e.sx<<" m yaw="<<e.sa*180/M_PI<<" deg\n";
}
''')
    subprocess.run(['g++','-std=c++17','-I'+str(ROOT/'astribot_nav_bringup/src'),str(source),'-o',str(tmp_path/'test')],check=True)
    subprocess.run([str(tmp_path/'test')],check=True)
