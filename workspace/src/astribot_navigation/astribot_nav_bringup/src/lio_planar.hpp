#pragma once
#include <cmath>

namespace towergo {
inline double wrap(double x) { return std::atan2(std::sin(x),std::cos(x)); }
struct PlanarStep {
  double x, y, yaw, sx, sy, sa;
};
// LIO measures a planar rigid transform, including lateral displacement. Do not
// invent two large opposite rotations from the direction of a tiny translation.
inline PlanarStep lio_step(double old_yaw,double dx,double dy,double da,
                          double a1,double a2,double a3,double a4,double a5) {
  const double c=std::cos(old_yaw),s=std::sin(old_yaw);
  const double d2=dx*dx+dy*dy, yaw=wrap(da),r2=yaw*yaw;
  return {c*dx+s*dy,-s*dx+c*dy,yaw,
          std::sqrt(a3*d2+a4*r2),std::sqrt(a5*d2+a4*r2),std::sqrt(a1*r2+a2*d2)};
}
inline void apply_step(double &x,double &y,double &yaw,const PlanarStep &d,
                       double nx=0,double ny=0,double na=0) {
  const double c=std::cos(yaw),s=std::sin(yaw);
  x+=c*(d.x+nx)-s*(d.y+ny);y+=s*(d.x+nx)+c*(d.y+ny);
  yaw=wrap(yaw+d.yaw+na);
}
}
