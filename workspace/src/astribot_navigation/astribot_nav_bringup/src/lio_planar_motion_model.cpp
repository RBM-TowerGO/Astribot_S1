#include "lio_planar.hpp"
#include "nav2_amcl/motion_model/motion_model.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <stdexcept>

namespace towergo {
class LioPlanarMotionModel : public nav2_amcl::MotionModel {
 public:
  void initialize(double a1,double a2,double a3,double a4,double a5) override {
    for (double a:{a1,a2,a3,a4,a5})
      if (!std::isfinite(a) || a<0) throw std::invalid_argument("Invalid LIO motion noise alpha");
    a1_=a1;a2_=a2;a3_=a3;a4_=a4;a5_=a5;
  }
  void odometryUpdate(pf_t *pf,const pf_vector_t &pose,const pf_vector_t &delta) override {
    const auto d=lio_step(pose.v[2]-delta.v[2],delta.v[0],delta.v[1],delta.v[2],a1_,a2_,a3_,a4_,a5_);
    auto &set=pf->sets[pf->current_set];
    for(int i=0;i<set.sample_count;++i) {
      auto &p=set.samples[i].pose;
      apply_step(p.v[0],p.v[1],p.v[2],d,pf_ran_gaussian(d.sx),pf_ran_gaussian(d.sy),pf_ran_gaussian(d.sa));
    }
  }
 private:
  double a1_=0,a2_=0,a3_=0,a4_=0,a5_=0;
};
}
PLUGINLIB_EXPORT_CLASS(towergo::LioPlanarMotionModel,nav2_amcl::MotionModel)
