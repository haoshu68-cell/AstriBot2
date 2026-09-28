#pragma once
#include "astribot_s1_path_tracking/arrival_settling.hpp"

namespace astribot_s1_path_tracking {
class CornerStopEvidence {
public:
  void reset() {xy_.reset();yaw_.reset();}
  void command(double vx,double vy,double wz,double now) {
    xy_.command(std::hypot(vx,vy)<=1e-9,now);
    yaw_.command(std::abs(wz)<=1e-9,now);
  }
  bool observe(double stamp,double now,double x,double y,double yaw,
    double duration,double max_gap,double angular_limit,double xy_drift,double yaw_drift)
  {
    xy_.observe(stamp,now,{x,y},duration,max_gap,.01,xy_drift);
    yaw_.observe(stamp,now,{yaw},duration,max_gap,angular_limit,yaw_drift,true);
    return xy_.evidence().stopped && yaw_.evidence().stopped;
  }
private:
  ArrivalSettling<2> xy_;
  ArrivalSettling<1> yaw_;
};
}
