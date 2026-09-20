#ifndef ASTRIBOT_S1_PATH_TRACKING__LINEAR_SLIP_HPP_
#define ASTRIBOT_S1_PATH_TRACKING__LINEAR_SLIP_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>

namespace astribot_s1_path_tracking
{
// SDK and SLAM have independent world frames. Compare increments in each
// window's initial body frame, never their absolute coordinates or raw twists.
class LinearSlip
{
public:
  struct Config
  {
    double window{1.5}, confirm{1.5}, max_age{0.3};
    double min_speed{0.008}, max_speed{0.06}, max_angular{0.03}, max_curvature{0.3};
    double min_distance{0.01}, low_ratio{0.35}, recovery_ratio{0.65};
    double max_extra{0.01}, max_extra_ratio{0.35}, ramp{0.01}, duration{2.0};
  };
  struct Pose {double t, received, x, y, yaw;};
  struct Command {double t, x, y, w;};
  struct Evidence
  {
    std::string state{"WARMUP"};
    double start{0.}, end{0.}, command_m{0.}, slam_m{0.}, sdk_m{0.}, ratio{0.};
    double extra{0.};
    bool valid{false};
  };

  void configure(const Config & config)
  {
    const double values[]={config.window,config.confirm,config.max_age,config.min_speed,
      config.max_speed,config.max_angular,config.max_curvature,config.min_distance,config.low_ratio,
      config.recovery_ratio,config.max_extra,config.max_extra_ratio,config.ramp,config.duration};
    for (double v:values) {
      if (!std::isfinite(v) || v<=0.) {throw std::invalid_argument("Slip parameters must be finite and positive");}
    }
    if (config.max_age>=config.window || config.min_speed>=config.max_speed ||
      config.low_ratio>=config.recovery_ratio || config.recovery_ratio>1. ||
      config.max_extra_ratio>0.5 || config.max_extra>0.02 || config.duration>3.)
    {throw std::invalid_argument("Invalid slip window, thresholds or bounded boost");}
    config_=config; reset();
  }
  const Config & config() const {return config_;}
  const Evidence & evidence() const {return evidence_;}
  void reset() {spent_=false; last_tick_=-1.; pause();}
  // Pauses revoke compensation, but do not grant another attempt without recovery.
  void pause()
  {
    slam_.clear(); sdk_.clear(); commands_.clear();
    suspect_since_=active_since_=healthy_since_=-1.; evidence_={};
  }
  void slam(const Pose & p) {addPose(slam_,p);}
  void sdk(const Pose & p) {addPose(sdk_,p);}
  void command(const Command & c)
  {
    if (!finite(c.t,c.x,c.y,c.w)) {pause(); return;}
    if (!commands_.empty() && c.t<=commands_.back().t) {
      if (c.t==commands_.back().t && c.x==commands_.back().x &&
        c.y==commands_.back().y && c.w==commands_.back().w) {return;}
      pause();
    }
    commands_.push_back(c); trim(commands_,c.t);
  }

  double apply(double now, double x, double y, double w, double cap, bool eligible, bool compensate)
  {
    const double speed=std::hypot(x,y);
    if (!finite(now,x,y,w) || !std::isfinite(cap) || cap<0. ||
      !eligible || speed<config_.min_speed || speed>config_.max_speed+1e-9 ||
      std::abs(w)>std::min(config_.max_angular,config_.max_curvature*speed) || (last_tick_>=0. &&
      (now<last_tick_ || now-last_tick_>config_.max_age))) {
      pause(); last_tick_=now; evidence_.state="INELIGIBLE"; return 0.;
    }
    last_tick_=now;
    evidence_={};
    if (!measure(now,x/speed,y/speed)) {
      suspect_since_=active_since_=healthy_since_=-1.; return 0.;
    }
    const double mean=evidence_.command_m/(evidence_.end-evidence_.start);
    if (std::abs(mean-speed)>0.015) {
      suspect_since_=active_since_=-1.; evidence_.state="COMMAND_TRANSIENT"; return 0.;
    }
    if (evidence_.ratio>=config_.recovery_ratio) {
      suspect_since_=active_since_=-1.;
      if (healthy_since_<0.) {healthy_since_=now;}
      if (now-healthy_since_>=config_.window) {spent_=false;}
      evidence_.state="TRACKING"; return 0.;
    }
    healthy_since_=-1.;
    const bool low=evidence_.ratio<config_.low_ratio;
    if (active_since_<0.) {
      if (!low) {suspect_since_=-1.; evidence_.state="MARGINAL_RESPONSE"; return 0.;}
      evidence_.state=evidence_.sdk_m>=0.5*evidence_.command_m ? "SUSPECTED_SLIP" : "LOW_RESPONSE";
      if (spent_) {evidence_.state="ATTEMPT_EXHAUSTED"; return 0.;}
      if (suspect_since_<0.) {suspect_since_=now;}
      if (!compensate || now-suspect_since_<config_.confirm) {return 0.;}
      active_since_=now; spent_=true;
    }
    const double elapsed=now-active_since_;
    if (elapsed>=config_.duration) {
      active_since_=suspect_since_=-1.; evidence_.state="ATTEMPT_EXHAUSTED"; return 0.;
    }
    // A finite pulse with a ramp at both ends; never an accumulating integrator.
    evidence_.extra=std::max(0.,std::min({config_.max_extra,config_.max_extra_ratio*speed,
      cap-speed,config_.max_speed-speed,elapsed*config_.ramp,
      (config_.duration-elapsed)*config_.ramp}));
    evidence_.state=evidence_.extra>0. ? "BOOST" : "BOOST_LIMITED";
    return evidence_.extra;
  }

private:
  static bool finite(double a,double b,double c,double d)
  {return std::isfinite(a)&&std::isfinite(b)&&std::isfinite(c)&&std::isfinite(d);}
  static double angle(double a) {return std::remainder(a,2.*std::acos(-1.));}
  template<class T> void trim(std::deque<T> & samples,double t)
  {
    while (samples.size()>2 && samples[1].t<t-config_.window-2.*config_.max_age) {samples.pop_front();}
    while (samples.size()>2048) {samples.pop_front();}
  }
  void addPose(std::deque<Pose> & samples,const Pose & p)
  {
    if (!finite(p.t,p.x,p.y,p.yaw) || !std::isfinite(p.received) ||
      p.received-p.t < -0.05 || p.received-p.t>config_.max_age) {pause(); return;}
    if (!samples.empty() && p.t<=samples.back().t) {
      const auto & last=samples.back();
      if (p.t==last.t && p.x==last.x && p.y==last.y && p.yaw==last.yaw) {return;}
      pause();
    }
    samples.push_back(p); trim(samples,p.t);
  }
  bool point(const std::deque<Pose> & samples,double t,Pose & result) const
  {
    for (std::size_t i=1;i<samples.size();++i) {
      const auto & a=samples[i-1];const auto & b=samples[i];
      if (a.t<=t && t<=b.t && b.t-a.t<=config_.max_age) {
        const double u=(t-a.t)/(b.t-a.t);
        result={t,0.,a.x+u*(b.x-a.x),a.y+u*(b.y-a.y),a.yaw+u*angle(b.yaw-a.yaw)};
        return true;
      }
    }
    return false;
  }
  bool progress(const std::deque<Pose> & samples,double dx,double dy,double & value)
  {
    Pose a{},b{};
    if (!point(samples,evidence_.start,a) || !point(samples,evidence_.end,b)) {return false;}
    Pose previous=a;
    for (const auto & p:samples) {
      if (p.t<=a.t || p.t>b.t) {continue;}
      const double dt=p.t-previous.t;
      if (dt>config_.max_age || std::hypot(p.x-previous.x,p.y-previous.y)>0.5*dt+0.003 ||
        std::abs(angle(p.yaw-a.yaw))>0.08) {return false;}
      previous=p;
    }
    if (b.t>previous.t && (b.t-previous.t>config_.max_age ||
      std::hypot(b.x-previous.x,b.y-previous.y)>0.5*(b.t-previous.t)+0.003 ||
      std::abs(angle(b.yaw-a.yaw))>0.08)) {return false;}
    const double c=std::cos(a.yaw),s=std::sin(a.yaw);
    const double along=(c*dx-s*dy)*(b.x-a.x)+(s*dx+c*dy)*(b.y-a.y);
    const double side=(-s*dx-c*dy)*(b.x-a.x)+(c*dx-s*dy)*(b.y-a.y);
    if (along < -0.003 || std::abs(side)>0.01) {return false;}
    value=std::max(0.,along); return true;
  }
  bool measure(double now,double dx,double dy)
  {
    if (slam_.size()<3 || sdk_.size()<3 || commands_.size()<2) {return false;}
    if (now-slam_.back().t>config_.max_age || now-sdk_.back().t>config_.max_age ||
      now-commands_.back().t>config_.max_age || now<slam_.back().t-0.05 ||
      now<sdk_.back().t-0.05 || now<commands_.back().t) {
      evidence_.state="STALE_INPUT"; return false;
    }
    evidence_.end=std::min({now,slam_.back().t,sdk_.back().t,commands_.back().t});
    evidence_.start=evidence_.end-config_.window;
    if (commands_.front().t>evidence_.start) {return false;}
    double span=0.,low=std::numeric_limits<double>::infinity(),high=0.;
    for (std::size_t i=1;i<commands_.size();++i) {
      const auto & c=commands_[i-1];
      const double dt=std::min(commands_[i].t,evidence_.end)-std::max(c.t,evidence_.start);
      if (dt<=0.) {continue;}
      const double speed=std::hypot(c.x,c.y),along=dx*c.x+dy*c.y;
      if (commands_[i].t-c.t>config_.max_age || speed<config_.min_speed ||
        speed>config_.max_speed+1e-9 || std::abs(c.w)>std::min(config_.max_angular,config_.max_curvature*speed) ||
        along<0.985*speed)
      {evidence_.state="COMMAND_TRANSIENT"; return false;}
      evidence_.command_m+=dt*along;span+=dt;
      low=std::min(low,speed);high=std::max(high,speed);
    }
    if (span<config_.window-1e-6 || high-low>0.015 || evidence_.command_m<config_.min_distance)
    {evidence_.state="INSUFFICIENT_EXCITATION"; return false;}
    if (!progress(slam_,dx,dy,evidence_.slam_m) || !progress(sdk_,dx,dy,evidence_.sdk_m))
    {evidence_.state="POSE_DISCONTINUITY"; return false;}
    evidence_.ratio=evidence_.slam_m/evidence_.command_m;
    evidence_.valid=true; return true;
  }
  Config config_;
  Evidence evidence_;
  std::deque<Pose> slam_,sdk_;
  std::deque<Command> commands_;
  double suspect_since_{-1.},active_since_{-1.},healthy_since_{-1.},last_tick_{-1.};
  bool spent_{false};
};
}  // namespace astribot_s1_path_tracking
#endif
