#ifndef HIGHRATE_ODOM_HPP
#define HIGHRATE_ODOM_HPP

#include <filesystem>
#include <unistd.h>

// Upsamples the ~10Hz LiDAR-corrected pose (x_curr) to a 20Hz tf stream for
// the motion-control consumer, by continuously dead-reckoning the IMU between
// LiDAR frames and re-anchoring to the corrected pose as soon as each scan's
// optimization finishes.
//
// Why re-anchor by *replaying* buffered IMU instead of just accepting x_curr
// as-is: x_curr's timestamp (pcl_end_time) is already in the past by the time
// optimization finishes (processing latency L), and during that L the
// steady-state propagation below was running on the *previous* (pre-
// correction) bias estimate. Re-integrating the trailing L worth of IMU with
// the freshly-corrected bias removes that bias-driven drift instead of
// carrying it forward.
//
// anchor() always overwrites `state` outright with this replayed-forward
// x_curr — every published tick is always the best currently-available
// estimate, never a knowingly-transient blend. The cost: real IMU-only
// dead-reckoning between corrections does drift — worst on z, the least-
// observable axis for a LiDAR-IMU setup — and since that drift only gets
// removed in a discrete jump whenever the next correction lands, the
// published tf has a visible pop/sawtooth once every scan. Smoothing that
// pop away (a fixed-ratio ramp spreading the correction over ~2 ticks, and
// separately a covariance-weighted fusion) was tried and measured — both
// reduced the pop but at the cost of the ~1-tick window right after each
// anchor being a knowing blend rather than the best available estimate;
// removed once the decision was made to always publish the optimal
// estimate and accept the pop instead. See git history for the numbers
// if reviving either is ever worth it.
//
// anchor() only updates the running state; only the 20Hz timer
// (publish_tick()) ever calls publish_tf(), so correction events (per scan,
// ~10Hz, plus loop-closure snaps) never distort the output's cadence.
//
// Threading note: sub_imu and this class's 20Hz timer both run on their own
// dedicated executor/thread (g_imu_cbg, set up in main()) — see that
// variable's comment in voxelslam.hpp for why: thd_odometry_localmapping's
// rclcpp::spin_some(n) also runs each scan's (sometimes 100+ms) optimization
// inline, and anything sharing that thread would stall for just as long.
// anchor()/reset() do still run on the odometry thread (called next to
// pub_localtraj()/loop_update()/system_reset()), so they genuinely race
// on_imu()/publish_tick() now — hence `mtx`, no longer just defensive.

#include "tools.hpp"
#include "ekf_imu.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

class HighRateOdom
{
public:
  static HighRateOdom &instance()
  {
    static HighRateOdom inst;
    return inst;
  }

  // Set once at startup from General.enable_highrate_odom (see main()).
  // false reproduces the pre-this-feature behavior: no 20Hz timer output,
  // anchor() publishes immediately instead (~10Hz, LiDAR-frame-rate,
  // exactly like the old pub_odom_func) — a fallback to switch back to
  // during testing without reverting code.
  void set_enabled(bool e) { enabled = e; }
  void set_max_pose_age(double seconds) { max_pose_age = seconds; }

  // TUM trajectory data, deliberately separate from the rotating session.log.
  // Unique per process/run: restarting SLAM must not truncate earlier evidence.
  void open_log()
  {
    lock_guard<mutex> lk(mtx);
    if(log_ofs.is_open()) return;
    try
    {
      const auto directory = std::filesystem::path(vxlm_log::log_dir()) / "artifacts";
      std::filesystem::create_directories(directory);
      const auto path = directory / ("highrate_tf_" + std::to_string(getpid()) + "_" +
          std::to_string(vxlm_log::now_monotonic_ns()) + ".txt");
      log_ofs.open(path, ios::out);
      if(!log_ofs) throw std::runtime_error("cannot open " + path.string());
      LOG_INFO(SYS, "TF trajectory artifact | path:{}", path.string());
    }
    catch(const std::exception &error)
    {
      LOG_ERROR(SYS, "TF trajectory recording unavailable | reason:{}", error.what());
    }
  }

  // Called from imu_handler() for every IMU sample, in addition to (not
  // instead of) the existing imu_buf/sync_packages path — this keeps its own
  // buffer and does not affect scan de-skewing at all.
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr &msg)
  {
    lock_guard<mutex> lk(mtx);

    if(ready && !buf.empty())
      integrate_pair(state, *buf.back(), *msg);

    buf.push_back(msg);

    double newest = stamp2sec(msg->header.stamp);
    while(buf.size() > 1 && stamp2sec(buf.front()->header.stamp) < newest - retention_sec)
      buf.pop_front();
  }

  // Called right after a scan's LiDAR-involved pose optimization finishes
  // (x_curr is the corrected state, x_curr.t == pcl_end_time). Re-anchors
  // the running estimate to it — replaying whatever IMU has arrived since
  // x_curr.t to catch back up to "now" — and overwrites `state` with the
  // result outright (see the top-of-file comment for the tradeoff this
  // implies).
  //
  // Only ever called after anchor_init() has already run once (see
  // anchor_init() below for the first post-initialization anchor) — by then
  // `ready` is already true and buf has been tracking real-time IMU
  // continuously since, so x_curr.t (normally within L, ~100+ms, of "now")
  // is always well inside buf's retention_sec window and the replay below
  // is exactly "the trailing L worth of IMU" the class is meant to replay.
  //
  // Deliberately does NOT publish here — the 20Hz wall timer
  // (publish_tick()) is the only thing that calls publish_tf(), so the tf
  // stream stays on one clean, uniformly-spaced 20Hz cadence no matter
  // when/how often anchor() fires (per scan, ~10Hz, plus loop-closure
  // snaps).
  //
  // When disabled (General.enable_highrate_odom: 0), publish_tick() is a
  // no-op, so this publishes directly instead — same timing as the old
  // pub_odom_func (once per anchor(), i.e. ~10Hz at LiDAR-frame rate).
  void anchor(const IMUST &x_curr)
  {
    lock_guard<mutex> lk(mtx);

    IMUST target = x_curr;
    double anchor_t = x_curr.t;
    for(size_t i=0; i+1<buf.size(); i++)
    {
      sensor_msgs::msg::Imu &head = *buf[i];
      sensor_msgs::msg::Imu &tail = *buf[i+1];
      if(stamp2sec(tail.header.stamp) <= anchor_t) continue;
      integrate_pair(target, head, tail, anchor_t);
    }

    publish_anchor_pose(x_curr);
    last_anchor_t = x_curr.t;
    state = target;
    ready = true;

    if(!enabled)
      publish_tf(state);
  }

  // Called exactly once per init/re-init: the first anchor() after
  // motion_init() succeeds (see voxelslam.cpp's main loop). Deliberately
  // does NOT replay buf the way anchor() does — buf, up to this call, holds
  // whatever raw IMU on_imu() accumulated during initialization/
  // motion_init()'s (slower-than-a-single-scan) window optimization, none of
  // which is a "trailing L worth of IMU" relative to this x_curr the way
  // anchor()'s replay assumes. So: discard it outright, take x_curr as-is,
  // and publish it immediately right here — not waiting for the next 50ms
  // tick — so the very first pose the tf consumer sees is this optimized
  // anchor, not silence followed later by an initial pose. `ready` only
  // flips true after that publish, which is what actually gates
  // publish_tick() (see below) into publishing anything at all — so in
  // effect the 20Hz stream only starts now, at this anchor, even though the
  // timer object itself (created once at node startup, see main()) has been
  // ticking harmlessly the whole time. Every anchor() call after this one is
  // the normal replay-based re-anchor.
  //
  // hr_odom_timer->reset() (rclcpp::TimerBase, declared in voxelslam.hpp,
  // already in scope by the time this file is included there) restarts its
  // 50ms countdown from *now* instead of leaving it on whatever phase it's
  // been ticking at since node startup — without this, the gap between this
  // call's direct publish_tf() and publish_tick()'s first real firing would
  // be a leftover phase offset (anywhere in [0, 50ms), not 50ms), and only
  // every tick after that one would be a clean 50ms apart. Safe to call
  // under `mtx`: it's a separate, internally-locked rclcpp mutex, and
  // rcl_timer_reset() is documented safe to call from any thread while the
  // timer's executor (g_imu_cbg here) keeps firing it concurrently.
  void anchor_init(const IMUST &x_curr)
  {
    lock_guard<mutex> lk(mtx);

    publish_anchor_pose(x_curr);
    last_anchor_t = x_curr.t;
    state = x_curr;
    buf.clear();
    publish_tf(state);
    ready = true;

    if(hr_odom_timer)
      hr_odom_timer->reset();
  }


  // Called from system_reset() so a stale pre-reset state/buffer never gets
  // propagated across the reset.
  void reset()
  {
    lock_guard<mutex> lk(mtx);
    buf.clear();
    ready = false;
    // Otherwise the next publish_tf() after reset would measure its
    // interval against a pre-reset timestamp, logging one bogus huge gap
    // (or divide-by-near-zero frequency) that has nothing to do with the
    // actual publish cadence.
    pub_last_ts = -1.0;
    pub_win_start_ts = -1.0;
    pub_win_count = 0;
  }

  // 20Hz wall-timer callback. No-op when disabled — anchor() publishes
  // directly in that case instead (see above).
  void publish_tick()
  {
    lock_guard<mutex> lk(mtx);
    if(!enabled || !ready) return;
    const double now = g_node->get_clock()->now().seconds();
    // Do not present unbounded IMU propagation as a fresh localized pose.
    if(now - last_anchor_t > max_pose_age || now - state.t > max_pose_age || now < state.t) return;
    publish_tf(state);
  }

private:
  HighRateOdom() = default;

  mutex mtx;
  deque<sensor_msgs::msg::Imu::SharedPtr> buf;
  IMUST state;
  double last_anchor_t = 0.0, max_pose_age = 0.5;
  bool ready = false;
  bool enabled = true;
  ofstream log_ofs;
  // Trailing window kept for anchor() replay — must cover the worst-case
  // scan-optimization latency; 1s is generous headroom over that.
  const double retention_sec = 1.0;

  // Actual publish-cadence tracking for the "pose publish" PERF line (see
  // publish_tf() below) — measures the real interval between successive
  // publish_tf() calls, whichever path drives them (20Hz timer when
  // `enabled`, direct per-anchor() publish otherwise — see publish_tick()/
  // anchor() above), so the logged frequency is what actually left the
  // process, not an assumed nominal rate. Aggregated over a ~1s rolling
  // window rather than logged per-tick: at 20Hz that would be 20x the
  // odometry-loop line's rate and would drown it out for no benefit, since
  // a single tick's interval is too noisy on its own to be worth reporting.
  double pub_last_ts = -1.0;
  double pub_win_start_ts = -1.0;
  double pub_win_sum = 0.0, pub_win_sumsq = 0.0;
  double pub_win_min = 0.0, pub_win_max = 0.0;
  int pub_win_count = 0;

  // Mean/midpoint integration of `state` from head's timestamp to tail's,
  // identical in form to ekf_imu.hpp's motion_blur core (bias-corrected
  // mean angular velocity/acceleration, R*acc+g for the world-frame
  // acceleration), minus the covariance propagation — this stream only
  // feeds a tf publish, not the estimator. `clamp_from`, when given,
  // clamps the integration start to max(head_time, clamp_from) the same way
  // motion_blur clamps to last_pcl_end_time, so replay starting mid-interval
  // (from x_curr.t) doesn't double-integrate the part before the anchor.
  static void integrate_pair(IMUST &st, const sensor_msgs::msg::Imu &head,
                              const sensor_msgs::msg::Imu &tail,
                              double clamp_from = -1.0)
  {
    double head_t = stamp2sec(head.header.stamp);
    double tail_t = stamp2sec(tail.header.stamp);
    if(clamp_from > head_t) head_t = clamp_from;
    double dt = tail_t - head_t;
    if(dt <= 0) return;

    Eigen::Vector3d angvel_avr, acc_avr;
    angvel_avr << 0.5*(head.angular_velocity.x + tail.angular_velocity.x),
                  0.5*(head.angular_velocity.y + tail.angular_velocity.y),
                  0.5*(head.angular_velocity.z + tail.angular_velocity.z);
    acc_avr << 0.5*(head.linear_acceleration.x + tail.linear_acceleration.x),
               0.5*(head.linear_acceleration.y + tail.linear_acceleration.y),
               0.5*(head.linear_acceleration.z + tail.linear_acceleration.z);

    angvel_avr -= st.bg;
    // Scale by imupre_scale_gravity *before* subtracting bias, matching
    // motion_blur (ekf_imu.hpp:74) and preintegration.hpp:69 exactly — this
    // used to be omitted here on the assumption that it's always 1.0 (true
    // for an accelerometer that already reports m/s^2), but that assumption
    // doesn't hold for every IMU this runs against, and skipping it here
    // while the main EKF applies it introduces a scale mismatch that's
    // dominated by gravity on a level, static rig — i.e. it shows up almost
    // entirely as a z-axis error, growing with however long this class
    // free-integrates between corrections (small right after anchor(), up
    // to ~1 scan period by the next one), not as a bounded per-tick noise
    // term. imupre_scale_gravity is declared in preintegration.hpp, pulled
    // in transitively (voxelslam.hpp includes voxel_map.hpp before
    // highrate_odom.hpp) — it *is* reachable here after all.
    acc_avr = acc_avr * imupre_scale_gravity - st.ba;
    Eigen::Vector3d acc_imu = st.R * acc_avr + st.g;

    st.p = st.p + st.v * dt + 0.5 * acc_imu * dt * dt;
    st.v = st.v + acc_imu * dt;
    st.R = st.R * Exp(angvel_avr, dt);
    st.t = tail_t;
  }


  // Publish the LiDAR-corrected anchor, with its estimator covariance. IMU-only
  // extrapolation has no covariance propagation here and stays a TF-only output.
  void publish_anchor_pose(const IMUST &xc)
  {
    if(!g_world_registered) return;
    static auto pub = g_node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/slam/pose", rclcpp::SensorDataQoS());
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    imu_pose_to_chassis(xc.R, xc.p, R, p);
    const Eigen::Vector3d lever = xc.R.transpose() * (p - xc.p);
    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
    J.block<3,3>(0,0) = -xc.R * hat(lever);
    J.block<3,3>(0,3).setIdentity();
    J.block<3,3>(3,0) = xc.R;
    const Eigen::Matrix<double, 6, 6> covariance = J * xc.cov.block<6,6>(0,0) * J.transpose();
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(xc.t * 1e9));
    msg.pose.pose.position.x = p.x(); msg.pose.pose.position.y = p.y(); msg.pose.pose.position.z = p.z();
    const Eigen::Quaterniond q(R);
    msg.pose.pose.orientation.x = q.x(); msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z(); msg.pose.pose.orientation.w = q.w();
    for(int i=0; i<6; ++i) for(int j=0; j<6; ++j) msg.pose.covariance[i*6+j] = covariance(i,j);
    pub->publish(msg);
  }

  void publish_tf(const IMUST &xc)
  {
    if(!g_world_registered) return;
    Eigen::Matrix3d R_chassis;
    Eigen::Vector3d t_chassis;
    imu_pose_to_chassis(xc.R, xc.p, R_chassis, t_chassis);

    Eigen::Quaterniond q_this(R_chassis);

    if(log_ofs.is_open())
    {
      log_ofs << fixed << setprecision(6) << xc.t << " ";
      log_ofs << setprecision(7) << t_chassis[0] << " " << t_chassis[1] << " " << t_chassis[2] << " ";
      log_ofs << q_this.x() << " " << q_this.y() << " " << q_this.z() << " " << q_this.w() << endl;
    }

    static tf2_ros::TransformBroadcaster br(g_node);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(xc.t * 1e9));
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "aft_mapped";
    tf_msg.transform.translation.x = t_chassis.x();
    tf_msg.transform.translation.y = t_chassis.y();
    tf_msg.transform.translation.z = t_chassis.z();
    tf_msg.transform.rotation.w = q_this.w();
    tf_msg.transform.rotation.x = q_this.x();
    tf_msg.transform.rotation.y = q_this.y();
    tf_msg.transform.rotation.z = q_this.z();
    br.sendTransform(tf_msg);

    log_publish_rate();
  }

  // Tracks the real interval between successive publish_tf() calls and
  // reports it as a "[PERF] pose publish" line, throttled to ~once/sec by
  // wall-clock time (not call count — the call rate itself differs between
  // the 20Hz-timer and direct-anchor() paths, see the field comments at
  // pub_last_ts's declaration) so it lands at roughly the same cadence as
  // voxelslam.cpp's "[PERF] odometry loop" line and the two are easy to
  // eyeball together in session.log. This is deliberately the actual
  // measured cadence of what left the process, not 1/total_ms of the
  // odometry loop — the two are architecturally decoupled (see this file's
  // top-of-file comment) and only this line answers "what is the real
  // pose-publish frequency".
  void log_publish_rate()
  {
    double now = (g_node ? g_node->get_clock()->now() : rclcpp::Clock().now()).seconds();
    if(pub_last_ts > 0)
    {
      double interval = now - pub_last_ts;
      if(pub_win_count == 0)
      {
        pub_win_start_ts = now;
        pub_win_min = pub_win_max = interval;
      }
      else
      {
        pub_win_min = std::min(pub_win_min, interval);
        pub_win_max = std::max(pub_win_max, interval);
      }
      pub_win_sum += interval;
      pub_win_sumsq += interval * interval;
      pub_win_count++;

      if(now - pub_win_start_ts >= 1.0)
      {
        double mean = pub_win_sum / pub_win_count;
        double var = pub_win_sumsq / pub_win_count - mean * mean;
        double jitter_std = var > 0 ? std::sqrt(var) : 0.0;
        LOG_INFO(PERF, "pose publish | freq:{:.2f}Hz mean:{:.1f}ms min:{:.1f}ms max:{:.1f}ms jitter:{:.2f}ms n:{} mode:{}",
                 1.0 / mean, mean * 1000.0, pub_win_min * 1000.0, pub_win_max * 1000.0,
                 jitter_std * 1000.0, pub_win_count, enabled ? "20hz_timer" : "direct_anchor");
        pub_win_count = 0;
        pub_win_sum = pub_win_sumsq = 0.0;
      }
    }
    pub_last_ts = now;
  }
};

#endif
