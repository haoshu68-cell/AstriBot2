#pragma once

#include "tools.hpp"
#include "ekf_imu.hpp"
#include "voxel_map.hpp"
#include "feature_point.hpp"
#include "robot_self_filter.hpp"
#include "initial_map_pose.hpp"
#include "loop_refine.hpp"
#include <mutex>
#include <Eigen/Eigenvalues>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <malloc.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <malloc.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include "BTC.h"
#include <csignal>
#include <atomic>
#include <std_msgs/msg/string.hpp>
#include <cstdlib>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/imgcodecs.hpp>
#include <astribot_slam_msgs/msg/keyframe_submap.hpp>
#include <astribot_slam_msgs/msg/keyframe_pose_update.hpp>
#include <astribot_slam_msgs/msg/keyframe_pose_array.hpp>
#include <sstream>
#include <iomanip>
#include <ctime>

using namespace std;

// Set by our own SIGINT handler (installed in main(), replacing rclcpp's
// default one) so the worker threads below can unwind cleanly *before*
// rclcpp::shutdown() tears down the DDS layer. Letting rclcpp's default
// handler shut the context down directly races with whichever thread is
// mid-spin/mid-publish at the moment Ctrl-C is pressed, and reliably
// segfaults on exit; checking this flag first and joining every thread
// before shutdown avoids that race.
std::atomic<bool> g_request_shutdown{false};
std::atomic<bool> g_world_registered{false};
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_slam_status;
inline void publish_slam_status(const std::string &state)
{
  std_msgs::msg::String msg;
  msg.data = state;
  pub_slam_status->publish(msg);
  LOG_INFO(SYS, "SLAM state: {}", state);
}


void voxelslam_sigint_handler(int)
{
  g_request_shutdown = true;
}

// --- Per-thread CPU-time-vs-wall-time tracking -----------------------------
// Diagnostic-only: lets us tell "this thread was genuinely busy" apart from
// "this thread was ready to run but the OS didn't schedule it", which is the
// open question behind the IMU rate-drop episodes (see imu_handler below).
// Each of the 4 long-lived threads (main odometry/local-mapping,
// dedicated IMU executor, loop-closure, global-mapping) periodically calls
// sample_thread_cpu() on ITSELF (CLOCK_THREAD_CPUTIME_ID only reports the
// calling thread's own CPU time — there is no portable way to query a
// different thread's from outside) and stores {utilization%} into its own
// slot below. Any thread can then read every slot as a plain atomic load —
// no cross-thread clock queries, no locking — which is what the IMU
// rate-drop check uses to log a consolidated snapshot of all 4 threads at
// the moment it fires.
enum ThreadCpuSlotId { TSLOT_MAIN = 0, TSLOT_IMU = 1, TSLOT_LOOP = 2, TSLOT_GBA = 3, TSLOT_COUNT = 4 };

inline const char *thread_slot_name(int slot)
{
  switch(slot)
  {
    case TSLOT_MAIN: return "main_odom";
    case TSLOT_IMU:  return "imu_exec";
    case TSLOT_LOOP: return "loop_closure";
    case TSLOT_GBA:  return "global_mapping";
  }
  return "?";
}

struct ThreadCpuSlot
{
  std::atomic<double> util_pct{-1.0};    // -1 = not sampled yet
  std::atomic<double> last_cpu_ms{0.0};
  std::atomic<double> last_wall_ms{0.0};
};
ThreadCpuSlot g_thread_cpu[TSLOT_COUNT];

// This thread's own CPU time in ms (0.0 if CLOCK_THREAD_CPUTIME_ID isn't
// available on this platform — Linux always has it).
inline double thread_cpu_time_ms()
{
  timespec ts;
  if(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) != 0)
    return 0.0;
  return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

// Call periodically (every ~1s is plenty) from inside the thread owning
// `slot`, passing that thread's own current wall-clock time in ms (any
// monotonic/wall source is fine as long as the same thread uses the same
// one call to call). Computes % CPU utilization since this slot's last
// sample and stores it; the first call on a given slot just seeds the
// baseline (util_pct stays -1 until a second call gives it something to
// diff against).
inline void sample_thread_cpu(int slot, double wall_ms_now)
{
  ThreadCpuSlot &s = g_thread_cpu[slot];
  double cpu_ms_now = thread_cpu_time_ms();
  double prev_cpu = s.last_cpu_ms.load(std::memory_order_relaxed);
  double prev_wall = s.last_wall_ms.load(std::memory_order_relaxed);
  if(prev_wall > 0.0)
  {
    double d_wall = wall_ms_now - prev_wall;
    if(d_wall > 0.0)
      s.util_pct.store(100.0 * (cpu_ms_now - prev_cpu) / d_wall, std::memory_order_relaxed);
  }
  s.last_cpu_ms.store(cpu_ms_now, std::memory_order_relaxed);
  s.last_wall_ms.store(wall_ms_now, std::memory_order_relaxed);
}

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan, pub_cmap, pub_init, pub_pmap;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_test, pub_prev_path, pub_curr_path;
// Height-ROI-filtered version of /map_scan, for navigation obstacle detection.
// Keeps only points whose world-frame z falls in [nav_scan_z_min, nav_scan_z_max]
// (see General.nav_scan_z_min/max) — /map_scan itself is left untouched.
// Published as plain PointCloud2 (same as /map_scan) — the standalone
// nav_prob_grid package subscribes to this and does its own occupancy-grid
// rasterization/accumulation; voxel_slam doesn't build a grid itself.
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_filtered;
// Cartographer-submap-style keyframe interface for downstream mapping
// (nav_prob_grid): each keyframe's own height-ROI-filtered local-frame
// point cloud is sent once, right when that keyframe is finalized
// (pub_keyframe_submap); its pose is republished — without resending the
// cloud — every time voxel_slam's own BTC loop closure / GTSAM pose-graph
// optimization / HBA global BA corrects one or more keyframes
// (pub_keyframe_pose_array). See astribot_slam_msgs for the message shapes.
rclcpp::Publisher<astribot_slam_msgs::msg::KeyframeSubmap>::SharedPtr pub_keyframe_submap;
rclcpp::Publisher<astribot_slam_msgs::msg::KeyframePoseArray>::SharedPtr pub_keyframe_pose_array;
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc2;
// 20Hz aft_mapped tf tick — see HighRateOdom::publish_tick(), created in main().
rclcpp::TimerBase::SharedPtr hr_odom_timer;
// sub_imu and hr_odom_timer both live in this callback group, which is
// spun on its own dedicated thread/executor (see main()) instead of
// through thd_odometry_localmapping's rclcpp::spin_some(n) — that loop
// also runs the (sometimes 100+ms) per-scan optimization inline, so an IMU
// callback/timer sharing its thread stalls for just as long, then
// delivers everything queued in one burst. Created with
// automatically_add_to_executor_with_node=false so spin_some(n) never
// touches it too (which would double-service it and race the dedicated
// thread).
rclcpp::CallbackGroup::SharedPtr g_imu_cbg;
// Optional second lidar ("lidar_back", General.lid_topic_back). Only created
// when that topic is non-empty; its scans are re-expressed into the front
// lidar's frame (see g_R_back_front/g_t_back_front below) and fed into the
// same pcl_buf/time_buf stream as the front lidar via pcl_handler_back().
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc2_back;
// Optional RGB-D color image sink (General.image_topic). Only created when
// that topic is non-empty; see image_handler()/image_buf below.
rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image;
// Shared node handle, set once in main(). ROS1's free functions/singletons (e.g.
// HighRateOdom::publish_tf's tf broadcaster, VOXEL_SLAM::system_reset's log)
// had no equivalent of a node handle to reach for outside of the ones passed
// explicitly into constructors/threads; this fills that gap.
rclcpp::Node::SharedPtr g_node;

// Fixed IMU->chassis extrinsic (p_imu = g_R_imu_chassis * p_chassis + g_t_imu_chassis),
// derived at startup in main() from General.extrinsic_{tran,rota} (lidar->IMU) and
// General.chassis_extrinsic_{tran,rota} (lidar_front->chassis). Only ever used to
// re-express an already-solved IMU pose as a chassis-center pose for output (tf,
// chassis trajectory export) — the EKF/BA/PGO state itself stays in the IMU frame,
// since gyro/acc measurements are only physically valid as body=IMU kinematics.
Eigen::Matrix3d g_R_imu_chassis = Eigen::Matrix3d::Identity();
Eigen::Vector3d g_t_imu_chassis = Eigen::Vector3d::Zero();

// Inverse of the above (p_chassis = g_R_chassis_imu * p_imu + g_t_chassis_imu),
// i.e. the pose of the IMU frame as seen from the chassis frame. Used as the
// odometry's genesis pose (x_curr.R/p at t=0) so that the whole world frame
// ("camera_init") is anchored at the vehicle's chassis-center start pose
// instead of the IMU's, keeping every downstream world-frame quantity
// (voxel map, saved pcd/keyframes, alidarState.txt, published clouds) in one
// consistent frame. Identity/zero — i.e. no change from prior behavior —
// when no chassis extrinsic is configured.
Eigen::Matrix3d g_R_chassis_imu = Eigen::Matrix3d::Identity();
Eigen::Vector3d g_t_chassis_imu = Eigen::Vector3d::Zero();
// Explicit registration only for a new map whose initial chassis pose is
// independently known. Loaded maps obtain their registration from matching.
std::optional<Eigen::Isometry3d> g_initial_map_chassis;

// Fixed lidar_back->lidar_front extrinsic (p_front = g_R_back_front * p_back +
// g_t_back_front), loaded in the VOXEL_SLAM constructor from
// General.back_extrinsic_{tran,rota} (only meaningful when General.lid_topic_back
// is set). pcl_handler_back() applies this to every point of each back-lidar
// scan so it can join the same pcl_buf/time_buf stream the front lidar feeds —
// downstream (feat extrinsic, EKF, BA/PGO) then sees a single lidar frame
// regardless of which physical sensor a given scan came from. Identity/zero
// (no-op) when the back lidar is not configured.
Eigen::Matrix3d g_R_back_front = Eigen::Matrix3d::Identity();
Eigen::Vector3d g_t_back_front = Eigen::Vector3d::Zero();

// Position of the chassis-frame origin, expressed in each lidar's own raw
// sensor frame — derived from the same chassis extrinsic as g_R/t_imu_chassis
// above (and, for the back lidar, composed with g_R/t_back_front). Handed to
// Features::blind_center (see feature_point.hpp) right before each lidar's
// process() call so the blind-zone check measures "distance from the chassis
// body" instead of "distance from wherever this particular lidar is
// mounted" — the only sane definition once there are two lidars at two
// different offsets from the chassis. Zero (no-op, falls back to the old
// lidar-centered blind check) when no chassis extrinsic is configured.
Eigen::Vector3d g_chassis_in_lidar_front = Eigen::Vector3d::Zero();
Eigen::Vector3d g_chassis_in_lidar_back = Eigen::Vector3d::Zero();
// Rotation taking a point already re-centered on g_chassis_in_lidar_front/
// back (i.e. p - blind_center) from that lidar's own raw axes into the
// chassis's own aligned axes — needed because the blind zone is now a
// rectangular BOX matching the chassis footprint (not a sphere/cylinder),
// so it only lines up with the real chassis body once expressed along the
// chassis's own forward/left axes, not whichever way a given lidar happens
// to be mounted. Handed to Features::blind_R alongside blind_center.
// Identity (no-op) when no chassis extrinsic is configured.
Eigen::Matrix3d g_chassis_R_in_lidar_front = Eigen::Matrix3d::Identity();
Eigen::Matrix3d g_chassis_R_in_lidar_back = Eigen::Matrix3d::Identity();

// lidar_front -> camera extrinsic (p_camera = g_R_cam_lidar * p_lidar +
// g_t_cam_lidar), loaded from General.image_extrinsic_{rota,tran}. Parsed
// and logged only for now: per product decision, image_poses.txt records
// the matched lidar frame's own IMU world pose as-is rather than a
// camera-frame pose, so this extrinsic isn't applied anywhere yet. Kept
// here so it's ready to wire in if a true camera-frame pose export is
// needed later.
Eigen::Matrix3d g_R_cam_lidar = Eigen::Matrix3d::Identity();
Eigen::Vector3d g_t_cam_lidar = Eigen::Vector3d::Zero();

// Max allowed |timestamp_image - timestamp_lidar_frame| (both hardware
// stamps) for a compressed image to be considered a match for a saved lidar
// frame (General.image_time_tolerance_ms). A lidar frame with no image
// within tolerance gets no image/pose line.
double g_image_time_tol_sec = 0.05;

// Buffer of not-yet-consumed images, kept as (own hardware timestamp,
// still-compressed bytes) — decoding is deferred to the one image actually
// picked for a given lidar frame, so this stays cheap to hold even at
// camera framerate. Guarded by m_image, which is never held at the same
// time as mBuf/m_pair. Trimmed by image_handler() to a rolling time
// horizon (IMAGE_BUF_HORIZON_SEC) plus a hard count cap, as a backstop
// against a runaway/unmatched buffer; find_and_consume_nearest_image()
// additionally erases every entry up to and including a match once one is
// picked, so a single image is never matched to more than one lidar frame.
mutex m_image;
deque<pair<double, vector<uint8_t>>> image_buf;
constexpr double IMAGE_BUF_HORIZON_SEC = 120.0;
constexpr size_t IMAGE_BUF_MAX_COUNT = 20000;

// Height ROI applied to /map_scan before republishing on /map_scan_filtered
// for navigation obstacle detection (General.nav_scan_z_min/max). Assumes
// flat ground, so world-frame z is used directly as height-above-ground —
// no per-frame chassis-height correction.
double g_nav_scan_z_min = 0.05;
double g_nav_scan_z_max = 1.63;

// Re-express an IMU-frame pose (world<-imu) as the equivalent chassis-frame pose
// (world<-chassis) using the fixed IMU->chassis extrinsic above.
inline void imu_pose_to_chassis(const Eigen::Matrix3d &R_wi, const Eigen::Vector3d &t_wi,
                                 Eigen::Matrix3d &R_wc, Eigen::Vector3d &t_wc)
{
  R_wc = R_wi * g_R_imu_chassis;
  t_wc = R_wi * g_t_imu_chassis + t_wi;
}

// Needs g_node and imu_pose_to_chassis(), both above — must be included
// after them, not up with the other includes at the top of this file.
#include "highrate_odom.hpp"

// Fills a Pose message directly from a world<-IMU pose (R,p) — no chassis
// conversion. This is deliberately NOT imu_pose_to_chassis()+chassis pose:
// Keyframe::plptr's points are stored relative to kf.x0's raw IMU pose (see
// e.g. pub_globalmap()'s `xx.R*vv + xx.p` with xx=kf.x0), so the pose sent
// alongside that cloud must use the exact same convention, or every submap
// gets displaced by the fixed IMU->chassis extrinsic rotated through that
// keyframe's own (time-varying) heading — a per-keyframe error that fans
// out worse the more the heading has turned, not a uniform offset.
inline void fill_pose(const Eigen::Matrix3d &R, const Eigen::Vector3d &p, geometry_msgs::msg::Pose &pose)
{
  Eigen::Quaterniond q(R);
  pose.position.x = p.x();
  pose.position.y = p.y();
  pose.position.z = p.z();
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
}

// Sends one keyframe's cloud+pose to nav_prob_grid (or any other Cartographer-
// submap-style consumer) exactly once, when that keyframe is finalized (kept,
// not discarded as a redundant near-duplicate — see the two call sites).
// The cloud is the keyframe's own LOCAL-frame points (Keyframe::plptr,
// relative to kf.x0), height-ROI-filtered using kf.x0 as it is right now —
// if kf.x0 is corrected later, this filtering isn't redone, only the pose
// gets republished (see publish_keyframe_pose_array below and
// astribot_slam_msgs/KeyframeSubmap.msg).
inline void publish_keyframe_submap(int session_id, Keyframe &kf,
                                     rclcpp::Publisher<astribot_slam_msgs::msg::KeyframeSubmap>::SharedPtr &pub)
{
  pcl::PointCloud<PointType> pl_local_filtered;
  pl_local_filtered.reserve(kf.plptr->size());
  for(PointType &lp: kf.plptr->points)
  {
    Eigen::Vector3d wld = kf.x0.R * Eigen::Vector3d(lp.x, lp.y, lp.z) + kf.x0.p;
    if(wld.z() >= g_nav_scan_z_min && wld.z() <= g_nav_scan_z_max)
      pl_local_filtered.push_back(lp);
  }

  astribot_slam_msgs::msg::KeyframeSubmap msg;
  msg.session_id = (uint32_t)session_id;
  msg.keyframe_id = (uint32_t)kf.id;
  fill_pose(kf.x0.R, kf.x0.p, msg.pose);
  pl_local_filtered.height = 1; pl_local_filtered.width = pl_local_filtered.size();
  pcl::toROSMsg(pl_local_filtered, msg.cloud);
  msg.cloud.header.stamp = g_node ? g_node->get_clock()->now() : rclcpp::Clock().now();
  pub->publish(msg);
}

// Re-broadcasts every currently-known keyframe's pose (no clouds — those
// were already sent once by publish_keyframe_submap). Called whenever
// voxel_slam's own BTC loop closure / GTSAM pose-graph optimization / HBA
// global BA corrects one or more keyframes' kf.x0, so downstream submap
// consumers can reposition their already-built local grids without
// rebuilding them. Simplicity over efficiency: resends the whole roster
// every time rather than tracking which keyframes actually moved.
//
// is_final/save_dir/map_name are only set (by topDownProcess(), the very
// last call to this function for a given run) once HBA's final top-down
// solve has corrected every keyframe above — i.e. every pose in this exact
// message is now final. save_dir/map_name are non-empty only when
// General.is_save_map && !General.mapname.empty(), and are a one-shot "your
// composited map is final now, save it under this path if you keep one" cue
// for a downstream consumer such as nav_prob_grid — voxel_slam itself
// doesn't build or save any grid; see astribot_slam_msgs/KeyframePoseArray.msg.
inline void publish_keyframe_pose_array(vector<vector<Keyframe*>*> &multimap_keyframes,
                                         rclcpp::Publisher<astribot_slam_msgs::msg::KeyframePoseArray>::SharedPtr &pub,
                                         bool is_final = false, const string &save_dir = "",
                                         const string &map_name = "")
{
  astribot_slam_msgs::msg::KeyframePoseArray arr;
  for(size_t session_id = 0; session_id < multimap_keyframes.size(); session_id++)
  {
    for(Keyframe *kf: *multimap_keyframes[session_id])
    {
      astribot_slam_msgs::msg::KeyframePoseUpdate upd;
      upd.session_id = (uint32_t)session_id;
      upd.keyframe_id = (uint32_t)kf->id;
      fill_pose(kf->x0.R, kf->x0.p, upd.pose);
      arr.updates.push_back(upd);
    }
  }
  arr.is_final = is_final;
  arr.save_dir = save_dir;
  arr.map_name = map_name;
  pub->publish(arr);
}

// Fixed 6-decimal (microsecond) formatting of a hardware timestamp, shared
// by every place that needs the timestamp as a filename/column and expects
// byte-identical results (pcd/png filenames, lidar_poses.txt/image_poses.txt
// timestamp column, and the previous_map_read fallback that reconstructs a
// pcd filename from alidarState.txt's own timestamp column).
inline string format_ts(double t)
{
  ostringstream oss;
  oss << fixed << setprecision(6) << t;
  return oss.str();
}

template <typename T>
void pub_pl_func(T &pl, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub, double source_time = -1.0)
{
  if(!g_world_registered && (pub == pub_scan || pub == pub_scan_filtered ||
      pub == pub_curr_path || pub == pub_cmap || pub == pub_init)) return;
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "map";
  output.header.stamp = g_node ? g_node->get_clock()->now() : rclcpp::Clock().now();
  if(source_time >= 0.0) output.header.stamp = rclcpp::Time(static_cast<int64_t>(source_time * 1e9));
  pub->publish(output);
}

mutex mBuf;
Features feat;
RobotSelfFilter robot_self_filter;
deque<sensor_msgs::msg::Imu::SharedPtr> imu_buf;
deque<pcl::PointCloud<PointType>::Ptr> pcl_buf;
deque<double> time_buf;
size_t lidar_pending_depth = 0; // zero preserves the existing unbounded hardware queue

double imu_last_time = -1;
int point_notime = 0;
double last_pcl_time = -1;

void imu_handler(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
  // Dual timestamp per guideline: sensor_ts is the driver-stamped capture time
  // (wall/ROS time, carried in the message header); recv_monotonic_ts is when
  // *this process* saw the message, on a monotonic clock so it stays usable for
  // latency/rate math even across wall-clock jumps (NTP step, sim-time resets).
  int64_t recv_mono = vxlm_log::now_monotonic_ns();
  double sensor_ts = stamp2sec(msg_in->header.stamp);

  static int flag = 1;
  if(flag)
  {
    flag = 0;
    LOG_STARTUP(IMU, "first IMU message | sensor_ts:{:.6f} recv_monotonic_ts:{}", sensor_ts, recv_mono);
  }

  // Steady-state quantized log (guideline §3): report rate once per window
  // instead of once per message, so IMU dropout/rate-change is visible without
  // flooding the log at ~200Hz.
  static uint32_t imu_msg_cnt = 0;
  static int64_t window_start_mono = recv_mono;
  if(++imu_msg_cnt >= 1000)
  {
    int64_t elapsed = recv_mono - window_start_mono;
    double rate_hz = elapsed > 0 ? imu_msg_cnt * 1e9 / double(elapsed) : 0.0;
    LOG_INFO(IMU, "IMU stream stats | sensor_ts:{:.6f} recv_monotonic_ts:{} rate:{:.1f}Hz",
             sensor_ts, recv_mono, rate_hz);

    // This window (~1000 msgs, ~5s at nominal 200Hz) is also this thread's
    // own periodic CPU-utilization sample point — see sample_thread_cpu()'s
    // comment. Piggybacked here rather than a separate timer since this
    // block already fires on a ~5s cadence on this exact thread.
    sample_thread_cpu(TSLOT_IMU, recv_mono / 1e6);

    // rate_hz < 180Hz flags one of the rare (observed ~12/11h in prior runs)
    // IMU delivery stalls this thread cannot explain by itself (it is its
    // own dedicated executor, see g_imu_cbg) — dump every thread's most
    // recent utilization sample so a post-hoc read of the log can tell
    // "some thread was pegged at ~100% CPU when this fired" apart from
    // "every thread was idle" (the latter would point upstream of this
    // process — driver/bus/transport — rather than at CPU contention here).
    if(rate_hz < 180.0)
    {
      LOG_INFO(IMU, "thread cpu dump (triggered by imu rate drop) | rate:{:.1f}Hz | {}:{:.1f}% {}:{:.1f}% {}:{:.1f}% {}:{:.1f}%",
               rate_hz,
               thread_slot_name(TSLOT_MAIN), g_thread_cpu[TSLOT_MAIN].util_pct.load(std::memory_order_relaxed),
               thread_slot_name(TSLOT_IMU),  g_thread_cpu[TSLOT_IMU].util_pct.load(std::memory_order_relaxed),
               thread_slot_name(TSLOT_LOOP), g_thread_cpu[TSLOT_LOOP].util_pct.load(std::memory_order_relaxed),
               thread_slot_name(TSLOT_GBA),  g_thread_cpu[TSLOT_GBA].util_pct.load(std::memory_order_relaxed));
    }

    imu_msg_cnt = 0;
    window_start_mono = recv_mono;
  }

  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

  // For Hilti 2022 exp03
  // double t0 = 1646320760 + 255.5;
  // double t1 = 1646320760 + 256.2;
  // double tc = stamp2sec(msg->header.stamp);
  // if(tc > t0 && tc < t1)
  //   msg->linear_acceleration.z = -9.7;

  mBuf.lock();
  imu_last_time = sensor_ts;
  imu_buf.push_back(msg);

  // Multi-sensor time-difference monitor (guideline §5/§8): the IMU and LiDAR
  // streams are captured by different drivers/clocks; if their timestamps drift
  // apart by more than a fraction of a second, sync_packages() will eventually
  // starve or stall silently, so flag the divergence here as soon as it's seen.
  if(!time_buf.empty())
  {
    double lidar_ts = time_buf.back();
    double diff_ms = (imu_last_time - lidar_ts) * 1000.0;
    if(fabs(diff_ms) > 500.0)
      LOG_WARN_THROTTLE(IMU, 200, "IMU/LiDAR timestamp gap large | imu_ts:{:.6f} lidar_ts:{:.6f} diff_ms:{:.1f}",
                         imu_last_time, lidar_ts, diff_ms);
  }
  mBuf.unlock();

  // Feeds the 20Hz tf output (see highrate_odom.hpp) — its own buffer,
  // independent of imu_buf above, so it doesn't interact with sync_packages().
  HighRateOdom::instance().on_imu(msg);
}

// General.image_topic subscription callback. Only wired up (in the
// VOXEL_SLAM constructor) when that topic is non-empty. Stores the frame's
// own hardware timestamp plus its still-compressed bytes in image_buf;
// decoding is deferred to whichever single frame actually gets matched to a
// saved lidar frame (see find_and_consume_nearest_image()).
void image_handler(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg_in)
{
  double sensor_ts = stamp2sec(msg_in->header.stamp);

  static int flag = 1;
  if(flag)
  {
    flag = 0;
    LOG_STARTUP(CAMERA, "first compressed image | sensor_ts:{:.6f} bytes:{}", sensor_ts, msg_in->data.size());
  }

  static uint32_t img_msg_cnt = 0;
  static int64_t window_start_mono = vxlm_log::now_monotonic_ns();
  if(++img_msg_cnt >= 100)
  {
    int64_t now_mono = vxlm_log::now_monotonic_ns();
    int64_t elapsed = now_mono - window_start_mono;
    double rate_hz = elapsed > 0 ? img_msg_cnt * 1e9 / double(elapsed) : 0.0;
    LOG_INFO(CAMERA, "image stream stats | sensor_ts:{:.6f} rate:{:.1f}Hz", sensor_ts, rate_hz);
    img_msg_cnt = 0;
    window_start_mono = now_mono;
  }

  lock_guard<mutex> lk(m_image);
  image_buf.emplace_back(sensor_ts, msg_in->data);

  // Rolling time-horizon trim (backstop against an ever-growing buffer when
  // no lidar frame is consuming matches, e.g. is_save_map off or lidar
  // stalled) plus a hard count cap for the same reason.
  while(!image_buf.empty() && sensor_ts - image_buf.front().first > IMAGE_BUF_HORIZON_SEC)
    image_buf.pop_front();
  while(image_buf.size() > IMAGE_BUF_MAX_COUNT)
    image_buf.pop_front();
}

// Finds the image in image_buf whose own hardware timestamp is closest to
// target_t (a saved lidar frame's hardware timestamp). On a match within
// tol_sec, copies its timestamp/bytes out and erases it plus every older
// entry (so a single image is never matched to more than one lidar frame,
// and the buffer doesn't grow unbounded); returns false, leaving the buffer
// untouched, if nothing is within tolerance yet.
bool find_and_consume_nearest_image(double target_t, double tol_sec, double &out_ts, vector<uint8_t> &out_data)
{
  lock_guard<mutex> lk(m_image);
  if(image_buf.empty()) return false;

  size_t best_idx = 0;
  double best_diff = fabs(image_buf[0].first - target_t);
  for(size_t i=1; i<image_buf.size(); i++)
  {
    double diff = fabs(image_buf[i].first - target_t);
    if(diff < best_diff)
    {
      best_diff = diff;
      best_idx = i;
    }
  }

  if(best_diff > tol_sec) return false;

  out_ts = image_buf[best_idx].first;
  out_data = image_buf[best_idx].second;
  for(size_t i=0; i<=best_idx; i++)
    image_buf.pop_front();
  return true;
}

// Set true in the VOXEL_SLAM constructor iff General.lid_topic_back is
// configured. Front-only deployments (the default) never touch the pairing
// machinery below — pcl_handler() falls back to the exact pre-dual-lidar
// behavior (build scan, push straight to pcl_buf/time_buf) when this is false.
bool g_dual_lidar_enabled = false;

// Hardware-sync tolerance between the front and back lidar's own header
// timestamps (General.back_sync_tolerance_ms, converted to seconds). The two
// sensors are assumed to be hardware-triggered together, so their header
// stamps should differ only by driver/network jitter; a front/back pair
// whose stamps differ by more than this is treated as unmatched and the
// stale scan is dropped rather than merged (see try_pair_and_push()).
double g_back_sync_tol_sec = 0.05;

// One-slot mailboxes used to pair up front/back scans that arrive on two
// independent subscription callbacks. Each holds at most the most recent
// not-yet-matched scan from that lidar; a second scan from the same lidar
// arriving before a match is found evicts (drops) the first. Guarded by
// m_pair, which is never held at the same time as mBuf (try_pair_and_push()
// releases it before the finalize step takes mBuf).
mutex m_pair;
pcl::PointCloud<PointType>::Ptr pending_front_pl, pending_back_pl;
double pending_front_t0 = -1, pending_back_t0 = -1;
uint32_t merge_ok_cnt = 0, drop_front_cnt = 0, drop_back_cnt = 0;

// Per-scan cleanup shared by the front/back/merged paths: fills in two dummy
// points if the scan came back empty (keeps downstream code, which assumes
// at least one point, from choking), sorts by curvature (per-point time
// offset from t0) ascending, and — only when the caller says this scan's
// curvature values are still on their ORIGINAL single-scan basis (i.e. not
// yet rebased by a cross-lidar time offset) — drops any tail points whose
// offset exceeds ~110ms as garbage/out-of-range timestamps. A merged
// front+back cloud must NOT re-run that last step: after rebasing, a valid
// back-lidar point can legitimately carry curvature beyond 0.11 (its own
// original offset plus the front/back header-time skew), and re-applying the
// 0.11 cutoff would wrongly truncate it.
void finalize_scan(pcl::PointCloud<PointType>::Ptr &pl_ptr, bool trim_by_own_duration)
{
  if(pl_ptr->empty())
  {
    PointType ap;
    ap.x = 0; ap.y = 0; ap.z = 0;
    ap.intensity = 0; ap.curvature = 0;
    pl_ptr->push_back(ap);
    ap.curvature = 0.09;
    pl_ptr->push_back(ap);
  }

  sort(pl_ptr->begin(), pl_ptr->end(), [](PointType &x, PointType &y)
  {
    return x.curvature < y.curvature;
  });

  if(trim_by_own_duration)
    while(pl_ptr->back().curvature > 0.11)
      pl_ptr->points.pop_back();
}

void push_scan(pcl::PointCloud<PointType>::Ptr &pl_ptr, double t0)
{
  mBuf.lock();
  time_buf.push_back(t0);
  pcl_buf.push_back(pl_ptr);
  // Opt-in bounded simulation queue after dual-LiDAR pairing.
  // Preserve sensor timestamps and IMU history; never relabel old scans as new.
  while(lidar_pending_depth && pcl_buf.size() > lidar_pending_depth) {
    pcl_buf.pop_front(); time_buf.pop_front();
  }
  mBuf.unlock();
}

// Pairs a just-finalized (dummy-filled/sorted/own-duration-trimmed) front or
// back scan with the most recent not-yet-matched scan from the other lidar,
// per General.back_sync_tolerance_ms (see g_back_sync_tol_sec above). On a
// match: the back scan's curvature (still relative to *its own* header time)
// is rebased onto the front scan's header time —
//   curvature_new = (t0_back - t0_front) + curvature_old
// — i.e. each point's absolute sensor time t0_back+curvature_old, minus the
// front scan's t0, so both clouds' points are on one common time axis before
// they're concatenated, re-sorted, and pushed as a single merged scan whose
// pcl_beg_time is the front scan's t0 (matching every other frame in
// pcl_buf, which is anchored to the front lidar's header time). Unmatched
// scans — no counterpart within tolerance, or evicted by a newer scan from
// their own side before a match arrived — are dropped, never pushed alone;
// see the design note in the pending_front_pl/pending_back_pl comment above.
void try_pair_and_push(pcl::PointCloud<PointType>::Ptr &pl, double t0, bool is_front)
{
  lock_guard<mutex> lk(m_pair);

  auto &pend_self_pl  = is_front ? pending_front_pl : pending_back_pl;
  auto &pend_self_t0  = is_front ? pending_front_t0 : pending_back_t0;
  auto &pend_other_pl = is_front ? pending_back_pl  : pending_front_pl;
  auto &pend_other_t0 = is_front ? pending_back_t0  : pending_front_t0;

  if(pend_other_pl)
  {
    double diff = t0 - pend_other_t0; // (back t0 - front t0), whichever side "t0"/"pend_other_t0" actually is
    if(fabs(diff) <= g_back_sync_tol_sec)
    {
      double front_t0 = is_front ? t0 : pend_other_t0;
      pcl::PointCloud<PointType>::Ptr front_pl = is_front ? pl : pend_other_pl;
      pcl::PointCloud<PointType>::Ptr back_pl  = is_front ? pend_other_pl : pl;
      double back_t0 = is_front ? pend_other_t0 : t0;
      double rebase = back_t0 - front_t0;

      for(PointType &ap : back_pl->points)
        ap.curvature += rebase;
      *front_pl += *back_pl;
      // Re-sort: front_pl/back_pl were each already curvature-sorted on their
      // own, but interleaving the two runs into one scan needs a fresh sort.
      sort(front_pl->begin(), front_pl->end(), [](PointType &x, PointType &y)
      {
        return x.curvature < y.curvature;
      });

      pend_other_pl.reset();
      pend_other_t0 = -1;
      merge_ok_cnt++;
      // Keep the steady-state counter throttled in the shared ROS/session log.
      LOG_INFO_THROTTLE(LIDAR, 50, "front+back lidar merged | front_ts:{:.6f} back_ts:{:.6f} rebase_ms:{:.1f} "
                         "merged:{} drop_front:{} drop_back:{}",
                         front_t0, back_t0, rebase * 1000.0, merge_ok_cnt, drop_front_cnt, drop_back_cnt);
      push_scan(front_pl, front_t0);
      return;
    }

    // Other side's pending scan is stale/mismatched — drop it (strict
    // pairing: only exact-tolerance matches are merged, nothing is pushed
    // unpaired) and fall through to stash the current scan below.
    if(is_front) { drop_back_cnt++;  LOG_WARN_THROTTLE(LIDAR, 20, "dropped unmatched back scan | back_ts:{:.6f} vs new front_ts:{:.6f} diff_ms:{:.1f} (tol {:.1f}ms)", pend_other_t0, t0, diff * 1000.0, g_back_sync_tol_sec * 1000.0); }
    else         { drop_front_cnt++; LOG_WARN_THROTTLE(LIDAR, 20, "dropped unmatched front scan | front_ts:{:.6f} vs new back_ts:{:.6f} diff_ms:{:.1f} (tol {:.1f}ms)", pend_other_t0, t0, diff * 1000.0, g_back_sync_tol_sec * 1000.0); }
    pend_other_pl.reset();
    pend_other_t0 = -1;
  }

  if(pend_self_pl)
  {
    // A second scan from the same lidar arrived before the first found a
    // match — the first is now unrecoverable (strict pairing), drop it.
    if(is_front) { drop_front_cnt++; LOG_WARN_THROTTLE(LIDAR, 20, "dropped unmatched front scan (superseded) | front_ts:{:.6f}", pend_self_t0); }
    else         { drop_back_cnt++;  LOG_WARN_THROTTLE(LIDAR, 20, "dropped unmatched back scan (superseded) | back_ts:{:.6f}", pend_self_t0); }
  }
  pend_self_pl = pl;
  pend_self_t0 = t0;
}

template<class T>
void pcl_handler(T &msg)
{
  int64_t recv_mono = vxlm_log::now_monotonic_ns();

  pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
  if(!robot_self_filter.update(msg->header.stamp)) return;
  feat.blind_center = g_chassis_in_lidar_front;
  feat.blind_R = g_chassis_R_in_lidar_front;
  double t0 = feat.process(msg, *pl_ptr);

  static int flag = 1;
  if(flag)
  {
    flag = 0;
    LOG_STARTUP(LIDAR, "first lidar scan | sensor_ts:{:.6f} recv_monotonic_ts:{}", t0, recv_mono);
  }

  static uint32_t scan_cnt = 0;
  static int64_t window_start_mono = recv_mono;
  if(++scan_cnt >= 50)
  {
    int64_t elapsed = recv_mono - window_start_mono;
    double rate_hz = elapsed > 0 ? scan_cnt * 1e9 / double(elapsed) : 0.0;
    LOG_INFO(LIDAR, "lidar scan stream stats | sensor_ts:{:.6f} recv_monotonic_ts:{} rate:{:.1f}Hz",
             t0, recv_mono, rate_hz);
    scan_cnt = 0;
    window_start_mono = recv_mono;
  }

  // Own-basis cleanup first (dummy-fill/sort/0.11-duration trim on this
  // scan's own curvature, i.e. offset from its own header time) — identical
  // regardless of whether this scan ends up merged with a back scan or
  // pushed on its own.
  finalize_scan(pl_ptr, /*trim_by_own_duration=*/true);

  if(g_dual_lidar_enabled)
    try_pair_and_push(pl_ptr, t0, /*is_front=*/true);
  else
    push_scan(pl_ptr, t0);
}

// Handles the optional second lidar ("lidar_back", General.lid_topic_back).
// Every point is first re-expressed from the back lidar's own frame into the
// front lidar's frame (via g_R_back_front/g_t_back_front); the scan is then
// handed to try_pair_and_push(), which rebases its (still own-basis)
// curvature onto whichever front scan it pairs with and merges the two
// clouds into a single scan — see try_pair_and_push()'s comment for the time
// math. Kept as its own function (rather than a parameter on pcl_handler) so
// its rate/first-scan logging counters are tracked separately from the front
// lidar's, and so the hot front-only path (g_dual_lidar_enabled == false)
// pays no extra transform/pairing cost.
template<class T>
void pcl_handler_back(T &msg)
{
  int64_t recv_mono = vxlm_log::now_monotonic_ns();

  pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
  if(!robot_self_filter.update(msg->header.stamp)) return;
  feat.blind_center = g_chassis_in_lidar_back;
  feat.blind_R = g_chassis_R_in_lidar_back;
  double t0 = feat.process(msg, *pl_ptr);

  for(PointType &ap : pl_ptr->points)
  {
    Eigen::Vector3d p_back(ap.x, ap.y, ap.z);
    Eigen::Vector3d p_front = g_R_back_front * p_back + g_t_back_front;
    ap.x = p_front(0); ap.y = p_front(1); ap.z = p_front(2);
  }

  static int flag = 1;
  if(flag)
  {
    flag = 0;
    LOG_STARTUP(LIDAR, "first lidar scan (back) | sensor_ts:{:.6f} recv_monotonic_ts:{}", t0, recv_mono);
  }

  static uint32_t scan_cnt = 0;
  static int64_t window_start_mono = recv_mono;
  if(++scan_cnt >= 50)
  {
    int64_t elapsed = recv_mono - window_start_mono;
    double rate_hz = elapsed > 0 ? scan_cnt * 1e9 / double(elapsed) : 0.0;
    LOG_INFO(LIDAR, "lidar scan stream stats (back) | sensor_ts:{:.6f} recv_monotonic_ts:{} rate:{:.1f}Hz",
             t0, recv_mono, rate_hz);
    scan_cnt = 0;
    window_start_mono = recv_mono;
  }

  // Own-basis cleanup (curvature still relative to this scan's own header
  // time, t0) — the cross-lidar rebase onto the matched front scan's time
  // happens inside try_pair_and_push(), after this trim, so it can't clip
  // valid points that only look out-of-range once rebased.
  finalize_scan(pl_ptr, /*trim_by_own_duration=*/true);

  try_pair_and_push(pl_ptr, t0, /*is_front=*/false);
}


bool sync_packages(pcl::PointCloud<PointType>::Ptr &pl_ptr, deque<sensor_msgs::msg::Imu::SharedPtr> &imus, IMUEKF &p_imu)
{
  static bool pl_ready = false;

  if(!pl_ready)
  {
    if(pcl_buf.empty()) return false;

    mBuf.lock();
    pl_ptr = pcl_buf.front();
    p_imu.pcl_beg_time = time_buf.front();
    pcl_buf.pop_front(); time_buf.pop_front();
    mBuf.unlock();

    p_imu.pcl_end_time = p_imu.pcl_beg_time + pl_ptr->back().curvature;

    if(point_notime)
    {
      if(last_pcl_time < 0)
      {
        last_pcl_time = p_imu.pcl_beg_time;
        return false;
      }

      p_imu.pcl_end_time = p_imu.pcl_beg_time;
      p_imu.pcl_beg_time = last_pcl_time;
      last_pcl_time = p_imu.pcl_end_time;
    }

    pl_ready = true;
  }

  // imu_last_time is written under mBuf by imu_handler(), which — since
  // sub_imu moved to its own dedicated executor/thread (see g_imu_cbg) —
  // now genuinely runs concurrently with this thread, not just
  // interleaved via spin_some(n) like before. Snapshot it under the same
  // lock rather than reading the bare global.
  mBuf.lock();
  bool imu_caught_up = imu_last_time > p_imu.pcl_end_time;
  mBuf.unlock();
  if(!pl_ready || !imu_caught_up) return false;

  mBuf.lock();
  double imu_time = stamp2sec(imu_buf.front()->header.stamp);
  while((!imu_buf.empty()) && (imu_time < p_imu.pcl_end_time))
  {
    imu_time = stamp2sec(imu_buf.front()->header.stamp);
    if(imu_time > p_imu.pcl_end_time) break;
    imus.push_back(imu_buf.front());
    imu_buf.pop_front();
  }
  mBuf.unlock();

  if(imu_buf.empty())
  {
    LOG_ERROR(IMU, "imu buffer exhausted while syncing to lidar scan | pcl_end_time:{:.6f} — check IMU driver/rate", p_imu.pcl_end_time);
    exit(0);
  }

  pl_ready = false;

  if(imus.size() > 4)
    return true;
  else
    return false;
}

double dept_err, beam_err;
void calcBodyVar(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &var) 
{
  if (pb[2] == 0)
    pb[2] = 0.0001;
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  float range_var = range_inc * range_inc;
  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);
  Eigen::Vector3d direction(pb);
  direction.normalize();
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  var = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
};

// Compute the variance of the each point
void var_init(IMUST &ext, pcl::PointCloud<PointType> &pl_cur, PVecPtr pptr, double dept_err, double beam_err)
{
  int plsize = pl_cur.size();
  pptr->clear();
  pptr->resize(plsize);
  for(int i=0; i<plsize; i++)
  {
    PointType &ap = pl_cur[i];
    pointVar &pv = pptr->at(i);
    pv.pnt << ap.x, ap.y, ap.z;
    calcBodyVar(pv.pnt, dept_err, beam_err, pv.var);
    pv.pnt = ext.R * pv.pnt + ext.p;
    pv.var = ext.R * pv.var * ext.R.transpose();
  }
}

void pvec_update(PVecPtr pptr, IMUST &x_curr, PLV(3) &pwld)
{
  Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);
  Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);

  for(pointVar &pv: *pptr)
  {
    Eigen::Matrix3d phat = hat(pv.pnt);
    pv.var = x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
    pwld.push_back(x_curr.R * pv.pnt + x_curr.p);
  }
}

// Read the alidarstate.txt
void read_lidarstate(string filename, vector<ScanPose*> &bl_tem)
{
  ifstream file(filename);
  if(!file.is_open())
  {
    LOG_ERROR(INIT, "file not found | path:{}", filename);
    exit(0);
  }

  string lineStr, str;
  vector<double> nums;
  while(getline(file, lineStr))
  {
    nums.clear();
    stringstream ss(lineStr);
    while(getline(ss, str, ' '))
      nums.push_back(stod(str));
    
    IMUST xx;
    xx.t = nums[0];
    xx.p << nums[1], nums[2], nums[3];
    xx.R = Eigen::Quaterniond(nums[7], nums[4], nums[5], nums[6]).matrix();

    if(nums.size() >= 20)
    {
      xx.v << nums[8], nums[9], nums[10];
      xx.bg << nums[11], nums[12], nums[13];
      xx.ba << nums[14], nums[15], nums[16];
      xx.g << nums[17], nums[18], nums[19];
    }

    ScanPose* blp = new ScanPose(xx, nullptr);
    bl_tem.push_back(blp);

    if(nums.size() >= 26)
      for(int i=0; i<6; i++) 
        blp->v6[i] = nums[i + 20];
  }
}

double get_memory()
{
  ifstream infile("/proc/self/status");
  double mem = -1;
  string lineStr, str;
  while(getline(infile, lineStr))
  {
    stringstream ss(lineStr);
    bool is_find = false;
    while(ss >> str)
    {
      if(str == "VmRSS:")
      {
        is_find = true; continue;
      }

      if(is_find) mem = stod(str);
      break;
    }
    if(is_find) break;
  }
  return mem / (1048576);
}

void icp_check(pcl::PointCloud<PointType> &pl_src, pcl::PointCloud<PointType> &pl_tar, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub_src, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub_tar, pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform, IMUST &xx)
{
  pcl::PointCloud<PointType> pl1, pl2;
  for(PointType ap: pl_src.points)
  {
    Eigen::Vector3d v(ap.x, ap.y, ap.z);
    v = loop_transform.second * v + loop_transform.first;
    v = xx.R * v + xx.p;
    ap.x = v[0]; ap.y = v[1]; ap.z = v[2];
    pl1.push_back(ap);
  }
  for(PointType ap: pl_tar.points)
  {
    Eigen::Vector3d v(ap.x, ap.y, ap.z);
    v = xx.R * v + xx.p;
    ap.x = v[0]; ap.y = v[1]; ap.z = v[2];
    pl2.push_back(ap);
  }
  pub_pl_func(pl1, pub_src); pub_pl_func(pl2, pub_tar);
}
