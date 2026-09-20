// nav_prob_grid_node — global occupancy grid for navigation, split into two
// layers with two different jobs, active in different circumstances:
//
//   1. Historical layer (voxel_slam keyframes) — active always, and it's
//      the ONLY thing removing dynamic obstacles while the robot is
//      MOVING. Each keyframe is one frozen local submap, built once from
//      its own local-frame cloud (astribot_slam_msgs/KeyframeSubmap) and
//      never touched again — hit/miss ray-cast from the submap's own
//      local origin (0,0), same priority rule as Cartographer: every hit
//      in a frame is applied before that frame's misses, so one ray
//      grazing past a cell can't undo another ray's genuine hit in the
//      same frame. Whenever voxel_slam's own BTC loop closure / GTSAM
//      pose-graph optimization / HBA global BA corrects one or more
//      keyframes' poses, it resends just id+pose (astribot_slam_msgs/
//      KeyframePoseArray); this node only updates that submap's stored
//      placement, so a correction moves its whole frozen grid rigidly at
//      the next publish, without re-inserting any points. Composited in
//      publish() by cross-submap score summation + dwell-clustering (see
//      there) — a stale hit from a since-departed dynamic obstacle, seen
//      by only one submap, gets pulled back down once later submaps'
//      rays see through it; correlated nearby-viewpoint submaps (dwelling
//      near one spot) are folded into a single vote first, so loitering
//      doesn't erode a genuinely static cell just from vote count.
//
//   2. Live layer (voxel_slam's raw per-scan stream) — active ONLY while
//      the robot is genuinely STATIONARY (is_moving_ false, decided
//      straight off live TF velocity in onScan(), nothing to do with
//      keyframe cadence). A hit anywhere in a scan marks its cell occupied
//      immediately (one scan is enough — safety-first) and overrides the
//      historical layer's composite outright; a miss only clears a cell
//      after live_clear_min_duration_sec_ of REAL elapsed time with no
//      intervening hit (general caution against transient/sparse-coverage
//      noise, now that this layer only runs while stationary — it does
//      NOT help the one specific failure mode already found and accepted
//      below, since that one repeats identically forever, not just
//      briefly). Counting scan callbacks instead of elapsed time was
//      tried first and rejected: a single scan's own point yield is
//      sparse, and MID360's non-repetitive pattern only fills those gaps
//      in given enough integration TIME, not just enough callback firings
//      — a handful of scans close together in time can share the same
//      coverage gap and would have thinned or broken a genuinely solid,
//      thick wall one cell at a time. No vote-counting either way, no
//      "how many keyframes agree" — this is about one cell's own recent
//      scan history, nothing else. A hit anywhere in a scan also blocks
//      every OTHER ray in that same scan outright the moment its path
//      reaches that cell — not just protecting the hit cell itself
//      (same-frame hit-before-miss priority, like the historical layer
//      above), but treating it as opaque, so a farther-reaching beam that
//      passed a gap the obstruction doesn't cover (different bearing or
//      height) can't be mistaken for having seen through the obstruction
//      to whatever's behind it (confirmed by testing: without this, a
//      wall a scan visibly still hit could still have real structure
//      behind it erased by a different beam in that same scan). This node
//      subscribes to /map_scan_filtered (raw_scan_topic — already
//      height-ROI-filtered, published in world_frame coordinates every
//      LIO scan, regardless of whether voxel_slam is currently producing
//      keyframes) plus the continuous world_frame->chassis_frame TF
//      voxel_slam broadcasts at the same per-scan cadence (pub_odom_func()
//      in voxelslam.cpp) — used both as each scan's ray-casting origin
//      AND, differenced scan to scan, as the velocity estimate that
//      decides is_moving_. The moment that estimate flips back to moving,
//      whatever this layer had accumulated is promoted into a permanent
//      historical-layer submap (see takeSnapshot()) rather than just
//      stopping being consulted — confirmed by testing: while stationary,
//      hundreds of scans build up far denser coverage than any single
//      keyframe's own sparse cloud ever could, and without this the map
//      would visibly thin back down to that single-keyframe density the
//      instant motion resumed, even though nothing in the real world got
//      any sparser.
//
// Why split by motion at all: an earlier version made the live layer
// active unconditionally, always overriding the historical layer — clean
// and simple, and it does fix dynamic obstacles the historical layer alone
// can't (once independently confirmed occupied by more than one real
// keyframe — trivially possible after a bit of back-and-forth movement;
// confirmed by testing: 5 real keyframes from one such session still split
// into 2 separate dwell-clusters despite every mitigation tried — no
// single vote could ever outweigh that backlog by summing alone). But
// running it unconditionally means every single scan's own geometric
// quirks (a beam that clears an obstacle at one height and reaches much
// farther can look, once flattened to this grid's 2D (x,y) cells, like it
// passed straight through that obstacle — confirmed by testing, a real
// static wall with open space above it) get to instantly overwrite
// anything, with no accumulated evidence backing either side up. Gating
// it to stationary periods only keeps that instant-override behavior
// where it's needed (nothing else can clear a dynamic obstacle while the
// robot isn't producing new keyframes) while falling back to the more
// conservative, evidence-accumulating historical layer whenever the robot
// is actually moving through new keyframes anyway.
//
// Published output follows Cartographer's own occupancy-grid convention:
// int8[] data, -1 unknown (neither layer has ever touched that world
// cell), 0 free, 100 occupied — except for small unknown "holes" fully
// ringed by free space, which publish() fills to free (see
// unknown_fill_radius_/unknown_fill_min_free_frac_ below). Root cause,
// confirmed against voxelslam.cpp: a keyframe's cloud is voxel_slam's own
// merge of win_size (10) raw scans, but that merge is then voxel-grid
// downsampled to at most one point per voxel via down_sampling_pvec(...,
// voxel_size/10, ...) — with Odometry.voxel_size: 0.5 in mid360.yaml,
// that's a 0.05m voxel, exactly this node's own resolution_. MID360's real
// point yield over that merge window still leaves some of those 0.05m
// cells with zero points, no matter how many raw scans get merged
// upstream — a genuine sensor/pipeline density limit at exactly our
// grid's resolution, not new information, so it's corrected here rather
// than left to look like unexplored terrain.
//
// enable (this package's own yaml param) is the on/off switch: when false,
// the node does no work and never publishes, so downstream nav simply won't
// see anything on output_topic. Deciding which topic nav actually consumes
// is a launch/remap concern outside this node.
//
// One-shot final save: onPoseArray() also carries voxel_slam's own
// end-of-session cue (astribot_slam_msgs/KeyframePoseArray's is_final/save_dir/
// map_name — sent exactly once, right after voxel_slam's own HBA finishes
// correcting every keyframe's pose, whenever General.is_save_map &&
// General.mapname is non-empty over there). On that cue this node saves its
// own current composite grid (composite_grid() — the exact same historical-
// layer + live-layer compositing publish() uses every tick) to
// <save_dir><map_name>.pgm + .yaml, a standard ROS map_server/Nav2
// "trinary" map pair — see save_grid_to_disk()/write_pgm()/write_yaml()
// below. Nothing else here changes: this is purely "also write the map
// nav_prob_grid was already showing to disk, once, when told to".

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <astribot_slam_msgs/msg/keyframe_submap.hpp>
#include <astribot_slam_msgs/msg/keyframe_pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <limits>
#include <fstream>
#include <string>

// One keyframe's frozen local grid plus its (possibly since-corrected)
// world placement.
struct Submap
{
  double x = 0.0, y = 0.0;        // world position of this submap's local origin
  double cos_yaw = 1.0, sin_yaw = 0.0; // yaw-only rotation (matches the flat-ground
                                        // assumption already used elsewhere in this
                                        // system) from local frame into world frame
  // Local (col,row) -> accumulated score, built once from this keyframe's
  // own cloud and never touched again. Absence = never touched by a ray.
  std::unordered_map<int64_t, int8_t> cells;
};

class NavProbGridNode : public rclcpp::Node
{
public:
  NavProbGridNode() : Node("nav_prob_grid_node")
  {
    declare_parameter<bool>("enable", true);
    declare_parameter<std::string>("submap_topic", "/voxel_slam/keyframe_submap");
    declare_parameter<std::string>("pose_array_topic", "/voxel_slam/keyframe_pose_array");
    declare_parameter<std::string>("raw_scan_topic", "/map_scan_filtered");
    declare_parameter<std::string>("chassis_frame", "aft_mapped");
    declare_parameter<std::string>("output_topic", "/map");
    declare_parameter<std::string>("world_frame", "map");
    declare_parameter<double>("resolution", 0.05);
    body_half_extent_ = declare_parameter<double>("current_body_half_extent_m", 0.31);
    source_max_age_ = declare_parameter<double>("source_max_age_sec", 0.5);
    if(!std::isfinite(body_half_extent_) || body_half_extent_ < 0 ||
       !std::isfinite(source_max_age_) || source_max_age_ <= 0)
      throw std::invalid_argument("Invalid body-clearance/freshness parameters");
    declare_parameter<double>("publish_rate_hz", 1.0);
    declare_parameter<int>("hit_delta", 6);
    declare_parameter<int>("miss_delta", 1);
    declare_parameter<int>("score_min", -100);
    declare_parameter<int>("score_max", 100);
    declare_parameter<int>("occ_threshold", 0);
    declare_parameter<int>("unknown_fill_radius", 1);
    declare_parameter<double>("unknown_fill_min_free_frac", 0.6);
    declare_parameter<double>("dwell_cluster_radius", 0.3);
    declare_parameter<double>("dwell_cluster_angle_deg", 15.0);
    declare_parameter<double>("stationary_lin_vel_thresh", 0.02);
    declare_parameter<double>("stationary_ang_vel_thresh", 0.05);
    declare_parameter<double>("live_clear_min_duration_sec", 0.5);
    // Plain map_server/Nav2 yaml metadata for the one-shot final save (see
    // save_grid_to_disk()/write_yaml() below) — not consulted by any of the
    // grid-building params above.
    declare_parameter<int>("negate", 0);
    declare_parameter<double>("occupied_thresh", 0.65);
    declare_parameter<double>("free_thresh", 0.25);

    enable_ = get_parameter("enable").as_bool();
    submap_topic_ = get_parameter("submap_topic").as_string();
    pose_array_topic_ = get_parameter("pose_array_topic").as_string();
    raw_scan_topic_ = get_parameter("raw_scan_topic").as_string();
    chassis_frame_ = get_parameter("chassis_frame").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    world_frame_ = get_parameter("world_frame").as_string();
    resolution_ = get_parameter("resolution").as_double();
    double publish_rate_hz = get_parameter("publish_rate_hz").as_double();
    hit_delta_ = get_parameter("hit_delta").as_int();
    miss_delta_ = get_parameter("miss_delta").as_int();
    score_min_ = get_parameter("score_min").as_int();
    score_max_ = get_parameter("score_max").as_int();
    occ_threshold_ = get_parameter("occ_threshold").as_int();
    unknown_fill_radius_ = get_parameter("unknown_fill_radius").as_int();
    unknown_fill_min_free_frac_ = get_parameter("unknown_fill_min_free_frac").as_double();
    dwell_cluster_radius_ = get_parameter("dwell_cluster_radius").as_double();
    dwell_cluster_angle_ = get_parameter("dwell_cluster_angle_deg").as_double() * M_PI / 180.0;
    stationary_lin_vel_thresh_ = get_parameter("stationary_lin_vel_thresh").as_double();
    stationary_ang_vel_thresh_ = get_parameter("stationary_ang_vel_thresh").as_double();
    live_clear_min_duration_sec_ = get_parameter("live_clear_min_duration_sec").as_double();
    negate_ = get_parameter("negate").as_int();
    occupied_thresh_ = get_parameter("occupied_thresh").as_double();
    free_thresh_ = get_parameter("free_thresh").as_double();

    if(!enable_)
    {
      RCLCPP_WARN(get_logger(), "enable=false — node is idle, publishing nothing on %s",
                  output_topic_.c_str());
      return;
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Nav2's static layer subscribes with transient-local durability. Keep the
    // latest grid latched so a lifecycle restart does not wait for a new scan.
    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    submap_sub_ = create_subscription<astribot_slam_msgs::msg::KeyframeSubmap>(
      submap_topic_, 200,
      [this](astribot_slam_msgs::msg::KeyframeSubmap::ConstSharedPtr msg){ onSubmap(msg); });
    pose_sub_ = create_subscription<astribot_slam_msgs::msg::KeyframePoseArray>(
      pose_array_topic_, 10,
      [this](astribot_slam_msgs::msg::KeyframePoseArray::ConstSharedPtr msg){ onPoseArray(msg); });
    // Live layer (see module comment) — runs on every scan, always,
    // regardless of keyframe activity.
    raw_scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      raw_scan_topic_, 50,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){ onScan(msg); });

    double period_sec = (publish_rate_hz > 0.0) ? (1.0 / publish_rate_hz) : 1.0;
    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(period_sec), [this](){ publish(); });

    RCLCPP_INFO(get_logger(),
      "nav_prob_grid running | submaps:%s poses:%s live:%s(%s->%s) out:%s@%.2fHz res:%.2fm",
      submap_topic_.c_str(), pose_array_topic_.c_str(), raw_scan_topic_.c_str(),
      world_frame_.c_str(), chassis_frame_.c_str(), output_topic_.c_str(),
      publish_rate_hz, resolution_);
  }

private:
  static inline int64_t packCellKey(int col, int row)
  {
    return ((int64_t)(uint32_t)col << 32) | (uint32_t)row;
  }
  static inline void unpackCellKey(int64_t key, int &col, int &row)
  {
    col = (int)(uint32_t)(key >> 32);
    row = (int)(uint32_t)(key & 0xffffffffLL);
  }
  static inline uint64_t packSubmapKey(uint32_t session_id, uint32_t keyframe_id)
  {
    return ((uint64_t)session_id << 32) | keyframe_id;
  }
  static inline double yawFromQuat(const geometry_msgs::msg::Quaternion &q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  inline void bumpLocal(std::unordered_map<int64_t, int8_t> &cells, int col, int row, int delta)
  {
    int64_t key = packCellKey(col, row);
    auto it = cells.find(key);
    int base = (it == cells.end()) ? 0 : (int)it->second;
    cells[key] = (int8_t)std::clamp(base + delta, score_min_, score_max_);
  }

  // Bresenham from this submap's own local origin (0,0) to a local hit cell,
  // hit-only pass: marks just the endpoint. Called for every point BEFORE
  // any of this frame's miss passes run, so every cell this frame actually
  // hit is known up front.
  void raycastLocalHit(std::unordered_map<int64_t, int8_t> &cells,
                        std::unordered_set<int64_t> &hit_cells,
                        int x1, int y1)
  {
    int64_t key = packCellKey(x1, y1);
    if(hit_cells.insert(key).second)
      bumpLocal(cells, x1, y1, hit_delta_);
  }

  // Bresenham from this submap's own local origin (0,0) to a local hit cell,
  // miss-only pass: every intermediate cell on the way there is a miss —
  // UNLESS that same cell was itself a hit endpoint for some OTHER point in
  // this same frame, in which case this ray STOPS there instead of just
  // skipping that one cell and carrying on. A hit means something solid is
  // confirmed at that cell — this ray's own farther cells, beyond a
  // confirmed obstruction another point in this same frame already put in
  // its path, were never actually seen through by this ray at all; letting
  // it keep marking them as miss would erase real structure behind a wall
  // this exact frame just confirmed, using a ray that in reality never
  // reached back there (a different beam, at a different bearing/height,
  // that happened to pass a gap the wall doesn't cover). Same reason real
  // Cartographer applies every hit in a scan before any of that scan's
  // misses — this goes one step further and treats a same-frame hit as
  // opaque, not just protected. Unbounded up to that point, like the
  // global grid this feeds into.
  void raycastLocalMiss(std::unordered_map<int64_t, int8_t> &cells,
                         const std::unordered_set<int64_t> &hit_cells,
                         int x1, int y1)
  {
    int x0 = 0, y0 = 0;
    int dx = std::abs(x1 - x0), sx = (x1 >= x0) ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = (y1 >= y0) ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;
    while(true)
    {
      bool is_last = (x == x1 && y == y1);
      if(!is_last)
      {
        if(hit_cells.find(packCellKey(x, y)) != hit_cells.end())
          break; // occluded by a hit elsewhere in this same frame — stop here
        bumpLocal(cells, x, y, -miss_delta_);
      }
      if(is_last) break;
      int e2 = 2 * err;
      if(e2 >= dy) { err += dy; x += sx; }
      if(e2 <= dx) { err += dx; y += sy; }
    }
  }

  void onSubmap(const astribot_slam_msgs::msg::KeyframeSubmap::ConstSharedPtr &msg)
  {
    Submap sm;
    sm.x = msg->pose.position.x;
    sm.y = msg->pose.position.y;
    double yaw = yawFromQuat(msg->pose.orientation);
    sm.cos_yaw = std::cos(yaw);
    sm.sin_yaw = std::sin(yaw);

    if(msg->cloud.width * msg->cloud.height > 0)
    {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg->cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg->cloud, "y");
      // Two passes over this one frame's points: all hits first, then all
      // misses (see raycastLocalMiss above) — so no ray in this frame can
      // erode a cell this same frame genuinely hit.
      std::vector<std::pair<int,int>> hit_points;
      hit_points.reserve(msg->cloud.width * msg->cloud.height);
      std::unordered_set<int64_t> hit_cells;
      for(; iter_x != iter_x.end(); ++iter_x, ++iter_y)
      {
        int hit_col = (int)std::floor(*iter_x / resolution_);
        int hit_row = (int)std::floor(*iter_y / resolution_);
        hit_points.emplace_back(hit_col, hit_row);
        raycastLocalHit(sm.cells, hit_cells, hit_col, hit_row);
      }
      for(auto &hp: hit_points)
        raycastLocalMiss(sm.cells, hit_cells, hp.first, hp.second);
    }

    const auto key = packSubmapKey(msg->session_id, msg->keyframe_id);
    submaps_[key] = std::move(sm);
    auto pending = pending_poses_.find(key);
    if(pending != pending_poses_.end())
    {
      apply_pose(submaps_[key], pending->second);
      pending_poses_.erase(pending);
    }
    finish_export();
  }

  void onPoseArray(const astribot_slam_msgs::msg::KeyframePoseArray::ConstSharedPtr &msg)
  {
    for(auto &upd: msg->updates)
    {
      auto it = submaps_.find(packSubmapKey(upd.session_id, upd.keyframe_id));
      if(it == submaps_.end())
        pending_poses_[packSubmapKey(upd.session_id, upd.keyframe_id)] = upd.pose;
      else
        apply_pose(it->second, upd.pose);
    }
    if(msg->is_final && !msg->save_dir.empty()) pending_final_ = msg;
    finish_export();
  }

  void apply_pose(Submap &sm, const geometry_msgs::msg::Pose &pose)
  {
    sm.x = pose.position.x; sm.y = pose.position.y;
    const double yaw = yawFromQuat(pose.orientation);
    sm.cos_yaw = std::cos(yaw); sm.sin_yaw = std::sin(yaw);
  }

  void finish_export()
  {
    if(!pending_final_) return;
    for(const auto &upd: pending_final_->updates)
      if(submaps_.count(packSubmapKey(upd.session_id, upd.keyframe_id)) == 0) return;
    save_grid_to_disk(pending_final_->save_dir, pending_final_->map_name);
    pending_final_.reset();
  }

  // Promotes the live layer's current accumulated state into a permanent
  // historical-layer submap, at identity pose — its "local" cells ARE
  // world cells directly, so it composites in publish() exactly like a
  // real keyframe's submap, no changes needed there. Called only at the
  // stationary->moving transition (see onScan()); live_occupied_ is
  // cleared afterward so the NEXT stationary period starts fresh (it'll
  // build its own, possibly different, coverage from scratch — this
  // snapshot is what preserves the PREVIOUS one instead of just discarding
  // it the moment it stops being consulted). kSnapshotSessionId is a
  // reserved session_id no real voxel_slam keyframe will ever use, so
  // these never collide with real submaps; every snapshot ever taken
  // shares the same identity pose, so publish()'s dwell-cluster walk
  // always merges them together regardless of how much time separates
  // them — harmless here (unlike the live layer's own per-scan data, a
  // snapshot is already a settled, time-debounced, one-time promotion,
  // not a fragile, frequently-refreshed observation), and correct: two
  // snapshots only ever combine votes for cells they both actually
  // touched, i.e. the same real-world location confirmed more than once.
  void takeSnapshot()
  {
    if(live_occupied_.empty()) return;
    Submap sm; // identity pose (defaults): x=y=0, cos_yaw=1, sin_yaw=0
    for(auto &kv: live_occupied_)
      sm.cells[kv.first] = kv.second ? (int8_t)hit_delta_ : (int8_t)(-miss_delta_);
    // Steady-state per-snapshot status, not a one-off startup/warning
    // event — DEBUG (like the startup line above is INFO) so it doesn't
    // scroll the console on every snapshot; still available if debug
    // logging is ever turned on for this node.
    RCLCPP_DEBUG(get_logger(), "snapshot #%u: promoting %zu live cells into the historical layer",
                snapshot_counter_ + 1, sm.cells.size());
    submaps_[packSubmapKey(kSnapshotSessionId, ++snapshot_counter_)] = std::move(sm);
    live_occupied_.clear();
    live_miss_since_.clear();
  }

  // Live layer (see module comment) — only actually does anything while
  // is_moving_ is false. msg is voxel_slam's /map_scan_filtered: already
  // height-ROI-filtered, published in world_frame_ coordinates every LIO
  // scan regardless of keyframe activity, but this layer's own hit/miss
  // update is now gated to stationary periods only (see module comment):
  // while moving, dynamic-obstacle removal is back to the historical
  // layer's cross-submap score-sum + dwell-clustering (see publish()) —
  // this layer just tracks the TF's own velocity every scan (needed
  // regardless of is_moving_, to detect the next transition promptly) and,
  // once genuinely stationary, sets each cell a ray reaches DIRECTLY to
  // hit or miss in live_occupied_ — plain overwrite, no scoring — so
  // whatever the most recent scan says about a cell is what publish()
  // reports for it while stationary, overriding the historical layer
  // outright.
  void onScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
  {
    geometry_msgs::msg::TransformStamped tf;
    try
    {
      tf = tf_buffer_->lookupTransform(world_frame_, chassis_frame_, tf2::TimePointZero);
    }
    catch(const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "live layer: no TF %s->%s yet (%s), dropping this scan",
        world_frame_.c_str(), chassis_frame_.c_str(), ex.what());
      return;
    }

    const double scan_age = (now() - rclcpp::Time(msg->header.stamp)).seconds();
    const double tf_age = (now() - rclcpp::Time(tf.header.stamp)).seconds();
    if(msg->header.frame_id != world_frame_ || scan_age < -0.05 ||
       scan_age > source_max_age_ || tf_age < -0.05 || tf_age > source_max_age_)
      return;

    double tx = tf.transform.translation.x, ty = tf.transform.translation.y;
    double tyaw = yawFromQuat(tf.transform.rotation);
    rclcpp::Time t_now = now();
    bool was_moving = is_moving_;

    // Real-time stationary/moving decision straight off the TF — nothing
    // to do with keyframe cadence (an earlier "no keyframe for N seconds"
    // definition conflated "voxel_slam hasn't published one yet" with
    // "the robot hasn't actually moved", which isn't the same thing).
    if(have_last_tf_)
    {
      double dt = (t_now - last_tf_time_).seconds();
      if(dt > 1e-3)
      {
        double dx = tx - last_tf_x_, dy = ty - last_tf_y_;
        double dyaw = std::fabs(std::atan2(std::sin(tyaw - last_tf_yaw_), std::cos(tyaw - last_tf_yaw_)));
        double lin_vel = std::hypot(dx, dy) / dt;
        double ang_vel = dyaw / dt;
        is_moving_ = (lin_vel > stationary_lin_vel_thresh_) || (ang_vel > stationary_ang_vel_thresh_);
      }
    }
    else
    {
      have_last_tf_ = true;
      is_moving_ = true; // no velocity estimate yet — assume moving until we have one
    }
    last_tf_x_ = tx; last_tf_y_ = ty; last_tf_yaw_ = tyaw; last_tf_time_ = t_now;

    // The instant the robot starts moving again, promote whatever the live
    // layer had built up while stationary into a permanent historical-
    // layer submap (see takeSnapshot()) — otherwise that accumulated,
    // densely-confirmed coverage would just stop being consulted at all
    // (publish() only applies live_occupied_ while is_moving_ is false)
    // and the map would visibly thin back down to a single keyframe's own
    // sparse density the moment motion resumes, even though nothing about
    // the real world actually got any sparser.
    if(!was_moving && is_moving_)
      takeSnapshot();

    if(is_moving_ || msg->width * msg->height == 0) return;

    int ocol = (int)std::floor(tx / resolution_);
    int orow = (int)std::floor(ty / resolution_);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    std::vector<std::pair<int,int>> hit_points;
    hit_points.reserve((size_t)msg->width * msg->height);
    std::unordered_set<int64_t> hit_cells;
    for(; iter_x != iter_x.end(); ++iter_x, ++iter_y)
    {
      int col = (int)std::floor(*iter_x / resolution_);
      int row = (int)std::floor(*iter_y / resolution_);
      int64_t key = packCellKey(col, row);
      hit_points.emplace_back(col, row);
      hit_cells.insert(key);
      live_occupied_[key] = true;
      live_miss_since_.erase(key); // a hit resets this cell's miss streak
    }
    for(auto &hp: hit_points)
    {
      int x1 = hp.first, y1 = hp.second;
      int dx = std::abs(x1 - ocol), sx = (x1 >= ocol) ? 1 : -1;
      int dy = -std::abs(y1 - orow), sy = (y1 >= orow) ? 1 : -1;
      int err = dx + dy;
      int x = ocol, y = orow;
      while(true)
      {
        bool is_last = (x == x1 && y == y1);
        if(!is_last)
        {
          int64_t key = packCellKey(x, y);
          // A hit registered anywhere in THIS scan blocks this ray outright
          // — this ray's own farther cells were never actually seen through
          // (some other beam in this same scan just confirmed something
          // solid sits in the way; this ray reaching farther only means it
          // passed a gap that solid thing doesn't cover, at this beam's own
          // bearing/height — it says nothing about the cells behind it).
          if(hit_cells.find(key) != hit_cells.end())
            break;
          // A single miss doesn't clear a cell outright any more — needs
          // live_clear_min_duration_sec_ of REAL elapsed time with no
          // intervening hit, not just a handful of scan callbacks (hit
          // stays immediate/single-frame, on purpose: asymmetric, safety-
          // first). Counting scans instead of time was tried first and
          // rejected: a single scan's own point yield is sparse, and
          // MID360's non-repetitive pattern only fills those gaps in
          // given enough integration TIME — several scan callbacks can
          // fire close enough together that the same coverage gap simply
          // hasn't had a chance to be resampled yet, which would have
          // thinned or broken a genuinely solid, thick wall one cell at a
          // time. Requiring real wall-clock duration instead gives the
          // sensor's own integration time to actually fill the gap if the
          // obstruction is real, independent of however fast onScan() is
          // actually being called. Still doesn't help the one specific
          // failure mode already found and accepted (a real 3D->2D
          // flattening ambiguity that repeats identically forever, not
          // just briefly, so no amount of waiting tells it apart from
          // genuine clearing) — this is just an extra margin against
          // transient/sparse-coverage noise on top of that.
          auto streak_it = live_miss_since_.find(key);
          if(streak_it == live_miss_since_.end())
            live_miss_since_[key] = t_now; // first miss of a fresh streak
          else if((t_now - streak_it->second).seconds() >= live_clear_min_duration_sec_)
            live_occupied_[key] = false;
        }
        if(is_last) break;
        int e2 = 2 * err;
        if(e2 >= dy) { err += dy; x += sx; }
        if(e2 <= dx) { err += dx; y += sy; }
      }
    }
  }

  void publish()
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = world_frame_;
    grid.header.stamp = now();
    grid.info.map_load_time = grid.header.stamp;
    composite_grid(grid); // leaves width=height=0 (and grid.data empty) if nothing's been observed yet
    pub_->publish(grid);
  }

  // Builds the current composite grid — same historical-layer (cross-submap
  // score summation + dwell-clustering) + live-layer-override + unknown-
  // hole-fill this node has always used, just factored out so publish()'s
  // periodic ROS publish and save_grid_to_disk()'s one-shot pgm/yaml export
  // (see this file's module comment) share one implementation. Fills
  // grid.info.{width,height,origin} and grid.data; leaves grid.header alone
  // (that's each caller's own business). Returns false (grid left at
  // width=height=0) if nothing has ever been observed.
  bool composite_grid(nav_msgs::msg::OccupancyGrid &grid)
  {
    grid.info.resolution = resolution_;

    // Historical layer: reinstated cross-submap score summation +
    // dwell-clustering (see module comment) — this is what clears a
    // dynamic obstacle while the robot is MOVING (is_moving_ true), since
    // the live layer below is now gated to stationary periods only. Each
    // submap's own already-clamped local score feeds into that world
    // cell — not OR'd — so a stale hit from a since-departed dynamic
    // obstacle, seen by only one submap, gets pulled back down by every
    // later submap whose ray passed straight through that same world
    // cell. But submaps aren't independent observations just because
    // they're separate keyframes: dwelling near one spot produces a burst
    // of submaps from nearly the same viewpoint, and every one of those
    // near-duplicate submaps voting the same way would erode a genuinely
    // static edge cell not because of independently-confirmed
    // disagreement, but from loitering long enough to rack up correlated
    // votes — so submaps are walked in (session, keyframe_id) order and
    // consecutive ones whose CURRENT world pose stays within
    // dwell_cluster_radius_/dwell_cluster_angle_ of each other are folded
    // into one cluster, casting at most one hit_delta_/miss_delta_ vote
    // per world cell no matter how many member submaps it has. The
    // running sum is clamped to [score_min_, score_max_].
    std::unordered_map<int64_t, int32_t> world32;
    std::unordered_map<int64_t, int32_t> cluster; // world cell -> raw sum within the current cluster
    bool have_anchor = false;
    double anchor_x = 0.0, anchor_y = 0.0, anchor_yaw = 0.0;

    auto flush_cluster = [&]()
    {
      for(auto &kv: cluster)
      {
        // vote is 0 for a cluster that's a genuine wash on this cell — but
        // that's still "swept and seen as not-hit", not "never observed
        // at all". world32[kv.first] is looked up (default-inserted at 0
        // if absent) either way, so the cell stays present and free, not
        // -1 unknown, even then.
        int32_t vote = (kv.second > 0) ? hit_delta_ : (kv.second < 0 ? -miss_delta_ : 0);
        int32_t sum = world32[kv.first] + vote;
        world32[kv.first] = std::clamp(sum, (int32_t)score_min_, (int32_t)score_max_);
      }
      cluster.clear();
    };

    std::vector<uint64_t> ordered_keys;
    ordered_keys.reserve(submaps_.size());
    for(auto &kv: submaps_) ordered_keys.push_back(kv.first);
    std::sort(ordered_keys.begin(), ordered_keys.end()); // (session_id<<32|keyframe_id) -> per-session temporal order

    for(uint64_t key: ordered_keys)
    {
      const Submap &sm = submaps_.at(key);
      double yaw = std::atan2(sm.sin_yaw, sm.cos_yaw);

      if(have_anchor)
      {
        double dx = sm.x - anchor_x, dy = sm.y - anchor_y;
        double dyaw = std::fabs(std::atan2(std::sin(yaw - anchor_yaw), std::cos(yaw - anchor_yaw)));
        if(std::hypot(dx, dy) > dwell_cluster_radius_ || dyaw > dwell_cluster_angle_)
        {
          flush_cluster();
          have_anchor = false;
        }
      }
      if(!have_anchor)
      {
        anchor_x = sm.x; anchor_y = sm.y; anchor_yaw = yaw;
        have_anchor = true;
      }

      for(auto &cell_kv: sm.cells)
      {
        int col, row;
        unpackCellKey(cell_kv.first, col, row);
        double lx = (col + 0.5) * resolution_;
        double ly = (row + 0.5) * resolution_;
        double wx = sm.x + lx * sm.cos_yaw - ly * sm.sin_yaw;
        double wy = sm.y + lx * sm.sin_yaw + ly * sm.cos_yaw;
        int wcol = (int)std::floor(wx / resolution_);
        int wrow = (int)std::floor(wy / resolution_);
        int64_t wkey = packCellKey(wcol, wrow);

        cluster[wkey] += (int32_t)cell_kv.second;
      }
    }
    flush_cluster(); // last cluster never hit a break in the loop above

    std::unordered_map<int64_t, int8_t> world; // 1 = free-seen, 2 = occupied-seen
    for(auto &kv: world32)
      world[kv.first] = (kv.second > occ_threshold_) ? 2 : 1;

    // Live layer override — see module comment: only while the robot is
    // currently stationary (is_moving_ false). While moving, this layer
    // isn't even being fed fresh data (onScan() returns early), so
    // whatever it holds is stale and left out entirely — the historical
    // layer above is fully responsible for dynamic-obstacle removal while
    // moving.
    if(!is_moving_ && have_last_tf_ && (now() - last_tf_time_).seconds() <= source_max_age_)
    {
      for(auto &kv: live_occupied_)
        world[kv.first] = kv.second ? 2 : 1;
    }

    if(world.empty())
    {
      grid.info.width = 0;
      grid.info.height = 0;
      return false;
    }

    int min_col = std::numeric_limits<int>::max(), max_col = std::numeric_limits<int>::min();
    int min_row = std::numeric_limits<int>::max(), max_row = std::numeric_limits<int>::min();
    for(auto &kv: world)
    {
      int col, row;
      unpackCellKey(kv.first, col, row);
      min_col = std::min(min_col, col); max_col = std::max(max_col, col);
      min_row = std::min(min_row, row); max_row = std::max(max_row, row);
    }

    uint32_t width = (uint32_t)(max_col - min_col) + 1;
    uint32_t height = (uint32_t)(max_row - min_row) + 1;

    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = min_col * resolution_;
    grid.info.origin.position.y = min_row * resolution_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.assign((size_t)width * height, -1); // unknown by default
    for(auto &kv: world)
    {
      int col, row;
      unpackCellKey(kv.first, col, row);
      size_t idx = (size_t)(row - min_row) * width + (col - min_col);
      grid.data[idx] = (kv.second == 2) ? 100 : 0;
    }

    // Fill isolated unknown "holes" that are almost entirely ringed by free
    // space — see the module comment above for why they exist. Single pass
    // over a snapshot (not iterative/cascading): a cell is only promoted
    // to free if, within its unknown_fill_radius_ neighborhood, at least
    // unknown_fill_min_free_frac_ of the cells are free AND none are
    // occupied — the occupied veto means this can never erode safety
    // margin by turning unknown space next to a real obstacle into free
    // space; the free-fraction requirement means a genuinely unexplored
    // region (mostly ringed by other unknown cells, not free ones) is left
    // alone.
    if(unknown_fill_radius_ > 0)
    {
      int r = unknown_fill_radius_;
      std::vector<int8_t> snapshot = grid.data;
      int window_cells = (2 * r + 1) * (2 * r + 1) - 1;
      int min_free_needed = (int)std::ceil(unknown_fill_min_free_frac_ * window_cells);
      for(uint32_t row = 0; row < height; ++row)
      {
        for(uint32_t col = 0; col < width; ++col)
        {
          size_t idx = (size_t)row * width + col;
          if(snapshot[idx] != -1) continue; // only ever fill unknown cells

          int free_count = 0;
          bool occupied_nearby = false;
          for(int dr = -r; dr <= r && !occupied_nearby; ++dr)
          {
            int nr = (int)row + dr;
            if(nr < 0 || nr >= (int)height) continue;
            for(int dc = -r; dc <= r; ++dc)
            {
              if(dr == 0 && dc == 0) continue;
              int nc = (int)col + dc;
              if(nc < 0 || nc >= (int)width) continue;
              int8_t v = snapshot[(size_t)nr * width + nc];
              if(v == 100) { occupied_nearby = true; break; }
              if(v == 0) ++free_count;
            }
          }
          if(!occupied_nearby && free_count >= min_free_needed)
            grid.data[idx] = 0;
        }
      }
    }

    if(body_half_extent_ > 0)
    {
      try
      {
        const auto tf = tf_buffer_->lookupTransform(world_frame_, chassis_frame_, tf2::TimePointZero);
        const double age = (now() - rclcpp::Time(tf.header.stamp)).seconds();
        const double half = body_half_extent_ - resolution_ * std::sqrt(0.5);
        if(age >= -0.05 && age <= source_max_age_ && half > 0)
        {
          const double tx = tf.transform.translation.x, ty = tf.transform.translation.y;
          const double yaw = yawFromQuat(tf.transform.rotation);
          const double c = std::cos(yaw), sn = std::sin(yaw);
          const int reach = std::ceil(body_half_extent_ * std::sqrt(2.0) / resolution_);
          const int cx = std::floor(tx / resolution_) - min_col;
          const int cy = std::floor(ty / resolution_) - min_row;
          for(int y = std::max(0, cy-reach); y <= std::min(int(height)-1, cy+reach); ++y)
            for(int x = std::max(0, cx-reach); x <= std::min(int(width)-1, cx+reach); ++x)
            {
              const double dx = (x+min_col+0.5)*resolution_-tx;
              const double dy = (y+min_row+0.5)*resolution_-ty;
              if(std::abs(c*dx+sn*dy) <= half && std::abs(-sn*dx+c*dy) <= half)
                grid.data[size_t(y)*width+x] = 0;
            }
        }
      }
      catch(const tf2::TransformException &) {} // No fresh pose means no clearing.
    }

    return true;
  }

  // One-shot final export triggered by voxel_slam's is_final/save_dir/
  // map_name cue (see this file's module comment and onPoseArray() above) —
  // builds the SAME composite grid publish() would show right now, and
  // writes it as save_dir+map_name+".pgm"/".yaml".
  void save_grid_to_disk(const std::string &save_dir, const std::string &map_name)
  {
    nav_msgs::msg::OccupancyGrid grid;
    if(!composite_grid(grid))
    {
      RCLCPP_WARN(get_logger(), "save_grid_to_disk: composite grid is empty — nothing to save (dir:%s map_name:%s)",
                  save_dir.c_str(), map_name.c_str());
      return;
    }

    std::string pgm_filename = map_name + ".pgm";
    std::string pgm_path = save_dir + pgm_filename;
    std::string yaml_path = save_dir + map_name + ".yaml";

    bool ok_pgm = write_pgm(pgm_path, grid);
    bool ok_yaml = write_yaml(yaml_path, pgm_filename, grid);

    if(ok_pgm && ok_yaml)
      RCLCPP_INFO(get_logger(), "saved final global map | pgm:%s yaml:%s size:%ux%u origin:(%.3f,%.3f)",
                  pgm_path.c_str(), yaml_path.c_str(), grid.info.width, grid.info.height,
                  grid.info.origin.position.x, grid.info.origin.position.y);
    else
      RCLCPP_ERROR(get_logger(), "failed to save final global map | pgm_ok:%d yaml_ok:%d dir:%s",
                   (int)ok_pgm, (int)ok_yaml, save_dir.c_str());
  }

  // Raw binary P5 pgm, standard map_server/Nav2 "trinary" convention: 254
  // free, 0 occupied, 205 unknown, row order flipped so pixel row 0 (top of
  // the image) is the grid's largest-row/northernmost data row — matches
  // ROS map_saver byte-for-byte. Written by hand (this package has no
  // OpenCV/image-codec dependency) rather than via any imwrite-style call.
  bool write_pgm(const std::string &path, const nav_msgs::msg::OccupancyGrid &grid)
  {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if(!f.is_open()) return false;
    uint32_t width = grid.info.width, height = grid.info.height;
    f << "P5\n# CREATOR: nav_prob_grid_node\n" << width << " " << height << "\n255\n";
    std::vector<unsigned char> row(width);
    for(uint32_t y = 0; y < height; ++y)
    {
      for(uint32_t x = 0; x < width; ++x)
      {
        int8_t v = grid.data[(size_t)(height - y - 1) * width + x];
        unsigned char pixel;
        if(v == 0) pixel = 254;        // free
        else if(v == 100) pixel = 0;   // occupied
        else pixel = 205;              // unknown (-1)
        row[x] = pixel;
      }
      f.write(reinterpret_cast<const char*>(row.data()), width);
    }
    return f.good();
  }

  // Standard map_server/Nav2 map-metadata yaml (image/mode/resolution/
  // origin/negate/occupied_thresh/free_thresh).
  bool write_yaml(const std::string &path, const std::string &pgm_filename,
                   const nav_msgs::msg::OccupancyGrid &grid)
  {
    std::ofstream f(path, std::ios::trunc);
    if(!f.is_open()) return false;
    f << "image: " << pgm_filename << "\n";
    f << "mode: trinary\n";
    f << "resolution: " << grid.info.resolution << "\n";
    f << "origin: [" << grid.info.origin.position.x << ", " << grid.info.origin.position.y << ", 0]\n";
    f << "negate: " << negate_ << "\n";
    f << "occupied_thresh: " << occupied_thresh_ << "\n";
    f << "free_thresh: " << free_thresh_ << "\n";
    return f.good();
  }

  bool enable_ = true;
  std::string submap_topic_, pose_array_topic_, output_topic_, world_frame_;
  std::string raw_scan_topic_, chassis_frame_;
  double resolution_ = 0.05;
  double body_half_extent_ = 0.31, source_max_age_ = 0.5;
  int hit_delta_ = 6, miss_delta_ = 2;
  int score_min_ = -100, score_max_ = 100;
  int occ_threshold_ = 0;
  int unknown_fill_radius_ = 1;
  double unknown_fill_min_free_frac_ = 0.6;
  double dwell_cluster_radius_ = 0.3;  // meters
  double dwell_cluster_angle_ = 15.0 * M_PI / 180.0; // radians

  // Real-time stationary/moving decision, straight off the live TF — see
  // onScan(). is_moving_ starts true (safe default: don't trust the live
  // layer until we've actually measured a low enough velocity).
  double stationary_lin_vel_thresh_ = 0.02; // m/s
  double stationary_ang_vel_thresh_ = 0.05; // rad/s
  bool have_last_tf_ = false;
  double last_tf_x_ = 0.0, last_tf_y_ = 0.0, last_tf_yaw_ = 0.0;
  rclcpp::Time last_tf_time_;
  bool is_moving_ = true;
  double live_clear_min_duration_sec_ = 0.5;
  // Plain map_server/Nav2 yaml metadata for the one-shot final save — see
  // write_yaml(). Not used by any grid-building/compositing code above.
  int negate_ = 0;
  double occupied_thresh_ = 0.65;
  double free_thresh_ = 0.25;
  // Snapshots promoting the live layer into the historical layer at each
  // stationary->moving transition — see takeSnapshot(). Reserved
  // session_id no real voxel_slam keyframe will ever use.
  static constexpr uint32_t kSnapshotSessionId = 0xFFFFFFFEu;
  uint32_t snapshot_counter_ = 0;

  std::unordered_map<uint64_t, Submap> submaps_;
  std::unordered_map<uint64_t, geometry_msgs::msg::Pose> pending_poses_;
  astribot_slam_msgs::msg::KeyframePoseArray::ConstSharedPtr pending_final_;
  // Live layer's persistent, always-updated verdict per world cell: true =
  // occupied, false = free. Absence = never touched by the live layer, so
  // the historical layer's own verdict (or unknown) applies instead. Only
  // consulted by publish() while is_moving_ is false (see module comment).
  std::unordered_map<int64_t, bool> live_occupied_;
  // When each cell's current unbroken clean-miss streak started (live
  // layer only) — see onScan(). Absence = no streak in progress (never
  // missed, or last touch was a hit).
  std::unordered_map<int64_t, rclcpp::Time> live_miss_since_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Subscription<astribot_slam_msgs::msg::KeyframeSubmap>::SharedPtr submap_sub_;
  rclcpp::Subscription<astribot_slam_msgs::msg::KeyframePoseArray>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_scan_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavProbGridNode>());
  rclcpp::shutdown();
  return 0;
}
