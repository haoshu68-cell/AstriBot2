#include "astribot_s1_manipulation_perception/inference_contract.hpp"
#include "astribot_s1_manipulation_perception/worker_process.hpp"
#include <astribot_perception_msgs/action/compute_grasps.hpp>
#include <astribot_perception_msgs/action/estimate_object_pose.hpp>
#include <astribot_perception_msgs/msg/camera_health.hpp>
#include <astribot_perception_msgs/msg/projection_health.hpp>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <tf2/LinearMath/Matrix3x3.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>

namespace ai = astribot::inference;
namespace pm = astribot_perception_msgs;
using Grasp = pm::action::ComputeGrasps;
using Pose = pm::action::EstimateObjectPose;
using Steady = std::chrono::steady_clock;

namespace {
int64_t ns(const builtin_interfaces::msg::Time &t) {
  return t.sec < 0 || t.nanosec >= 1000000000u
             ? 0
             : int64_t(t.sec) * 1000000000LL + t.nanosec;
}
bool identifier(const std::string &s) {
  return !s.empty() && s.size() <= 128 && s.find('\0') == std::string::npos;
}
std::string file_digest(const std::string &path,
                        const std::function<void()> &guard = {}) {
  if (guard)
    guard();
  if (!std::filesystem::path(path).is_absolute() ||
      !std::filesystem::is_regular_file(path))
    throw std::runtime_error("MODEL_PATH_INVALID");
  if (std::filesystem::file_size(path) > 1024ULL * 1024 * 1024)
    throw std::runtime_error("MODEL_SIZE_LIMIT");
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error("MODEL_READ_FAILED");
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(),
                                                              EVP_MD_CTX_free);
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("HASH_INIT_FAILED");
  char block[65536];
  while (in) {
    if (guard)
      guard();
    in.read(block, sizeof(block));
    if (EVP_DigestUpdate(ctx.get(), block, in.gcount()) != 1)
      throw std::runtime_error("HASH_FAILED");
  }
  if (!in.eof())
    throw std::runtime_error("MODEL_READ_FAILED");
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned length = 0;
  if (EVP_DigestFinal_ex(ctx.get(), digest, &length) != 1)
    throw std::runtime_error("HASH_FAILED");
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (unsigned i = 0; i < length; ++i)
    out << std::setw(2) << unsigned(digest[i]);
  return out.str();
}
geometry_msgs::msg::Quaternion quaternion(const double *r) {
  if (!ai::valid_rotation(r))
    throw std::runtime_error("INVALID_BACKEND_ROTATION");
  tf2::Matrix3x3 matrix(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
  tf2::Quaternion q;
  matrix.getRotation(q);
  q.normalize();
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}
} // namespace

struct Model {
  std::string path, revision, symmetry;
  std::string visibility_path, visibility_revision;
};

class ManipulationPerceptionServer : public rclcpp::Node {
public:
  explicit ManipulationPerceptionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("manipulation_perception_server", options) {
    camera_id_ = fixed<std::string>("camera_id", "head_rgbd");
    frame_ =
        fixed<std::string>("optical_frame", "head_rgbd_camera_optical_frame");
    calibration_ = fixed<int64_t>("calibration_revision", 0);
    scene_ = fixed<int64_t>("planning_scene_revision", 0);
    envelope_ = fixed<int64_t>("envelope_epoch", 0);
    input_age_ = fixed<double>("max_input_age_sec", .5);
    result_age_ = fixed<double>("max_result_age_sec", 5.);
    health_age_ = fixed<double>("max_health_age_sec", .5);
    if (input_age_ <= 0 || input_age_ > .5 || !std::isfinite(input_age_) ||
        result_age_ <= 0 || result_age_ > 30 || !std::isfinite(result_age_) ||
        health_age_ <= 0 || health_age_ > .5 || !std::isfinite(health_age_))
      throw std::runtime_error("INVALID_TIME_BUDGET");
    grasp_worker_ = fixed<std::string>("grasp_worker", "");
    grasp_model_ = fixed<std::string>("grasp_model", "");
    device_ = fixed<std::string>("grasp_device", "cuda");
    if (device_ != "cpu" && device_ != "cuda")
      throw std::runtime_error("INVALID_DEVICE");
    if (!grasp_worker_.empty()) {
      check_worker(grasp_worker_);
      grasp_revision_ = file_digest(grasp_model_);
    }
    pose_worker_ = fixed<std::string>("pose_worker", "");
    const auto registry = fixed<std::string>("model_registry", "");
    if (!pose_worker_.empty()) {
      check_worker(pose_worker_);
      std::ifstream in(registry);
      nlohmann::json data;
      in >> data;
      if (!data.is_object() || data.size() > 32)
        throw std::runtime_error("INVALID_MODEL_REGISTRY");
      for (auto it = data.begin(); it != data.end(); ++it) {
        Model m;
        m.path = it.value().at("path").get<std::string>();
        m.revision = file_digest(m.path);
        m.visibility_path = it.value().value("visibility_model", "");
        if (!m.visibility_path.empty())
          m.visibility_revision = file_digest(m.visibility_path);
        m.symmetry = it.value().value("symmetry", "");
        if (!identifier(it.key()) ||
            (!m.symmetry.empty() && m.symmetry != "box" &&
             m.symmetry != "square_prism_z"))
          throw std::runtime_error("INVALID_MODEL_REGISTRY");
        models_.emplace(it.key(), std::move(m));
      }
    }
    health_sub_ = create_subscription<pm::msg::CameraHealth>(
        fixed<std::string>("camera_health_topic",
                           "/perception/camera_health/head_rgbd"),
        rclcpp::QoS(10).reliable(),
        [this](pm::msg::CameraHealth::ConstSharedPtr h) {
          if (h->camera_id != camera_id_)
            return;
          std::lock_guard<std::mutex> lock(health_mutex_);
          const auto now_ns = now().nanoseconds();
          observe_clock_locked(now_ns);
          // Independent /clock delivery can lag a same-context heartbeat.
          // Discard it without adopting its timestamp or refreshing old data.
          // Explicit revocation and changed context still invalidate below.
          if(ns(h->header.stamp)>now_ns&&h->valid&&(!health_||
             (h->source_epoch==health_->source_epoch&&h->calibration_revision==health_->calibration_revision&&
              h->frame_id==health_->frame_id&&h->header.frame_id==health_->header.frame_id)))return;
          if (!h->valid ||
              (health_ &&
               (h->source_epoch != health_->source_epoch ||
                h->calibration_revision != health_->calibration_revision ||
                h->frame_id != health_->frame_id ||
                ns(h->capture_stamp) < ns(health_->capture_stamp))))
            ++health_generation_;
          health_ = h;
          health_received_ = Steady::now();
        });
    // A snapshot supplied by another provider has its own provenance. Only
    // deployments explicitly using this projector opt into its extra contract.
    const auto projection_topic=fixed<std::string>("projection_health_topic","");
    require_projection_=!projection_topic.empty();
    if(require_projection_)projection_sub_=create_subscription<pm::msg::ProjectionHealth>(
      projection_topic,rclcpp::QoS(10).reliable().transient_local(),
      [this](pm::msg::ProjectionHealth::ConstSharedPtr h) {
        if(h->camera_id!=camera_id_)return;
        std::lock_guard<std::mutex> lock(health_mutex_);
        const auto current=now().nanoseconds();
        observe_clock_locked(current);
        if(projection_&&(ns(h->header.stamp)<ns(projection_->header.stamp)||
           (h->processing_epoch==projection_->processing_epoch&&h->sequence<=projection_->sequence)))return;
        if(ns(h->header.stamp)>current&&h->valid&&(!projection_||
           (h->processing_epoch==projection_->processing_epoch&&h->header.frame_id==projection_->header.frame_id)))return;
        if(!h->valid||!projection_||h->processing_epoch!=projection_->processing_epoch||
           h->header.frame_id!=projection_->header.frame_id)++health_generation_;
        if(!projection_||h->processing_epoch!=projection_->processing_epoch||
           ns(h->capture_stamp)!=ns(projection_->capture_stamp))projection_capture_received_=Steady::now();
        projection_=h;projection_received_=Steady::now();
      });
    grasp_server_ = make_server<Grasp>("/perception/compute_grasps");
    pose_server_ = make_server<Pose>("/perception/estimate_object_pose");
    RCLCPP_INFO(get_logger(),
                "Inference actions ready; no execution or collision approval. "
                "Grasp=%s pose_models=%zu",
                grasp_worker_.empty() ? "disabled" : "enabled", models_.size());
  }
  ~ManipulationPerceptionServer() override {
    stopping_ = true;
    if (worker_.joinable())
      worker_.join();
  }

private:
  bool observe_clock_locked(int64_t current) {
    const bool rollback=last_ros_time_&&current<last_ros_time_;
    if(rollback){++health_generation_;health_.reset();projection_.reset();}
    last_ros_time_=current;return rollback;
  }
  template <class T> T fixed(const std::string &name, const T &value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  static void check_worker(const std::string &path) {
    if (!std::filesystem::path(path).is_absolute() ||
        access(path.c_str(), X_OK) != 0)
      throw std::runtime_error("WORKER_PATH_INVALID");
  }
  template <class Action>
  typename rclcpp_action::Server<Action>::SharedPtr
  make_server(const std::string &name) {
    return rclcpp_action::create_server<Action>(
        this, name,
        [this](const rclcpp_action::GoalUUID &,
               std::shared_ptr<const typename Action::Goal> goal) {
          const auto reason = validate(*goal, true);
          if (!reason.empty()) {
            RCLCPP_WARN(get_logger(), "Goal rejected: %s", reason.c_str());
            return rclcpp_action::GoalResponse::REJECT;
          }
          bool expected = false;
          if (!busy_.compare_exchange_strong(expected, true))
            return rclcpp_action::GoalResponse::REJECT;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](
            std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> handle) {
          if (worker_.joinable())
            worker_.join();
          worker_ = std::thread([this, handle] {
            auto result = std::make_shared<typename Action::Result>();
            auto start = Steady::now();
            work_started_ = start;
            uint64_t generation;
            {
              std::lock_guard<std::mutex> lock(health_mutex_);
              generation = health_generation_;
            }
            work_generation_ = generation;
            try {
              auto reason = validate(*handle->get_goal(), true);
              if (!reason.empty())
                throw std::runtime_error(reason);
              execute(handle, *result);
              if (std::chrono::duration<double>(Steady::now() - start)
                      .count() >= handle->get_goal()->timeout_sec)
                throw std::runtime_error("INFERENCE_TIMEOUT");
              reason = validate(*handle->get_goal(), false);
              if (!reason.empty())
                throw std::runtime_error(reason);
              {
                std::lock_guard<std::mutex> lock(health_mutex_);
                if (generation != health_generation_)
                  throw std::runtime_error(
                      "CAMERA_CONTEXT_CHANGED_DURING_INFERENCE");
              }
              if (handle->is_canceling() || stopping_)
                throw std::runtime_error("CANCELED");
            } catch (const std::exception &e) {
              result->success = false;
              result->reason_code = e.what();
              clear(*result);
            }
            result->inference_time_sec =
                std::chrono::duration<float>(Steady::now() - start).count();
            try {
              if (handle->is_canceling()) {
                result->success = false;
                result->reason_code = "CANCELED";
                clear(*result);
                handle->canceled(result);
              } else if (result->success)
                handle->succeed(result);
              else
                handle->abort(result);
            } catch (const std::exception &e) {
              RCLCPP_WARN(get_logger(), "Action completion unavailable: %s",
                          e.what());
            }
            busy_ = false;
          });
        });
  }
  template <class Goal> std::string common(const Goal &g, bool admission) {
    if (!identifier(g.task_id) || !identifier(g.object_id) ||
        !identifier(g.source_epoch))
      return "SNAPSHOT_ID_MISSING";
    if (g.camera_id != camera_id_ || g.header.frame_id != frame_ ||
        g.object_cloud.header.frame_id != frame_ ||
        ns(g.header.stamp) != ns(g.object_cloud.header.stamp))
      return "SNAPSHOT_FRAME_OR_STAMP_MISMATCH";
    if (calibration_ <= 0 || scene_ <= 0 || envelope_ <= 0 ||
        g.calibration_revision != uint64_t(calibration_) ||
        g.planning_scene_revision != uint64_t(scene_) ||
        g.envelope_epoch != uint64_t(envelope_))
      return "CONTEXT_REVISION_MISMATCH";
    if (!std::isfinite(g.timeout_sec) || g.timeout_sec <= 0 ||
        g.timeout_sec > 120)
      return "INVALID_TIMEOUT";
    // Sampling and comparing the ROS clock must share the health lock. Otherwise
    // a callback can commit a newer time between this read and last_ros_time_.
    std::lock_guard<std::mutex> lock(health_mutex_);
    const auto now_ns = now().nanoseconds();
    if(observe_clock_locked(now_ns))return "CLOCK_ROLLBACK";
    if (!ai::valid_window(ns(g.header.stamp), now_ns, ns(g.valid_until),
                          admission ? input_age_ : result_age_, result_age_))
      return admission ? "INPUT_EXPIRED_OR_FUTURE" : "RESULT_EXPIRED";
    if (g.object_cloud.data.size() > 64000000 ||
        uint64_t(g.object_cloud.width) * g.object_cloud.height > 200000)
      return "CLOUD_LIMIT_EXCEEDED";
    if (!health_ || !health_->valid ||
        std::chrono::duration<double>(Steady::now() - health_received_)
                .count() > health_age_ ||
        !ai::valid_window(ns(health_->capture_stamp), now_ns,
                          ns(health_->valid_until), health_age_, 5.) ||
        ns(health_->header.stamp) > now_ns ||
        now_ns - ns(health_->header.stamp) > health_age_ * 1e9)
      return "CAMERA_HEALTH_INVALID";
    if (health_->source_epoch != g.source_epoch ||
        health_->calibration_revision != g.calibration_revision ||
        health_->frame_id != frame_ || health_->header.frame_id != frame_)
      return "CAMERA_CONTEXT_MISMATCH";
    if(require_projection_) {
      if(!projection_||!projection_->valid||
         std::chrono::duration<double>(Steady::now()-projection_received_).count()>.25||
         std::chrono::duration<double>(Steady::now()-projection_capture_received_).count()>.25||
         !ai::valid_window(ns(projection_->capture_stamp),now_ns,ns(projection_->valid_until),.25,.5)||
         ns(projection_->header.stamp)>now_ns||now_ns-ns(projection_->header.stamp)>250000000LL)
        return "PROJECTION_HEALTH_INVALID";
      if(g.processing_epoch.empty()||g.processing_epoch!=projection_->processing_epoch||
         projection_->header.frame_id!=frame_||ns(projection_->epoch_first_capture_stamp)<=0||
         ns(g.header.stamp)<ns(projection_->epoch_first_capture_stamp)||
         (admission&&ns(g.header.stamp)>ns(projection_->capture_stamp)))
        return "PROJECTION_CONTEXT_MISMATCH";
    }else if(!g.processing_epoch.empty())return "PROJECTION_SOURCE_UNCONFIGURED";
    return {};
  }
  std::string validate(const Grasp::Goal &g, bool admission) {
    auto e = common(g, admission);
    if (!e.empty())
      return e;
    if (grasp_worker_.empty())
      return "GRASPNET_BACKEND_DISABLED";
    if (!identifier(g.arm_id) || g.max_candidates == 0 ||
        g.max_candidates > 128)
      return "INVALID_GRASP_REQUEST";
    if (!g.workspace.points.empty())
      return "WORKSPACE_UNSUPPORTED_USE_SEGMENTED_CLOUD";
    return {};
  }
  std::string validate(const Pose::Goal &g, bool admission) {
    auto e = common(g, admission);
    if (!e.empty())
      return e;
    return models_.count(g.model_id) ? "" : "MODEL_NOT_REGISTERED";
  }
  static void clear(Grasp::Result &r) { r.candidates.clear(); }
  static void clear(Pose::Result &r) {
    r.observation = pm::msg::ObjectPoseObservation();
    r.model_coverage = 0;
    r.visible_model_coverage = 0;
    r.visibility_model_used = false;
    r.observable_normal_directions = 0;
    r.scene_coverage = 0;
    r.rmse_m = 0;
    r.symmetry_equivalent = false;
  }
  template <class Action>
  void
  check_inflight(std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> h) {
    if (h->is_canceling() || stopping_)
      throw std::runtime_error("CANCELED");
    if (std::chrono::duration<double>(Steady::now() - work_started_).count() >=
        h->get_goal()->timeout_sec)
      throw std::runtime_error("INFERENCE_TIMEOUT");
    const auto reason = validate(*h->get_goal(), false);
    if (!reason.empty())
      throw std::runtime_error(reason);
    std::lock_guard<std::mutex> lock(health_mutex_);
    if (work_generation_ != health_generation_)
      throw std::runtime_error("CAMERA_CONTEXT_CHANGED_DURING_INFERENCE");
  }
  template <class Action>
  ai::WorkerResult
  run(std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> h,
      const std::vector<std::string> &args, const std::string &log,
      uint32_t count, bool allow_rejection_json = false) {
    auto feedback = std::make_shared<typename Action::Feedback>();
    feedback->processed_points = count;
    feedback->state = "INFERENCE_RUNNING";
    h->publish_feedback(feedback);
    check_inflight<Action>(h);
    const double remaining =
        h->get_goal()->timeout_sec -
        std::chrono::duration<double>(Steady::now() - work_started_).count();
    std::string context_error;
    auto result = ai::run_worker(args, log, remaining, [&] {
      try {
        check_inflight<Action>(h);
      } catch (const std::exception &e) {
        context_error = e.what();
        return true;
      }
      return false;
    });
    if (!context_error.empty())
      throw std::runtime_error(context_error);
    if (result.reason != "OK" &&
        !(allow_rejection_json && result.reason == "WORKER_FAILED" &&
          result.exit_code == 2))
      throw std::runtime_error(result.reason);
    return result;
  }
  void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Grasp>> h,
               Grasp::Result &result) {
    const auto &g = *h->get_goal();
    auto points = ai::decode_cloud(g.object_cloud, 2048);
    ai::JobDirectory dir;
    if (file_digest(grasp_model_, [&] { check_inflight<Grasp>(h); }) !=
        grasp_revision_)
      throw std::runtime_error("MODEL_REVISION_CHANGED");
    auto input = dir.path() + "/cloud.bin", output = dir.path() + "/grasps.bin";
    {
      std::ofstream file(input, std::ios::binary);
      file.write(reinterpret_cast<const char *>(points.data()),
                 points.size() * sizeof(float));
      if (!file)
        throw std::runtime_error("INPUT_WRITE_FAILED");
    }
    run<Grasp>(h,
               {grasp_worker_, "--model", grasp_model_, "--input", input,
                "--output", output, "--device", device_},
               dir.path() + "/worker.log", points.size() / 3);
    if (file_digest(grasp_model_, [&] { check_inflight<Grasp>(h); }) !=
        grasp_revision_)
      throw std::runtime_error("MODEL_REVISION_CHANGED");
    if (!std::filesystem::is_regular_file(output))
      throw std::runtime_error("MISSING_BACKEND_OUTPUT");
    auto size = std::filesystem::file_size(output);
    if (size == 0 || size % 68 != 0 || size > 68 * 10000)
      throw std::runtime_error("INVALID_GRASP_OUTPUT_SIZE");
    std::vector<float> raw(size / 4);
    {
      std::ifstream in(output, std::ios::binary);
      in.read(reinterpret_cast<char *>(raw.data()), size);
      if (!in)
        throw std::runtime_error("BACKEND_READ_FAILED");
    }
    std::vector<pm::msg::GraspCandidate> candidates;
    result.raw_candidate_count = raw.size() / 17;
    for (size_t i = 0; i < raw.size(); i += 17) {
      const auto *p = raw.data() + i;
      for (size_t j = 0; j < 17; ++j)
        if (!std::isfinite(p[j]))
          throw std::runtime_error("NONFINITE_GRASP_OUTPUT");
      if (p[1] < 0 || p[1] > .2 || p[2] <= 0 || p[2] > .2 || p[3] <= 0 ||
          p[3] > .2 || std::abs(p[13]) > 10 || std::abs(p[14]) > 10 ||
          p[15] <= 0 || p[15] > 10)
        throw std::runtime_error("INVALID_GRASP_GEOMETRY");
      pm::msg::GraspCandidate c;
      c.header = g.header;
      c.source_epoch = g.source_epoch;
      c.candidate_id = g.task_id + ":" + std::to_string(i / 17);
      c.object_id = g.object_id;
      c.arm_id = g.arm_id;
      c.camera_id = g.camera_id;
      c.grasp_pose.header = g.header;
      c.grasp_pose.pose.position.x = p[13];
      c.grasp_pose.pose.position.y = p[14];
      c.grasp_pose.pose.position.z = p[15];
      double r[9];
      for (int j = 0; j < 9; ++j)
        r[j] = p[4 + j];
      c.grasp_pose.pose.orientation = quaternion(r);
      // Ordinary regression outputs can have nonpositive scores or zero-clamped
      // widths. Filter these proposals after checking the output structure.
      if (p[0] <= 0 || p[1] == 0) {
        ++result.filtered_candidate_count;
        continue;
      }
      c.raw_model_score = p[0];
      c.score = double(p[0]) / (1. + double(p[0]));
      c.score_semantics = "graspnet_raw_over_one_plus_raw_not_probability";
      c.gripper_width_m = p[1];
      c.gripper_height_m = p[2];
      c.gripper_depth_m = p[3];
      c.geometry_valid = true;
      // Network proposals are NOT a scene collision or robot IK check.
      c.collision_checked = false;
      c.collision_free = false;
      c.reason_code = "REQUIRES_MTC_VALIDATION";
      c.model_name = "graspnet_baseline_torchscript";
      c.model_revision = grasp_revision_;
      c.calibration_revision = g.calibration_revision;
      c.planning_scene_revision = g.planning_scene_revision;
      c.envelope_epoch = g.envelope_epoch;
      c.valid_until = g.valid_until;
      candidates.push_back(std::move(c));
    }
    std::stable_sort(
        candidates.begin(), candidates.end(),
        [](const auto &a, const auto &b) { return a.score > b.score; });
    if (candidates.size() > g.max_candidates)
      candidates.resize(g.max_candidates);
    if (candidates.empty())
      throw std::runtime_error("NO_USABLE_GRASPS");
    result.candidates = std::move(candidates);
    result.success = true;
    result.reason_code = "PROPOSALS_REQUIRE_MTC_VALIDATION";
  }
  void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Pose>> h,
               Pose::Result &result) {
    const auto &g = *h->get_goal();
    auto points = ai::decode_cloud(g.object_cloud);
    const auto &model = models_.at(g.model_id);
    ai::JobDirectory dir;
    if (file_digest(model.path, [&] { check_inflight<Pose>(h); }) !=
        model.revision)
      throw std::runtime_error("MODEL_REVISION_CHANGED");
    if (!model.visibility_path.empty() &&
        file_digest(model.visibility_path, [&] { check_inflight<Pose>(h); }) !=
            model.visibility_revision)
      throw std::runtime_error("MODEL_REVISION_CHANGED");
    auto input = dir.path() + "/scene.xyz", output = dir.path() + "/pose.json";
    {
      std::ofstream file(input);
      file << std::setprecision(9);
      for (size_t i = 0; i < points.size(); i += 3)
        file << points[i] << ' ' << points[i + 1] << ' ' << points[i + 2]
             << '\n';
      if (!file)
        throw std::runtime_error("INPUT_WRITE_FAILED");
    }
    std::vector<std::string> args{pose_worker_, "--model", model.path,
                                  "--scene",    input,     "--output",
                                  output};
    if (!model.symmetry.empty()) {
      args.push_back("--symmetry");
      args.push_back(model.symmetry);
    }
    if (!model.visibility_path.empty()) {
      args.push_back("--visibility-model");
      args.push_back(model.visibility_path);
    }
    run<Pose>(h, args, dir.path() + "/worker.log", points.size() / 3, true);
    if (file_digest(model.path, [&] { check_inflight<Pose>(h); }) !=
        model.revision)
      throw std::runtime_error("MODEL_REVISION_CHANGED");
    if (!model.visibility_path.empty() &&
        file_digest(model.visibility_path, [&] { check_inflight<Pose>(h); }) !=
            model.visibility_revision)
      throw std::runtime_error("MODEL_REVISION_CHANGED");
    if (!std::filesystem::is_regular_file(output) ||
        std::filesystem::file_size(output) > 4000000)
      throw std::runtime_error("INVALID_POSE_OUTPUT_SIZE");
    nlohmann::json data;
    {
      std::ifstream file(output);
      file >> data;
    }
    if (!data.at("success").get<bool>())
      throw std::runtime_error(data.value("reason", "POSE_REJECTED"));
    if (data.value("schema", "") != "astribot.object_pose/1")
      throw std::runtime_error("INVALID_POSE_SCHEMA");
    const auto rows =
        data.at("camera_from_object").get<std::vector<std::vector<double>>>();
    std::vector<double> transform;
    if (rows.size() != 4)
      throw std::runtime_error("INVALID_POSE_TRANSFORM");
    for (const auto &row : rows) {
      if (row.size() != 4)
        throw std::runtime_error("INVALID_POSE_TRANSFORM");
      transform.insert(transform.end(), row.begin(), row.end());
    }
    for (double v : transform)
      if (!std::isfinite(v))
        throw std::runtime_error("NONFINITE_POSE_OUTPUT");
    if (std::abs(transform[12]) + std::abs(transform[13]) +
                std::abs(transform[14]) + std::abs(transform[15] - 1.) >
            1e-6 ||
        std::abs(transform[3]) > 10 || std::abs(transform[7]) > 10 ||
        transform[11] <= 0 || transform[11] > 10)
      throw std::runtime_error("INVALID_POSE_TRANSFORM");
    double r[9] = {transform[0], transform[1], transform[2],
                   transform[4], transform[5], transform[6],
                   transform[8], transform[9], transform[10]};
    auto &observation = result.observation;
    observation.header = g.header;
    observation.object_id = g.object_id;
    observation.source_camera_id = g.camera_id;
    observation.source_epoch = g.source_epoch;
    observation.source_model = g.model_id;
    observation.model_revision = model.revision;
    if (!model.visibility_revision.empty())
      observation.model_revision += ":" + model.visibility_revision;
    observation.calibration_revision = g.calibration_revision;
    observation.planning_scene_revision = g.planning_scene_revision;
    observation.envelope_epoch = g.envelope_epoch;
    observation.valid_until = g.valid_until;
    observation.pose.pose.orientation = quaternion(r);
    observation.pose.pose.position.x = transform[3];
    observation.pose.pose.position.y = transform[7];
    observation.pose.pose.position.z = transform[11];
    // Registration residual is not calibrated pose covariance. Conservative
    // unknown diagonal.
    for (size_t i = 0; i < 6; ++i)
      observation.pose.covariance[i * 7] = 1e6;
    result.model_coverage = data.at("model_coverage").get<double>();
    result.visibility_model_used = !model.visibility_path.empty();
    result.visible_model_coverage =
        result.visibility_model_used
            ? data.at("visible_model_coverage").get<double>()
            : 0.;
    result.observable_normal_directions =
        data.value("observable_normal_directions", 0u);
    result.scene_coverage = data.at("scene_coverage").get<double>();
    result.rmse_m = data.at("rmse_m").get<double>();
    result.symmetry_equivalent = data.at("symmetry_equivalent").get<bool>();
    if (!std::isfinite(result.model_coverage) ||
        !std::isfinite(result.visible_model_coverage) ||
        result.visible_model_coverage < 0 ||
        result.visible_model_coverage > 1 ||
        !std::isfinite(result.scene_coverage) ||
        !std::isfinite(result.rmse_m) || result.model_coverage < 0 ||
        result.model_coverage > 1 || result.scene_coverage < 0 ||
        result.scene_coverage > 1 || result.rmse_m < 0 ||
        data.at("ambiguous").get<bool>())
      throw std::runtime_error("INVALID_OR_AMBIGUOUS_POSE");
    observation.position_valid = true;
    observation.orientation_valid = true;
    observation.quality = result.scene_coverage;
    observation.reason_code = result.symmetry_equivalent
                                  ? "SYMMETRY_EQUIVALENT_POSE"
                                  : "KNOWN_MODEL_6D_POSE";
    result.success = true;
    result.reason_code = observation.reason_code;
  }
  std::string camera_id_, frame_, grasp_worker_, grasp_model_, grasp_revision_,
      pose_worker_, device_;
  int64_t calibration_, scene_, envelope_;
  double input_age_, result_age_, health_age_;
  std::unordered_map<std::string, Model> models_;
  std::mutex health_mutex_;
  pm::msg::CameraHealth::ConstSharedPtr health_;
  bool require_projection_{false};
  pm::msg::ProjectionHealth::ConstSharedPtr projection_;
  Steady::time_point projection_received_,projection_capture_received_;
  rclcpp::Subscription<pm::msg::ProjectionHealth>::SharedPtr projection_sub_;
  Steady::time_point health_received_;
  Steady::time_point work_started_;
  uint64_t health_generation_{0};
  uint64_t work_generation_{0};
  int64_t last_ros_time_{0};
  std::atomic<bool> stopping_{false}, busy_{false};
  std::thread worker_;
  rclcpp::Subscription<pm::msg::CameraHealth>::SharedPtr health_sub_;
  rclcpp_action::Server<Grasp>::SharedPtr grasp_server_;
  rclcpp_action::Server<Pose>::SharedPtr pose_server_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ManipulationPerceptionServer>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("manipulation_perception_server"),
                 "Startup/runtime failure: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
