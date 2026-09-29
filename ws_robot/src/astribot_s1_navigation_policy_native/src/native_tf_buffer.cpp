#include "astribot_s1_navigation_policy_native/native_tf_buffer.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <thread>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace astribot::navigation
{
class NativeTfBuffer::Impl
{
public:
  Impl(bool sim_time, const std::string & requested_name)
  {
    static std::atomic<std::uint64_t> next_id{0};
    name = requested_name.empty() ? "native_tf_buffer_" + std::to_string(::getpid()) +
      "_" + std::to_string(++next_id) : requested_name;
    use_sim_time = sim_time;
    context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init;
    init.shutdown_on_signal = false;
    // Embedded in an initialized rclpy process: its logging lifetime belongs to
    // the host, not to this private receiver context.
    init.auto_initialize_logging(false);
    context->init(0, nullptr, init);  // Inherits this process's domain/RMW environment.
    domain_id = context->get_domain_id();
    rclcpp::NodeOptions options;
    options.context(context).use_global_arguments(false).enable_rosout(false)
    .start_parameter_services(false).start_parameter_event_publisher(false)
    .use_clock_thread(false)
    .parameter_overrides({rclcpp::Parameter("use_sim_time", sim_time)});
    node = std::make_shared<rclcpp::Node>(name, options);
    // All queries below are explicit-source-time, nonblocking BufferCore calls.
    // A local ROS clock correction must not erase source-stamped TF history.
    buffer = std::make_unique<tf2_ros::Buffer>(
      std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), std::chrono::seconds(10));
    // Use one owned executor for both /clock and TF, without hidden listener threads.
    listener = std::make_unique<tf2_ros::TransformListener>(
      *buffer, node, false, tf2_ros::DynamicListenerQoS(100), tf2_ros::StaticListenerQoS(100));
    rcl_jump_threshold_t threshold{};
    threshold.on_clock_change = true;
    threshold.min_backward.nanoseconds = -1;
    jump_handler = node->get_clock()->create_jump_callback(
      nullptr, [this](const rcl_time_jump_t & jump) {
        if (jump.clock_change == RCL_ROS_TIME_NO_CHANGE && jump.delta.nanoseconds < 0)
        {
          // Local clock ordering is diagnostic only. Keep source-stamped TF
          // samples available; session replacement is owned by the caller.
          ++clock_epoch;
        }
      }, threshold);
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor->add_node(node);
    // Construct the thread last. If setup throws, all prior members unwind.
    worker = std::thread([this] {
        running = true;
        try {
          // The stop flag also covers close before the first spin. cancel alone
          // would race with an executor whose spin() has not started yet.
          while (!stop.load() && context->is_valid()) {
            executor->spin_once(std::chrono::milliseconds(50));
          }
        } catch (const std::exception & error) {
          std::lock_guard<std::mutex> lock(error_mutex);
          executor_error = error.what();
        } catch (...) {
          std::lock_guard<std::mutex> lock(error_mutex);
          executor_error = "unknown native TF executor failure";
        }
        running = false;
      });
  }

  ~Impl() {try {close();} catch (...) {}}

  void require_open() const
  {
    if (closed || stop.load()) {
      throw std::runtime_error("native TF buffer is closed");
    }
    std::lock_guard<std::mutex> lock(error_mutex);
    if (!executor_error.empty()) {
      throw std::runtime_error("native TF executor failed: " + executor_error);
    }
  }

  void close()
  {
    // Only callers take lifetime_mutex; the executor and clock callback do not.
    // Thus join cannot wait for a callback blocked on this mutex.
    std::unique_lock<std::shared_mutex> lock(lifetime_mutex);
    if (closed) {return;}
    stop = true;
    std::exception_ptr cleanup_error;
    try {executor->cancel();} catch (...) {cleanup_error = std::current_exception();}
    // spin_once waits at most 50 ms for work even if cancel cannot trigger its
    // guard condition; join also waits for any callback already in progress.
    // Always join before a std::thread member is destroyed.
    if (worker.joinable()) {worker.join();}
    jump_handler.reset();
    listener.reset();
    buffer.reset();
    try {executor->remove_node(node);} catch (...) {
      if (!cleanup_error) {cleanup_error = std::current_exception();}
    }
    executor.reset();
    node.reset();
    try {context->shutdown("native TF buffer closed");} catch (...) {
      if (!cleanup_error) {cleanup_error = std::current_exception();}
    }
    context.reset();
    closed = true;
    if (cleanup_error) {std::rethrow_exception(cleanup_error);}
  }

  mutable std::shared_mutex lifetime_mutex;
  mutable std::mutex error_mutex;
  mutable std::mutex query_mutex;
  std::string name, executor_error;
  std::uint64_t domain_id{0};
  bool use_sim_time{false}, closed{false};
  std::atomic<bool> stop{false}, running{false};
  std::atomic<std::uint64_t> clock_epoch{0};
  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  rclcpp::JumpHandler::SharedPtr jump_handler;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread worker;
};

NativeTfBuffer::NativeTfBuffer(bool sim_time, const std::string & node_name)
: impl_(std::make_unique<Impl>(sim_time, node_name)) {}
NativeTfBuffer::~NativeTfBuffer() = default;

std::pair<bool, std::string> NativeTfBuffer::can_transform(
  const std::string & target, const std::string & source, std::int64_t stamp_ns) const
{
  std::shared_lock<std::shared_mutex> lock(impl_->lifetime_mutex);
  impl_->require_open();
  std::lock_guard<std::mutex> query_lock(impl_->query_mutex);
  std::string error;
  const bool ready = impl_->buffer->tf2::BufferCore::canTransform(
    target, source, tf2::TimePoint(std::chrono::nanoseconds(stamp_ns)), &error);
  return {ready, error};
}

geometry_msgs::msg::TransformStamped NativeTfBuffer::lookup_transform(
  const std::string & target, const std::string & source, std::int64_t stamp_ns) const
{
  std::shared_lock<std::shared_mutex> lock(impl_->lifetime_mutex);
  impl_->require_open();
  std::lock_guard<std::mutex> query_lock(impl_->query_mutex);
  return impl_->buffer->tf2::BufferCore::lookupTransform(
    target, source, tf2::TimePoint(std::chrono::nanoseconds(stamp_ns)));
}

void NativeTfBuffer::clear()
{
  std::shared_lock<std::shared_mutex> lock(impl_->lifetime_mutex);
  impl_->require_open();
  impl_->buffer->clear();
}

void NativeTfBuffer::close() {impl_->close();}

NativeTfBuffer::Diagnostics NativeTfBuffer::diagnostics() const
{
  std::shared_lock<std::shared_mutex> lock(impl_->lifetime_mutex);
  Diagnostics result;
  result.node_name = impl_->name;
  result.domain_id = impl_->domain_id;
  result.use_sim_time = impl_->use_sim_time;
  result.clock_epoch = impl_->clock_epoch.load();
  result.closed = impl_->closed;
  result.executor_running = impl_->running.load();
  if (impl_->node) {result.clock_ns = impl_->node->get_clock()->now().nanoseconds();}
  std::lock_guard<std::mutex> error_lock(impl_->error_mutex);
  result.executor_error = impl_->executor_error;
  return result;
}
}  // namespace astribot::navigation
