#include <atomic>
#include <chrono>
#include <exception>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
using Grid = nav_msgs::msg::OccupancyGrid;
using namespace std::chrono_literals;

rclcpp::QoS latched_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

std::string fingerprint(const Grid & msg)
{
  std::ostringstream out;
  out << msg.info.width << 'x' << msg.info.height << " res="
      << std::fixed << std::setprecision(3) << msg.info.resolution;
  if (msg.data.empty()) {
    out << " 空数据";
    return out.str();
  }
  std::size_t free = 0, occupied = 0, unknown = 0;
  for (const auto value : msg.data) {
    free += value == 0;
    occupied += value >= 65;
    unknown += value < 0;
  }
  const double known = 100.0 * static_cast<double>(msg.data.size() - unknown) /
    static_cast<double>(msg.data.size());
  out << " origin=(" << msg.info.origin.position.x << ", " << msg.info.origin.position.y
      << ") 自由=" << free << " 占据=" << occupied << " 未知=" << unknown
      << " 已知占比=" << std::setprecision(1) << known << '%';
  return out.str();
}

std::shared_ptr<rclcpp::Context> context_for(int argc, char ** argv, int64_t domain)
{
  // rclpy Context.init rejects negative values; casting -1 to size_t would
  // otherwise select RCL_DEFAULT_DOMAIN_ID and silently break domain isolation.
  if (domain < 0) {
    throw std::invalid_argument("domain_id must not be negative");
  }
  auto context = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions options;
  options.set_domain_id(static_cast<std::size_t>(domain));
  context->init(argc, argv, options);
  return context;
}

int relay(int argc, char ** argv)
{
  // Preserve the original bootstrap reader's environment domain, scoped
  // parameter-file/name remaps, then explicitly reconstruct both domains.
  auto reader = std::make_shared<rclcpp::Node>("map_domain_relay_params");
  const auto remote_domain = reader->declare_parameter<int64_t>("remote_domain_id", 25);
  const auto local_domain = reader->declare_parameter<int64_t>("local_domain_id", 25);
  const auto remote_topic = reader->declare_parameter<std::string>("remote_map_topic", "/map");
  const auto local_topic = reader->declare_parameter<std::string>("local_map_topic", "/map");
  const auto timeout = reader->declare_parameter<double>("relay_timeout_sec", 30.0);
  if (remote_domain == local_domain) {
    RCLCPP_ERROR(reader->get_logger(),
      "remote_domain_id 与 local_domain_id 相同(%ld)。同域下不需要中继，拒绝地图回环。",
      static_cast<long>(local_domain));
    return 1;
  }
  reader.reset();
  // Global rclcpp::shutdown() also uninstalls the SIGINT handler. This
  // bootstrap-only shutdown must leave the handler active for both new contexts.
  rclcpp::contexts::get_global_default_context()->shutdown("parameter reader finished");

  auto local_context = context_for(argc, argv, local_domain);
  auto remote_context = context_for(argc, argv, remote_domain);
  auto local_node = std::make_shared<rclcpp::Node>(
    "map_domain_relay_local", rclcpp::NodeOptions().context(local_context));
  auto remote_node = std::make_shared<rclcpp::Node>(
    "map_domain_relay_remote", rclcpp::NodeOptions().context(remote_context));
  auto publisher = local_node->create_publisher<Grid>(local_topic, latched_qos());
  std::size_t count = 0;
  std::string previous_fingerprint;
  auto subscription = remote_node->create_subscription<Grid>(remote_topic, latched_qos(),
    [&](Grid::ConstSharedPtr msg) {
      ++count;
      const auto signature = fingerprint(*msg);
      publisher->publish(*msg);
      if (signature != previous_fingerprint) {
        RCLCPP_INFO(local_node->get_logger(),
          "中继第 %zu 张地图 (domain %ld%s -> domain %ld%s): %s", count,
          static_cast<long>(remote_domain), remote_topic.c_str(),
          static_cast<long>(local_domain), local_topic.c_str(), signature.c_str());
        previous_fingerprint = signature;
      }
    });
  RCLCPP_INFO(local_node->get_logger(),
    "地图中继启动：远端 domain=%ld 话题=%s，本机 domain=%ld 话题=%s，等待远端地图，超时 %.0fs",
    static_cast<long>(remote_domain), remote_topic.c_str(),
    static_cast<long>(local_domain), local_topic.c_str(), timeout);
  rclcpp::ExecutorOptions local_options, remote_options;
  local_options.context = local_context;
  remote_options.context = remote_context;
  rclcpp::executors::SingleThreadedExecutor local_executor(local_options);
  rclcpp::executors::SingleThreadedExecutor remote_executor(remote_options);
  local_executor.add_node(local_node);
  remote_executor.add_node(remote_node);
  std::atomic<bool> stop{false};
  std::thread local_thread([&] {
    try {
      while (!stop.load() && local_context->is_valid()) {
        local_executor.spin_once(100ms);
      }
    } catch (...) {
      if (local_context->is_valid()) {
        // The original daemon thread logs its exception and ends, while the
        // remote loop continues. Preserve this inherited fault policy too.
        RCLCPP_ERROR(local_node->get_logger(), "local executor failed; local spin thread stopped");
      }
    }
  });

  int code = 0;
  std::exception_ptr remote_error;
  try {
    const auto started = std::chrono::system_clock::now();
    while (remote_context->is_valid() && count == 0) {
      remote_executor.spin_once(200ms);
      // SIGINT wakes this wait by shutting down the context. Python instead
      // raises KeyboardInterrupt and skips the timeout comparison entirely.
      if (!remote_context->is_valid()) {
        break;
      }
      // Python uses time.time(), strict > and checks after dispatch, even if
      // that dispatch just delivered the first map. Do not turn this into a
      // rolling timeout or reject inherited zero/negative/NaN/+Inf settings.
      const double elapsed = std::chrono::duration<double>(
        std::chrono::system_clock::now() - started).count();
      if (elapsed > timeout) {
        RCLCPP_ERROR(local_node->get_logger(),
          "%.0fs 内没在 domain %ld 上收到 %s。检查地图源、网络、ROS_LOCALHOST_ONLY 和 TRANSIENT_LOCAL QoS。",
          timeout, static_cast<long>(remote_domain), remote_topic.c_str());
        code = 1;
        break;
      }
    }
    while (code == 0 && remote_context->is_valid()) {
      remote_executor.spin_once(200ms);
    }
  } catch (...) {
    if (remote_context->is_valid()) {
      remote_error = std::current_exception();
    }
  }
  stop.store(true);
  local_executor.cancel();
  local_thread.join();
  if (remote_context->is_valid()) {
    remote_context->shutdown("relay stopped");
  }
  if (local_context->is_valid()) {
    local_context->shutdown("relay stopped");
  }
  if (remote_error) {
    std::rethrow_exception(remote_error);
  }
  return code;
}
}  // namespace

int main(int argc, char ** argv)
{
  int code = 0;
  try {
    // The explicit-context Python process handles SIGINT, while SIGTERM keeps
    // the OS default. Do not silently turn SIGTERM into successful completion.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigInt);
    code = relay(argc, argv);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("map_domain_relay"), "%s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
