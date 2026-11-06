// SPDX-License-Identifier: Apache-2.0
//
// kestrel_node — the ROS 2 front door of Kestrel SLAM.
//
// STATUS (M0): this node only wires up the plumbing that every later milestone relies on:
//   * reads the `config_file` parameter and loads it with kestrel::loadConfig(),
//   * routes kestrel_core's log messages into rclcpp logging (see kestrel/common/logging.hpp),
//   * advertises the map services (they answer "not implemented yet" until M6).
// Sensor subscriptions and outputs arrive in M9 (see docs/DEV_PLAN.md).
//
// TUTORIAL NOTE — why the core library is not a ROS node itself
// --------------------------------------------------------------
// Everything algorithmic lives in kestrel_core, which knows nothing about ROS. This node is a
// thin adapter: it converts ROS messages into plain C++ types, calls the library, and converts the
// results back. The payoff: the identical SLAM code runs in unit tests and offline dataset tools
// (no ROS required), and porting to another middleware would only mean writing another adapter.

#include <memory>
#include <string>

#include "kestrel/common/logging.hpp"
#include "kestrel/config.hpp"
#include "kestrel/version.hpp"
#include "kestrel_interfaces/srv/load_map.hpp"
#include "kestrel_interfaces/srv/reset_map.hpp"
#include "kestrel_interfaces/srv/save_map.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kestrel_ros {

class KestrelNode : public rclcpp::Node {
 public:
  explicit KestrelNode(const rclcpp::NodeOptions& options) : rclcpp::Node("kestrel", options) {
    installLogSink();

    const std::string config_file = declare_parameter<std::string>("config_file", "");
    try {
      config_ = config_file.empty() ? kestrel::Config{} : kestrel::loadConfig(config_file);
    } catch (const kestrel::ConfigError& e) {
      RCLCPP_FATAL(get_logger(), "%s", e.what());
      throw;
    }
    RCLCPP_INFO(get_logger(), "Kestrel SLAM %s — config: %s", KESTREL_VERSION,
                config_file.empty() ? "<built-in defaults>" : config_file.c_str());
    RCLCPP_DEBUG(get_logger(), "Resolved configuration:\n%s", kestrel::toYaml(config_).c_str());

    using namespace kestrel_interfaces::srv;
    save_srv_ = create_service<SaveMap>(
        "~/save_map", [](const SaveMap::Request::SharedPtr, SaveMap::Response::SharedPtr res) {
          res->success = false;
          res->message = "map saving is not implemented yet (milestone M6)";
        });
    load_srv_ = create_service<LoadMap>(
        "~/load_map", [](const LoadMap::Request::SharedPtr, LoadMap::Response::SharedPtr res) {
          res->success = false;
          res->message = "map loading is not implemented yet (milestone M6)";
        });
    reset_srv_ = create_service<ResetMap>(
        "~/reset", [](const ResetMap::Request::SharedPtr, ResetMap::Response::SharedPtr res) {
          res->success = false;
          res->message = "reset is not implemented yet (milestone M3)";
        });
  }

  ~KestrelNode() override { kestrel::setLogSink({}); }

 private:
  /// Forward kestrel_core log messages to this node's rclcpp logger.
  void installLogSink() {
    const rclcpp::Logger logger = get_logger().get_child("core");
    kestrel::setLogSink(
        [logger](kestrel::LogLevel level, const char*, int, const std::string& msg) {
          switch (level) {
            case kestrel::LogLevel::kDebug:
              RCLCPP_DEBUG(logger, "%s", msg.c_str());
              break;
            case kestrel::LogLevel::kInfo:
              RCLCPP_INFO(logger, "%s", msg.c_str());
              break;
            case kestrel::LogLevel::kWarn:
              RCLCPP_WARN(logger, "%s", msg.c_str());
              break;
            case kestrel::LogLevel::kError:
              RCLCPP_ERROR(logger, "%s", msg.c_str());
              break;
          }
        });
  }

  kestrel::Config config_;
  rclcpp::Service<kestrel_interfaces::srv::SaveMap>::SharedPtr save_srv_;
  rclcpp::Service<kestrel_interfaces::srv::LoadMap>::SharedPtr load_srv_;
  rclcpp::Service<kestrel_interfaces::srv::ResetMap>::SharedPtr reset_srv_;
};

}  // namespace kestrel_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kestrel_ros::KestrelNode)
