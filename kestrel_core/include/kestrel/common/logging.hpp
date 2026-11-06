// SPDX-License-Identifier: Apache-2.0
//
// kestrel/common/logging.hpp — a tiny, dependency-free logging facade.
//
// TUTORIAL NOTE
// -------------
// `kestrel_core` must not depend on ROS (see docs/DEV_PLAN.md, "Design rules"), yet when it runs
// inside the ROS node we want its messages to show up in `rclcpp` logging (with timestamps, the
// node name, /rosout, ...). The classic solution is a *logging facade*: the library writes to an
// abstract sink, and whoever embeds the library decides where the messages go.
//
//   * Standalone tools / tests  → the default sink prints to stderr.
//   * kestrel_ros               → installs a sink that forwards to RCLCPP_INFO & friends.
//
// Usage:
//
//   KLOG_INFO("tracked " << n << " features in " << ms << " ms");
//   KLOG_WARN("IMU gap of " << dt << " s");
//
// The macros take a *stream expression*, so formatting is only performed when the level is
// enabled (cheap when debug logging is off).

#pragma once

#include <functional>
#include <sstream>
#include <string>

namespace kestrel {

/// Severity levels, ordered from most to least verbose.
enum class LogLevel { kDebug = 0, kInfo = 1, kWarn = 2, kError = 3 };

/// Signature of a log sink: receives the level, the source location and the formatted message.
using LogSink = std::function<void(LogLevel level, const char* file, int line, const std::string&)>;

/// Replace the global sink (thread-safe). Passing an empty function restores the stderr sink.
void setLogSink(LogSink sink);

/// Messages below this level are dropped before formatting. Default: kInfo.
void setLogLevel(LogLevel level);
LogLevel logLevel();

/// Human-readable name of a level ("DEBUG", "INFO", ...).
const char* toString(LogLevel level);

namespace detail {
/// Dispatches a fully-formatted message to the current sink. Use the KLOG_* macros instead.
void emitLog(LogLevel level, const char* file, int line, const std::string& message);
}  // namespace detail

}  // namespace kestrel

// The do { } while (0) wrapper makes each macro behave like a single statement, so it is safe in
// un-braced if/else branches.
#define KLOG_AT(level, expr)                                                             \
  do {                                                                                   \
    if (static_cast<int>(level) >= static_cast<int>(::kestrel::logLevel())) {            \
      std::ostringstream kestrel_log_stream__;                                           \
      kestrel_log_stream__ << expr;                                                      \
      ::kestrel::detail::emitLog(level, __FILE__, __LINE__, kestrel_log_stream__.str()); \
    }                                                                                    \
  } while (0)

#define KLOG_DEBUG(expr) KLOG_AT(::kestrel::LogLevel::kDebug, expr)
#define KLOG_INFO(expr) KLOG_AT(::kestrel::LogLevel::kInfo, expr)
#define KLOG_WARN(expr) KLOG_AT(::kestrel::LogLevel::kWarn, expr)
#define KLOG_ERROR(expr) KLOG_AT(::kestrel::LogLevel::kError, expr)
