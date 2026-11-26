// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the logging facade declared in kestrel/common/logging.hpp.

#include "kestrel/common/logging.hpp"

#include <atomic>
#include <iostream>
#include <mutex>

namespace kestrel {
namespace {

// The sink can be swapped while other threads log, so it is guarded by a mutex. Logging is not on
// any hot path that matters (we never log per feature), so a mutex is perfectly adequate.
std::mutex g_sink_mutex;
LogSink g_sink;  // empty → use the default stderr sink
std::atomic<int> g_level{static_cast<int>(LogLevel::kInfo)};

void defaultSink(LogLevel level, const char* /*file*/, int /*line*/, const std::string& msg) {
  std::cerr << "[kestrel] [" << toString(level) << "] " << msg << '\n';
}

}  // namespace

void setLogSink(LogSink sink) {
  std::lock_guard<std::mutex> lock(g_sink_mutex);
  g_sink = std::move(sink);
}

void setLogLevel(LogLevel level) {
  g_level.store(static_cast<int>(level));
}

LogLevel logLevel() {
  return static_cast<LogLevel>(g_level.load());
}

const char* toString(LogLevel level) {
  switch (level) {
    case LogLevel::kDebug:
      return "DEBUG";
    case LogLevel::kInfo:
      return "INFO";
    case LogLevel::kWarn:
      return "WARN";
    case LogLevel::kError:
      return "ERROR";
  }
  return "?";
}

namespace detail {
void emitLog(LogLevel level, const char* file, int line, const std::string& message) {
  std::lock_guard<std::mutex> lock(g_sink_mutex);
  if (g_sink) {
    g_sink(level, file, line, message);
  } else {
    defaultSink(level, file, line, message);
  }
}
}  // namespace detail

}  // namespace kestrel
