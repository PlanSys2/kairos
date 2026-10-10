// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// \brief Logger: filters by level and channel before formatting anything.

#ifndef KAIROS__CORE__LOGGER_HPP_
#define KAIROS__CORE__LOGGER_HPP_

#include <format>
#include <string>
#include <string_view>
#include <utility>

#include "kairos/log.hpp"

namespace kairos
{

/**
 * @brief Thin wrapper over LogOptions used by every stage.
 *
 * Copies are cheap (the callback is shared by value). A default Logger discards everything.
 * Exceptions thrown by the user callback are swallowed: logging never breaks planning.
 */
class Logger
{
public:
  Logger() = default;
  explicit Logger(LogOptions options)
  : options_(std::move(options)) {}

  /// @brief True if a message with this level and channel would be delivered.
  bool enabled(LogLevel level, LogChannel channel) const noexcept
  {
    if (!options_.callback || level < options_.level) {
      return false;
    }
    return level != LogLevel::Debug || options_.debug_channels.contains(channel);
  }

  template<class ... Args>
  void log(
    LogLevel level, LogChannel channel, std::format_string<Args...> fmt,
    Args &&... args) const noexcept
  {
    if (!enabled(level, channel)) {
      return;
    }
    try {
      deliver(level, channel, std::format(fmt, std::forward<Args>(args)...));
    } catch (...) {
      // Formatting or the callback failed: drop the message
    }
  }

  template<class ... Args>
  void debug(LogChannel channel, std::format_string<Args...> fmt, Args &&... args) const noexcept
  {
    log(LogLevel::Debug, channel, fmt, std::forward<Args>(args)...);
  }

  template<class ... Args>
  void info(LogChannel channel, std::format_string<Args...> fmt, Args &&... args) const noexcept
  {
    log(LogLevel::Info, channel, fmt, std::forward<Args>(args)...);
  }

  template<class ... Args>
  void warning(LogChannel channel, std::format_string<Args...> fmt, Args &&... args) const noexcept
  {
    log(LogLevel::Warning, channel, fmt, std::forward<Args>(args)...);
  }

  template<class ... Args>
  void error(LogChannel channel, std::format_string<Args...> fmt, Args &&... args) const noexcept
  {
    log(LogLevel::Error, channel, fmt, std::forward<Args>(args)...);
  }

  const LogOptions & options() const noexcept {return options_;}

private:
  void deliver(LogLevel level, LogChannel channel, const std::string & message) const
  {
    options_.callback(level, channel, message);
  }

  LogOptions options_;
};

}  // namespace kairos

#endif  // KAIROS__CORE__LOGGER_HPP_
