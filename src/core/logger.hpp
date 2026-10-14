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

#include <charconv>
#include <cstdint>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include "kairos/log.hpp"

namespace kairos
{

namespace detail
{

// One value, as std::format writes it with an empty spec or ".1f" (the only one Kairos uses);
// not <format>: GCC 11 (RHEL 9, a ROS 2 target) lacks it
template<class T>
void put(std::string & out, std::string_view spec, const T & v)
{
  using U = std::decay_t<T>;
  if constexpr (std::is_same_v<U, bool>) {
    out += v ? "true" : "false";
  } else if constexpr (std::is_floating_point_v<U>) {
    char buf[64];
    auto r = spec == ".1f" ?
      std::to_chars(buf, buf + sizeof(buf), static_cast<double>(v), std::chars_format::fixed, 1) :
      std::to_chars(buf, buf + sizeof(buf), static_cast<double>(v));
    out.append(buf, r.ptr);
  } else if constexpr (std::is_same_v<U, char>) {
    out += v;
  } else if constexpr (std::is_integral_v<U>) {
    char buf[32];
    auto r = std::to_chars(buf, buf + sizeof(buf), v);
    out.append(buf, r.ptr);
  } else if constexpr (requires {to_log_string(v);}) {
    out += to_log_string(v);  // a type of its own, found by argument-dependent lookup
  } else {
    out += std::string_view(v);
  }
}

// The text up to the next placeholder ("{{" and "}}" are braces), then its spec
inline std::string_view literal(std::string & out, std::string_view & fmt)
{
  while (!fmt.empty()) {
    char c = fmt.front();
    if ((c == '{' || c == '}') && fmt.size() > 1 && fmt[1] == c) {
      out += c;
      fmt.remove_prefix(2);
    } else if (c == '{') {
      auto close = fmt.find('}');
      std::string_view inside = fmt.substr(1, close - 1);
      fmt.remove_prefix(close == std::string_view::npos ? fmt.size() : close + 1);
      return inside.empty() ? inside : inside.substr(inside.find(':') + 1);
    } else {
      out += c;
      fmt.remove_prefix(1);
    }
  }
  return {};
}

template<class ... Args>
std::string format(std::string_view fmt, const Args &... args)
{
  std::string out;
  ((put(out, literal(out, fmt), args)), ...);
  literal(out, fmt);
  return out;
}

}  // namespace detail

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
    LogLevel level, LogChannel channel, std::string_view fmt,
    Args &&... args) const noexcept
  {
    if (!enabled(level, channel)) {
      return;
    }
    try {
      deliver(level, channel, detail::format(fmt, args ...));
    } catch (...) {
      // Formatting or the callback failed: drop the message
    }
  }

  template<class ... Args>
  void debug(LogChannel channel, std::string_view fmt, Args &&... args) const noexcept
  {
    log(LogLevel::Debug, channel, fmt, std::forward<Args>(args)...);
  }

  template<class ... Args>
  void info(LogChannel channel, std::string_view fmt, Args &&... args) const noexcept
  {
    log(LogLevel::Info, channel, fmt, std::forward<Args>(args)...);
  }

  template<class ... Args>
  void warning(LogChannel channel, std::string_view fmt, Args &&... args) const noexcept
  {
    log(LogLevel::Warning, channel, fmt, std::forward<Args>(args)...);
  }

  template<class ... Args>
  void error(LogChannel channel, std::string_view fmt, Args &&... args) const noexcept
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
