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
/// \brief Logging options: levels, debug channels and the user callback.

#ifndef KAIROS__LOG_HPP_
#define KAIROS__LOG_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <string_view>

#include "kairos/export.hpp"

namespace kairos
{

enum class LogLevel
{
  Debug,
  Info,
  Warning,
  Error
};

/// @brief Planner stage a message comes from. Debug output can be enabled per channel.
enum class LogChannel : uint8_t
{
  General,
  Parse,
  Semantic,
  Normalize,
  Ground,
  Sas,
  Search,
  Heuristic,
  Temporal,
  Postprocess,
  Validate,
  Orchestrator,
  Memory,
  Count  // number of channels, not a channel
};

/// @brief Receives each message. May be called from the planner's threads.
using LogCallback = std::function<void (LogLevel, LogChannel, std::string_view)>;

/// @brief Set of channels, as a bit mask.
class KAIROS_API LogChannels
{
public:
  constexpr LogChannels() = default;

  static constexpr LogChannels all()
  {
    return LogChannels((uint32_t{1} << static_cast<uint32_t>(LogChannel::Count)) - 1);
  }

  static constexpr LogChannels none() {return LogChannels();}

  constexpr LogChannels & add(LogChannel c)
  {
    bits_ |= bit(c);
    return *this;
  }

  constexpr bool contains(LogChannel c) const {return (bits_ & bit(c)) != 0;}
  constexpr bool empty() const {return bits_ == 0;}
  constexpr bool operator==(const LogChannels & other) const = default;

  /// @brief Parses "all" or a comma separated list such as "search,ground". Empty on error.
  static std::optional<LogChannels> parse(std::string_view text);

private:
  constexpr explicit LogChannels(uint32_t bits)
  : bits_(bits) {}

  static constexpr uint32_t bit(LogChannel c) {return uint32_t{1} << static_cast<uint32_t>(c);}

  uint32_t bits_ = 0;
};

/**
 * @brief What to log and where.
 *
 * Messages at `level` or above are delivered. Debug messages are delivered only for the
 * channels in `debug_channels` (and only if `level` is Debug).
 */
struct LogOptions
{
  LogCallback callback;
  LogLevel level = LogLevel::Info;
  LogChannels debug_channels = LogChannels::all();
};

/// @brief "debug", "info", "warning" or "error".
KAIROS_API std::string_view to_string(LogLevel level);

/// @brief Lower-case channel name, e.g. "search". "count" for LogChannel::Count.
KAIROS_API std::string_view to_string(LogChannel channel);

/// @brief Inverse of to_string(LogChannel).
KAIROS_API std::optional<LogChannel> parse_log_channel(std::string_view name);

}  // namespace kairos

#endif  // KAIROS__LOG_HPP_
