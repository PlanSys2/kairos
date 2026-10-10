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

#include "kairos/log.hpp"

#include <array>
#include <cstddef>
#include <string_view>

namespace kairos
{

namespace
{

constexpr std::array<std::string_view, static_cast<size_t>(LogChannel::Count)> kChannelNames = {
  "general", "parse", "semantic", "normalize", "ground", "sas", "search", "heuristic",
  "temporal", "postprocess", "validate", "orchestrator", "memory"};

std::string_view trim(std::string_view s)
{
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
    s.remove_prefix(1);
  }
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) {
    s.remove_suffix(1);
  }
  return s;
}

}  // namespace

std::string_view to_string(LogLevel level)
{
  switch (level) {
    case LogLevel::Debug: return "debug";
    case LogLevel::Info: return "info";
    case LogLevel::Warning: return "warning";
    case LogLevel::Error: return "error";
  }
  return "unknown";
}

std::string_view to_string(LogChannel channel)
{
  auto index = static_cast<size_t>(channel);
  return index < kChannelNames.size() ? kChannelNames[index] : std::string_view("count");
}

std::optional<LogChannel> parse_log_channel(std::string_view name)
{
  for (size_t i = 0; i < kChannelNames.size(); ++i) {
    if (kChannelNames[i] == name) {
      return static_cast<LogChannel>(i);
    }
  }
  return std::nullopt;
}

std::optional<LogChannels> LogChannels::parse(std::string_view text)
{
  text = trim(text);
  if (text == "all") {
    return all();
  }
  if (text.empty()) {
    return std::nullopt;
  }
  LogChannels result;
  while (true) {
    size_t comma = text.find(',');
    auto channel = parse_log_channel(trim(text.substr(0, comma)));
    if (!channel) {
      return std::nullopt;
    }
    result.add(*channel);
    if (comma == std::string_view::npos) {
      return result;
    }
    text = text.substr(comma + 1);
  }
}

}  // namespace kairos
