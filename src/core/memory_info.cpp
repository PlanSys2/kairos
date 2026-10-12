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

#include "core/memory_info.hpp"

#include <sys/resource.h>
#include <unistd.h>

#include <algorithm>
#include <charconv>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

namespace kairos
{

namespace
{

std::string_view trim(std::string_view s)
{
  const auto is_space = [](char c) {return c == ' ' || c == '\t' || c == '\n' || c == '\r';};
  while (!s.empty() && is_space(s.front())) {
    s.remove_prefix(1);
  }
  while (!s.empty() && is_space(s.back())) {
    s.remove_suffix(1);
  }
  return s;
}

std::optional<size_t> parse_size(std::string_view s)
{
  s = trim(s);
  if (s.empty()) {
    return std::nullopt;
  }
  size_t value = 0;
  auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
  if (ec != std::errc() || ptr != s.data() + s.size()) {
    return std::nullopt;
  }
  return value;
}

bool mul_overflows(size_t a, size_t b)
{
  return b != 0 && a > std::numeric_limits<size_t>::max() / b;
}

size_t page_size()
{
  long size = sysconf(_SC_PAGESIZE);  // NOLINT(runtime/int): POSIX API
  return size > 0 ? static_cast<size_t>(size) : size_t{4096};
}

}  // namespace

std::optional<std::string> read_text_file(const std::filesystem::path & path)
{
  std::ifstream in(path);
  if (!in) {
    return std::nullopt;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::optional<size_t> parse_kb_field(std::string_view content, std::string_view key)
{
  size_t pos = 0;
  while (pos < content.size()) {
    size_t end = content.find('\n', pos);
    if (end == std::string_view::npos) {
      end = content.size();
    }
    std::string_view line = content.substr(pos, end - pos);
    pos = end + 1;
    if (!line.starts_with(key)) {
      continue;
    }
    std::string_view rest = trim(line.substr(key.size()));
    size_t multiplier = 1;
    if (rest.ends_with("kB")) {
      multiplier = 1024;
      rest = trim(rest.substr(0, rest.size() - 2));
    }
    auto value = parse_size(rest);
    if (!value || mul_overflows(*value, multiplier)) {
      return std::nullopt;
    }
    return *value * multiplier;
  }
  return std::nullopt;
}

std::optional<size_t> parse_meminfo_available(std::string_view content)
{
  return parse_kb_field(content, "MemAvailable:");
}

std::optional<size_t> parse_cgroup_value(std::string_view content)
{
  content = trim(content);
  if (content == "max") {
    return std::nullopt;
  }
  return parse_size(content);
}

std::optional<std::string> parse_self_cgroup(std::string_view content)
{
  size_t pos = 0;
  while (pos < content.size()) {
    size_t end = content.find('\n', pos);
    if (end == std::string_view::npos) {
      end = content.size();
    }
    std::string_view line = trim(content.substr(pos, end - pos));
    pos = end + 1;
    if (line.starts_with("0::")) {
      return std::string(line.substr(3));
    }
  }
  return std::nullopt;
}

std::optional<size_t> parse_statm_field(std::string_view content, size_t index, size_t page)
{
  std::string_view rest = trim(content);
  for (size_t i = 0; ; ++i) {
    size_t sep = rest.find(' ');
    std::string_view field = rest.substr(0, sep);
    if (i == index) {
      auto pages = parse_size(field);
      if (!pages || mul_overflows(*pages, page)) {
        return std::nullopt;
      }
      return *pages * page;
    }
    if (sep == std::string_view::npos) {
      return std::nullopt;
    }
    rest = trim(rest.substr(sep + 1));
  }
}

std::optional<size_t> cgroup_available(
  const std::filesystem::path & cgroup_root, const std::string & relative)
{
  std::optional<size_t> best;
  std::filesystem::path rel = std::filesystem::path(relative).relative_path();
  // From the process cgroup up to the root, inclusive
  while (true) {
    std::filesystem::path dir = cgroup_root / rel;
    auto max_text = read_text_file(dir / "memory.max");
    auto current_text = read_text_file(dir / "memory.current");
    if (max_text) {
      auto max = parse_cgroup_value(*max_text);
      if (max) {
        size_t current = current_text ? parse_cgroup_value(*current_text).value_or(0) : 0;
        size_t available = current >= *max ? 0 : *max - current;
        best = best ? std::min(*best, available) : available;
      }
    }
    if (rel.empty()) {
      break;
    }
    rel = rel.parent_path();
  }
  return best;
}

MemorySnapshot read_memory_snapshot(const MemoryPaths & paths)
{
  MemorySnapshot snapshot;

  if (auto meminfo = read_text_file(paths.meminfo)) {
    snapshot.mem_available = parse_meminfo_available(*meminfo);
  }

  if (auto self = read_text_file(paths.self_cgroup)) {
    if (auto relative = parse_self_cgroup(*self)) {
      snapshot.cgroup_available = cgroup_available(paths.cgroup_root, *relative);
    }
  }

  rlimit limit{};
  if (getrlimit(RLIMIT_AS, &limit) == 0 && limit.rlim_cur != RLIM_INFINITY) {
    size_t cap = static_cast<size_t>(limit.rlim_cur);
    size_t used = 0;
    if (auto statm = read_text_file(paths.self_statm)) {
      used = parse_statm_field(*statm, 0, page_size()).value_or(0);
    }
    snapshot.rlimit_available = used >= cap ? 0 : cap - used;
  }

  return snapshot;
}

size_t default_memory_limit(const MemorySnapshot & snapshot)
{
  std::optional<size_t> available;
  for (const auto & figure :
    {snapshot.mem_available, snapshot.cgroup_available, snapshot.rlimit_available})
  {
    if (figure) {
      available = available ? std::min(*available, *figure) : *figure;
    }
  }
  return available ? *available / 2 : kFallbackMemoryLimit;
}

std::optional<size_t> peak_rss_bytes(const MemoryPaths & paths)
{
  auto status = read_text_file(paths.self_status);
  if (!status) {
    return std::nullopt;
  }
  return parse_kb_field(*status, "VmHWM:");
}

std::optional<size_t> current_rss_bytes(const MemoryPaths & paths)
{
  auto statm = read_text_file(paths.self_statm);
  if (!statm) {
    return std::nullopt;
  }
  return parse_statm_field(*statm, 1, page_size());
}

}  // namespace kairos
