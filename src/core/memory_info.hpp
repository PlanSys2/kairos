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
/// \brief System memory queries (Linux /proc and cgroup v2) and the default memory limit.

#ifndef KAIROS__CORE__MEMORY_INFO_HPP_
#define KAIROS__CORE__MEMORY_INFO_HPP_

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace kairos
{

/// @brief Where to read from. Tests point these to fake files.
struct MemoryPaths
{
  std::filesystem::path meminfo = "/proc/meminfo";
  std::filesystem::path self_cgroup = "/proc/self/cgroup";
  std::filesystem::path cgroup_root = "/sys/fs/cgroup";
  std::filesystem::path self_statm = "/proc/self/statm";
  std::filesystem::path self_status = "/proc/self/status";
};

/// @brief Available memory according to each source, in bytes. Missing = unknown or unlimited.
struct MemorySnapshot
{
  std::optional<size_t> mem_available;     // MemAvailable
  std::optional<size_t> cgroup_available;  // min over cgroup levels of max - current
  std::optional<size_t> rlimit_available;  // RLIMIT_AS - address space in use
};

/// @brief Used when no source can be read.
inline constexpr size_t kFallbackMemoryLimit = size_t{1} << 30;

/// @brief Bytes of "MemAvailable:" in /proc/meminfo content.
std::optional<size_t> parse_meminfo_available(std::string_view content);

/// @brief Value of a cgroup memory file ("max" or a byte count). "max" -> nullopt.
std::optional<size_t> parse_cgroup_value(std::string_view content);

/// @brief cgroup v2 path ("0::/a/b" line) of /proc/self/cgroup content, e.g. "/a/b".
std::optional<std::string> parse_self_cgroup(std::string_view content);

/// @brief Field `index` (0 = size, 1 = resident) of /proc/self/statm, converted to bytes.
std::optional<size_t> parse_statm_field(std::string_view content, size_t index, size_t page_size);

/// @brief Smallest max - current over the cgroup at `relative` and all its ancestors.
std::optional<size_t> cgroup_available(
  const std::filesystem::path & cgroup_root, const std::string & relative);

/// @brief Reads all the sources.
MemorySnapshot read_memory_snapshot(const MemoryPaths & paths = {});

/// @brief Half of the smallest known available figure, or kFallbackMemoryLimit.
size_t default_memory_limit(const MemorySnapshot & snapshot);

/// @brief A "Key:   123 kB" line of /proc files (meminfo, status), in bytes.
std::optional<size_t> parse_kb_field(std::string_view content, std::string_view key);

/// @brief Highest resident set size of this process so far (VmHWM), if readable.
std::optional<size_t> peak_rss_bytes(const MemoryPaths & paths = {});

/// @brief Resident set size of this process, if readable.
std::optional<size_t> current_rss_bytes(const MemoryPaths & paths = {});

/// @brief Reads a whole small file; nullopt if it cannot be opened.
std::optional<std::string> read_text_file(const std::filesystem::path & path);

}  // namespace kairos

#endif  // KAIROS__CORE__MEMORY_INFO_HPP_
