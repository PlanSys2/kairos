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

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "core/memory_info.hpp"

namespace kairos
{
namespace
{

namespace fs = std::filesystem;

constexpr size_t KiB = 1024;
constexpr size_t MiB = 1024 * KiB;

// Temporary directory removed at the end of each test
class TempDir
{
public:
  TempDir()
  {
    std::string pattern = (fs::temp_directory_path() / "kairos_test_XXXXXX").string();
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    path_ = mkdtemp(buffer.data());
  }

  ~TempDir() {fs::remove_all(path_);}

  const fs::path & path() const {return path_;}

  void write(const fs::path & relative, const std::string & content) const
  {
    fs::create_directories((path_ / relative).parent_path());
    std::ofstream(path_ / relative) << content;
  }

private:
  fs::path path_;
};

const char * kMeminfo =
  "MemTotal:       65000000 kB\n"
  "MemFree:        10000000 kB\n"
  "MemAvailable:   37000000 kB\n"
  "Buffers:          100000 kB\n";

TEST(MeminfoTest, ParsesMemAvailableInKiB)
{
  EXPECT_EQ(parse_meminfo_available(kMeminfo), KiB * 37000000);
}

TEST(MeminfoTest, AcceptsLastLineWithoutNewlineAndNoUnit)
{
  EXPECT_EQ(parse_meminfo_available("MemTotal: 1 kB\nMemAvailable: 2048"), size_t{2048});
  EXPECT_EQ(parse_meminfo_available("MemAvailable:\t\t5 kB"), 5 * KiB);
}

TEST(MeminfoTest, MissingOrMalformed)
{
  EXPECT_FALSE(parse_meminfo_available("").has_value());
  EXPECT_FALSE(parse_meminfo_available("MemTotal: 1 kB\nMemFree: 2 kB\n").has_value());
  EXPECT_FALSE(parse_meminfo_available("MemAvailable: lots kB\n").has_value());
  EXPECT_FALSE(parse_meminfo_available("MemAvailable: -5 kB\n").has_value());
  EXPECT_FALSE(parse_meminfo_available("MemAvailable: kB\n").has_value());
  EXPECT_FALSE(parse_meminfo_available(" MemAvailable: 5 kB\n").has_value());  // not at start
}

TEST(MeminfoTest, OverflowIsRejected)
{
  std::string huge = "MemAvailable: " + std::to_string(std::numeric_limits<size_t>::max()) +
    " kB\n";
  EXPECT_FALSE(parse_meminfo_available(huge).has_value());
}

TEST(CgroupValueTest, MaxMeansUnlimited)
{
  EXPECT_FALSE(parse_cgroup_value("max").has_value());
  EXPECT_FALSE(parse_cgroup_value("max\n").has_value());
}

TEST(CgroupValueTest, Numbers)
{
  EXPECT_EQ(parse_cgroup_value("536870912\n"), size_t{536870912});
  EXPECT_EQ(parse_cgroup_value("  0  "), size_t{0});
}

TEST(CgroupValueTest, Garbage)
{
  EXPECT_FALSE(parse_cgroup_value("").has_value());
  EXPECT_FALSE(parse_cgroup_value("12abc").has_value());
  EXPECT_FALSE(parse_cgroup_value("maximum").has_value());
}

TEST(SelfCgroupTest, FindsTheV2Line)
{
  EXPECT_EQ(parse_self_cgroup("0::/user.slice/app.scope\n"), "/user.slice/app.scope");
  EXPECT_EQ(
    parse_self_cgroup("12:memory:/old\n0::/a/b\n"), "/a/b");
  EXPECT_EQ(parse_self_cgroup("0::/\n"), "/");
}

TEST(SelfCgroupTest, OnlyV1OrEmpty)
{
  EXPECT_FALSE(parse_self_cgroup("").has_value());
  EXPECT_FALSE(parse_self_cgroup("4:memory:/docker/abc\n").has_value());
}

TEST(StatmTest, ParsesFieldsAsPages)
{
  EXPECT_EQ(parse_statm_field("100 25 3 1 0 50 0\n", 0, 4096), size_t{4096} *100);
  EXPECT_EQ(parse_statm_field("100 25 3 1 0 50 0\n", 1, 4096), size_t{4096} *25);
  EXPECT_EQ(parse_statm_field("100 25 3 1 0 50 0", 6, 4096), size_t{0});
}

TEST(StatmTest, MissingFieldsOrGarbage)
{
  EXPECT_FALSE(parse_statm_field("", 0, 4096).has_value());
  EXPECT_FALSE(parse_statm_field("100", 1, 4096).has_value());
  EXPECT_FALSE(parse_statm_field("abc 1", 0, 4096).has_value());
  std::string huge = std::to_string(std::numeric_limits<size_t>::max()) + " 1";
  EXPECT_FALSE(parse_statm_field(huge, 0, 4096).has_value());
}

TEST(CgroupAvailableTest, TakesTheTightestLevel)
{
  TempDir root;
  // Root: no limit. Parent: 1 GiB with 900 MiB used. Leaf: 512 MiB with 100 MiB used.
  root.write("memory.max", "max\n");
  root.write("parent/memory.max", std::to_string(1024 * MiB));
  root.write("parent/memory.current", std::to_string(900 * MiB));
  root.write("parent/leaf/memory.max", std::to_string(512 * MiB));
  root.write("parent/leaf/memory.current", std::to_string(100 * MiB));
  EXPECT_EQ(cgroup_available(root.path(), "/parent/leaf"), 124 * MiB);
}

TEST(CgroupAvailableTest, LeafLimitWins)
{
  TempDir root;
  root.write("parent/memory.max", "max");
  root.write("parent/leaf/memory.max", std::to_string(256 * MiB));
  root.write("parent/leaf/memory.current", std::to_string(56 * MiB));
  EXPECT_EQ(cgroup_available(root.path(), "/parent/leaf"), 200 * MiB);
}

TEST(CgroupAvailableTest, NoLimitsAnywhere)
{
  TempDir root;
  root.write("a/memory.max", "max");
  EXPECT_FALSE(cgroup_available(root.path(), "/a").has_value());
  EXPECT_FALSE(cgroup_available(root.path(), "/does/not/exist").has_value());
}

TEST(CgroupAvailableTest, OverLimitGivesZeroAndMissingCurrentCountsAsZero)
{
  TempDir root;
  root.write("full/memory.max", "100");
  root.write("full/memory.current", "150");
  EXPECT_EQ(cgroup_available(root.path(), "/full"), size_t{0});

  TempDir other;
  other.write("x/memory.max", "1000");
  EXPECT_EQ(cgroup_available(other.path(), "/x"), size_t{1000});
}

TEST(CgroupAvailableTest, RootCgroupItself)
{
  TempDir root;
  root.write("memory.max", "2048");
  root.write("memory.current", "48");
  EXPECT_EQ(cgroup_available(root.path(), "/"), size_t{2000});
}

TEST(DefaultLimitTest, HalfOfTheSmallestFigure)
{
  MemorySnapshot s;
  s.mem_available = 8000 * MiB;
  s.cgroup_available = 1000 * MiB;
  s.rlimit_available = 4000 * MiB;
  EXPECT_EQ(default_memory_limit(s), 500 * MiB);

  s.cgroup_available.reset();
  EXPECT_EQ(default_memory_limit(s), 2000 * MiB);

  s.rlimit_available.reset();
  EXPECT_EQ(default_memory_limit(s), 4000 * MiB);
}

TEST(DefaultLimitTest, FallbackWhenNothingIsKnown)
{
  EXPECT_EQ(default_memory_limit(MemorySnapshot{}), kFallbackMemoryLimit);
}

TEST(DefaultLimitTest, ZeroAvailableGivesZero)
{
  MemorySnapshot s;
  s.mem_available = 8000 * MiB;
  s.cgroup_available = 0;
  EXPECT_EQ(default_memory_limit(s), size_t{0});
}

TEST(SnapshotTest, ReadsFakeFiles)
{
  TempDir dir;
  dir.write("meminfo", kMeminfo);
  dir.write("cgroup", "0::/robot\n");
  dir.write("sys/robot/memory.max", std::to_string(512 * MiB));
  dir.write("sys/robot/memory.current", "0");
  MemoryPaths paths;
  paths.meminfo = dir.path() / "meminfo";
  paths.self_cgroup = dir.path() / "cgroup";
  paths.cgroup_root = dir.path() / "sys";
  paths.self_statm = dir.path() / "statm";  // missing on purpose
  auto s = read_memory_snapshot(paths);
  EXPECT_EQ(s.mem_available, KiB * 37000000);
  EXPECT_EQ(s.cgroup_available, 512 * MiB);
  EXPECT_EQ(default_memory_limit(s), 256 * MiB);
}

TEST(SnapshotTest, MissingFilesGiveUnknown)
{
  MemoryPaths paths;
  paths.meminfo = "/nonexistent/meminfo";
  paths.self_cgroup = "/nonexistent/cgroup";
  auto s = read_memory_snapshot(paths);
  EXPECT_FALSE(s.mem_available.has_value());
  EXPECT_FALSE(s.cgroup_available.has_value());
}

TEST(SnapshotTest, RealSystem)
{
  // On Linux /proc/meminfo always exists and our own RSS is above zero
  auto s = read_memory_snapshot();
  ASSERT_TRUE(s.mem_available.has_value());
  EXPECT_GT(*s.mem_available, size_t{0});
  EXPECT_GT(default_memory_limit(s), size_t{0});
  auto rss = current_rss_bytes();
  ASSERT_TRUE(rss.has_value());
  EXPECT_GT(*rss, size_t{0});
  auto peak = peak_rss_bytes();
  ASSERT_TRUE(peak.has_value());
  EXPECT_GE(*peak, *rss);
}

TEST(ParseTest, KbFieldsOfProcFiles)
{
  const char * status = "Name:\tkairos\nVmPeak:\t  900 kB\nVmHWM:\t  2048 kB\nVmRSS:\t 1024 kB\n";
  EXPECT_EQ(parse_kb_field(status, "VmHWM:"), size_t{2048 * 1024});
  EXPECT_EQ(parse_kb_field(status, "VmRSS:"), size_t{1024 * 1024});
  EXPECT_FALSE(parse_kb_field(status, "VmSwap:"));
  EXPECT_FALSE(parse_kb_field("VmHWM: lots kB\n", "VmHWM:"));
  EXPECT_EQ(parse_kb_field("Plain: 12\n", "Plain:"), size_t{12});  // without unit: bytes
  EXPECT_FALSE(parse_kb_field("", "VmHWM:"));
}

TEST(PeakRssTest, MissingOrBrokenStatus)
{
  TempDir dir;
  MemoryPaths paths;
  paths.self_status = dir.path() / "none";
  EXPECT_FALSE(peak_rss_bytes(paths));
  dir.write("status", "VmHWM:\t 64 kB\n");
  paths.self_status = dir.path() / "status";
  EXPECT_EQ(peak_rss_bytes(paths), size_t{64 * 1024});
}

TEST(ReadTextFileTest, ExistingAndMissing)
{
  TempDir dir;
  dir.write("f.txt", "hello\nworld");
  EXPECT_EQ(read_text_file(dir.path() / "f.txt"), "hello\nworld");
  EXPECT_FALSE(read_text_file(dir.path() / "missing.txt").has_value());
}

}  // namespace
}  // namespace kairos
