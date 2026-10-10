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

#include <algorithm>
#include <random>
#include <string>
#include <vector>

#include "kairos/plan.hpp"

namespace kairos
{
namespace
{

std::string dump(const std::vector<Diagnostic> & ds)
{
  std::string out;
  for (const auto & d : ds) {
    out += to_string(d) + "\n";
  }
  return out;
}

Plan read_ok(std::string_view text)
{
  auto r = parse_plan(text);
  EXPECT_TRUE(r.ok()) << dump(r.diagnostics());
  return r.ok() ? std::move(r).value() : Plan{};
}

std::vector<std::string> errors_of(std::string_view text)
{
  auto r = parse_plan(text, "plan.txt");
  EXPECT_FALSE(r.ok());
  std::vector<std::string> out;
  for (const auto & d : r.diagnostics()) {
    out.push_back(d.code + "@" + std::to_string(d.location->line) + ":" +
      std::to_string(d.location->column));
  }
  return out;
}

PlanStep step(double time, double duration, std::string name, std::vector<std::string> args)
{
  return PlanStep{time, duration, std::move(name), std::move(args)};
}

// --- Model -----------------------------------------------------------------------------------

TEST(PlanTest, ActionText)
{
  EXPECT_EQ(step(0, 0, "move", {"r1", "a", "b"}).action(), "(move r1 a b)");
  EXPECT_EQ(step(0, 0, "noop", {}).action(), "(noop)");
}

TEST(PlanTest, Makespan)
{
  Plan empty;
  EXPECT_EQ(empty.makespan(), 0.0);
  Plan temporal{{step(0, 5, "a", {}), step(1, 10, "b", {}), step(5.001, 2, "c", {})}, true, {}};
  EXPECT_DOUBLE_EQ(temporal.makespan(), 11.0);  // not the start of the last step
  Plan sequential{{step(0, 0, "a", {}), step(1, 0, "b", {})}, false, {}};
  EXPECT_EQ(sequential.makespan(), 2.0);
}

// --- Reading ---------------------------------------------------------------------------------

TEST(PlanReadTest, IpcTemporal)
{
  Plan p = read_ok(
    "; a comment\n"
    "0.000: (move r1 kitchen hall) [5.000]\n"
    "\n"
    "5.001: (PICK R1 Cup hall)  [2.5]   ; trailing comment\n");
  EXPECT_TRUE(p.temporal);
  ASSERT_EQ(p.steps.size(), 2u);
  EXPECT_EQ(p.steps[0], step(0, 5, "move", {"r1", "kitchen", "hall"}));
  EXPECT_EQ(p.steps[1].name, "pick");
  EXPECT_EQ(p.steps[1].args, (std::vector<std::string>{"r1", "cup", "hall"}));
  EXPECT_DOUBLE_EQ(p.steps[1].time, 5.001);
  EXPECT_DOUBLE_EQ(p.steps[1].duration, 2.5);
}

TEST(PlanReadTest, TimedWithoutDurations)
{
  // Instantaneous actions at given times (temporal planners write these for snap actions)
  Plan p = read_ok("0.5: (a)\n1.5: (b)\n");
  EXPECT_TRUE(p.temporal);
  EXPECT_EQ(p.steps[1].duration, 0.0);
}

TEST(PlanReadTest, FastDownwardSequential)
{
  Plan p = read_ok("(pick-up a)\n(stack a b)\n; cost = 2 (unit cost)\n");
  EXPECT_FALSE(p.temporal);
  ASSERT_EQ(p.steps.size(), 2u);
  EXPECT_EQ(p.steps[0].time, 0.0);
  EXPECT_EQ(p.steps[1].time, 1.0);
  EXPECT_EQ(p.steps[1].action(), "(stack a b)");
}

TEST(PlanReadTest, NumberedSequential)
{
  // "0: (a)", "1: (b)": positions, not times
  Plan p = read_ok("0: (a x)\n1: (b y)\n2: (c z)\n");
  EXPECT_FALSE(p.temporal);
  EXPECT_EQ(p.steps.size(), 3u);
}

TEST(PlanReadTest, PopfStdout)
{
  Plan p = read_ok(
    "Number of literals: 7\n"
    "Constructing lookup tables: [10%] [20%]\n"
    ";;;; Solution Found\n"
    "; States evaluated: 12\n"
    "; Cost: 10.002\n"
    "0.000: (askcharge leia entrance chargingroom)  [5.000]\n"
    "5.001: (charge leia chargingroom)  [5.000]\n");
  EXPECT_TRUE(p.temporal);
  ASSERT_EQ(p.steps.size(), 2u);
  EXPECT_EQ(p.steps[1].action(), "(charge leia chargingroom)");
}

TEST(PlanReadTest, PopfAnytimeTakesTheLastPlan)
{
  Plan p = read_ok(
    ";;;; Solution Found\n0.000: (slow a)  [10.000]\n"
    ";;;; Solution Found\n0.000: (fast a)  [2.000]\n");
  ASSERT_EQ(p.steps.size(), 1u);
  EXPECT_EQ(p.steps[0].name, "fast");
}

TEST(PlanReadTest, StepsAreSortedByTimeStably)
{
  Plan p = read_ok("2: (c) [1]\n0: (a) [1]\n0: (b) [1]\n");
  ASSERT_EQ(p.steps.size(), 3u);
  EXPECT_EQ(p.steps[0].name, "a");
  EXPECT_EQ(p.steps[1].name, "b");
  EXPECT_EQ(p.steps[2].name, "c");
}

TEST(PlanReadTest, EmptyPlans)
{
  EXPECT_TRUE(read_ok("").steps.empty());
  EXPECT_TRUE(read_ok("; no plan\n\n   \n").steps.empty());
  EXPECT_TRUE(read_ok(";;;; Solution Found\n; Cost: 0\n").steps.empty());
}

TEST(PlanReadTest, LineEndingsAndNumberFormats)
{
  Plan p = read_ok("1e-3: (a) [2E0]\r\n0.5: (b) [.5]\r\n");
  ASSERT_EQ(p.steps.size(), 2u);
  EXPECT_DOUBLE_EQ(p.steps[0].time, 0.001);
  EXPECT_DOUBLE_EQ(p.steps[0].duration, 2.0);
}

TEST(PlanReadTest, MixedLinesAreSequentialWithAWarning)
{
  auto r = parse_plan("0.000: (a) [1]\n(b)\n");
  ASSERT_TRUE(r.ok());
  EXPECT_FALSE(r->temporal);
  ASSERT_EQ(r.diagnostics().size(), 1u);
  EXPECT_EQ(r.diagnostics()[0].code, "plan.mixed_lines");
}

TEST(PlanReadTest, Errors)
{
  EXPECT_EQ(errors_of("move r1 a b\n"), (std::vector<std::string>{"plan.syntax@1:1"}));
  EXPECT_EQ(errors_of("0.0 (a)\n"), (std::vector<std::string>{"plan.syntax@1:5"}));
  EXPECT_EQ(errors_of("0.0: a\n"), (std::vector<std::string>{"plan.syntax@1:6"}));
  EXPECT_EQ(errors_of("(a b\n"), (std::vector<std::string>{"plan.syntax@1:5"}));
  EXPECT_EQ(errors_of("()\n"), (std::vector<std::string>{"plan.syntax@1:2"}));
  EXPECT_EQ(errors_of("(a (b))\n"), (std::vector<std::string>{"plan.syntax@1:4"}));
  EXPECT_EQ(errors_of("0: (a) [x]\n"), (std::vector<std::string>{"plan.syntax@1:9"}));
  EXPECT_EQ(errors_of("0: (a) [1\n"), (std::vector<std::string>{"plan.syntax@1:10"}));
  EXPECT_EQ(errors_of("0: (a) junk\n"), (std::vector<std::string>{"plan.syntax@1:8"}));
  EXPECT_EQ(errors_of("-1: (a)\n"), (std::vector<std::string>{"plan.negative_time@1:1"}));
  EXPECT_EQ(errors_of("0: (a) [-2]\n"),
    (std::vector<std::string>{"plan.negative_duration@1:9"}));
}

TEST(PlanReadTest, ErrorLinesCountFromThePopfMarker)
{
  auto found = errors_of("noise\nmore noise\n;;;; Solution Found\n0.000: (a  [1.000]\n");
  EXPECT_EQ(found, (std::vector<std::string>{"plan.syntax@4:19"}));
}

TEST(PlanReadTest, SeveralErrorsAreReported)
{
  EXPECT_EQ(errors_of("(a)\nbad\n(b)\nworse\n").size(), 2u);
}

// --- Writing ---------------------------------------------------------------------------------

Plan sample_temporal()
{
  return Plan{{step(0, 5, "move", {"r1", "kitchen", "hall"}),
    step(5.001, 2, "pick", {"r1", "cup", "hall"})}, true, 7.001};
}

TEST(PlanWriteTest, IpcTemporal)
{
  EXPECT_EQ(to_string(sample_temporal(), PlanFormat::Ipc),
    "; cost = 7.001\n"
    "; makespan = 7.001\n"
    "0.000: (move r1 kitchen hall) [5.000]\n"
    "5.001: (pick r1 cup hall) [2.000]\n");
}

TEST(PlanWriteTest, IpcSequential)
{
  Plan p{{step(0, 0, "pick-up", {"a"}), step(1, 0, "stack", {"a", "b"})}, false, 2.0};
  EXPECT_EQ(to_string(p, PlanFormat::Ipc), "; cost = 2\n(pick-up a)\n(stack a b)\n");
}

TEST(PlanWriteTest, Precision)
{
  Plan p{{step(1.23456789, 2, "a", {})}, true, {}};
  EXPECT_EQ(to_string(p, PlanFormat::Ipc, 0), "; makespan = 3\n1: (a) [2]\n");
  EXPECT_EQ(to_string(p, PlanFormat::Ipc, 5), "; makespan = 3.23457\n1.23457: (a) [2.00000]\n");
  // Tiny negative values from arithmetic do not print as -0.000
  Plan tiny{{step(-1e-12, 1, "a", {})}, true, {}};
  EXPECT_EQ(to_string(tiny, PlanFormat::Ipc).find("-0"), std::string::npos);
}

TEST(PlanWriteTest, PopfFormatIsWhatThePlanSys2PluginParses)
{
  std::string text = to_string(sample_temporal(), PlanFormat::Popf);
  EXPECT_EQ(text,
    ";;;; Solution Found\n"
    "; Cost: 7.001\n"
    "0.000: (move r1 kitchen hall)  [5.000]\n"
    "5.001: (pick r1 cup hall)  [2.000]\n");
  // The plugin breaks on blank lines (line.front() on an empty string)
  EXPECT_EQ(text.find("\n\n"), std::string::npos);
  // Sequential plans get one time unit per step
  Plan seq{{step(0, 0, "a", {}), step(1, 0, "b", {})}, false, {}};
  EXPECT_EQ(to_string(seq, PlanFormat::Popf),
    ";;;; Solution Found\n0.000: (a)  [0.000]\n1.000: (b)  [0.000]\n");
}

TEST(PlanWriteTest, Json)
{
  EXPECT_EQ(to_string(sample_temporal(), PlanFormat::Json),
    "{\"temporal\":true,\"cost\":7.001,\"makespan\":7.001,\"steps\":["
    "{\"time\":0,\"duration\":5,\"action\":\"(move r1 kitchen hall)\"},"
    "{\"time\":5.001,\"duration\":2,\"action\":\"(pick r1 cup hall)\"}]}\n");
  Plan empty;
  EXPECT_EQ(to_string(empty, PlanFormat::Json),
    "{\"temporal\":false,\"cost\":null,\"makespan\":0,\"steps\":[]}\n");
}

// --- Round trips -----------------------------------------------------------------------------

TEST(PlanRoundTripTest, IpcAndPopfReadBack)
{
  Plan original = sample_temporal();
  for (PlanFormat f : {PlanFormat::Ipc, PlanFormat::Popf}) {
    Plan back = read_ok(to_string(original, f));
    EXPECT_TRUE(back.temporal);
    ASSERT_EQ(back.steps.size(), original.steps.size());
    for (size_t i = 0; i < back.steps.size(); ++i) {
      EXPECT_EQ(back.steps[i].action(), original.steps[i].action());
      EXPECT_NEAR(back.steps[i].time, original.steps[i].time, 1e-9);
      EXPECT_NEAR(back.steps[i].duration, original.steps[i].duration, 1e-9);
    }
  }
}

TEST(PlanRoundTripTest, RandomPlans)
{
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> time(0.0, 1000.0);
  std::uniform_int_distribution<int> count(0, 30);
  for (int iteration = 0; iteration < 200; ++iteration) {
    Plan p;
    p.temporal = true;
    int n = count(rng);
    for (int i = 0; i < n; ++i) {
      p.steps.push_back(step(time(rng), time(rng), "a" + std::to_string(i % 5),
        {"o" + std::to_string(i)}));
    }
    std::stable_sort(p.steps.begin(), p.steps.end(),
      [](const PlanStep & a, const PlanStep & b) {return a.time < b.time;});
    Plan back = read_ok(to_string(p, PlanFormat::Ipc, 9));
    ASSERT_EQ(back.steps.size(), p.steps.size());
    for (size_t i = 0; i < p.steps.size(); ++i) {
      EXPECT_NEAR(back.steps[i].time, p.steps[i].time, 1e-6);
    }
  }
}

TEST(PlanReadTest, LargePlan)
{
  std::string text;
  for (int i = 0; i < 100000; ++i) {
    text += std::to_string(i) + ".000: (move r" + std::to_string(i % 10) + " a b) [1.000]\n";
  }
  Plan p = read_ok(text);
  EXPECT_EQ(p.steps.size(), 100000u);
  EXPECT_DOUBLE_EQ(p.makespan(), 100000.0);
}

}  // namespace
}  // namespace kairos
