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
#include <sys/wait.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "kairos/kairos.hpp"

namespace
{

namespace fs = std::filesystem;

const char * kDomain =
  "(define (domain robots)\n"
  " (:requirements :strips :typing)\n"
  " (:types robot room)\n"
  " (:predicates (at ?r - robot ?l - room) (connected ?a ?b - room))\n"
  " (:action move :parameters (?r - robot ?a ?b - room)\n"
  "  :precondition (and (at ?r ?a) (connected ?a ?b))\n"
  "  :effect (and (not (at ?r ?a)) (at ?r ?b))))\n";

const char * kProblem =
  "(define (problem p) (:domain robots)\n"
  " (:objects r1 - robot kitchen corridor hall attic - room)\n"
  " (:init (at r1 kitchen) (connected kitchen corridor) (connected corridor hall))\n"
  " (:goal (at r1 hall)))\n";

const char * kUnsolvable =
  "(define (problem u) (:domain robots)\n"
  " (:objects r1 - robot kitchen attic - room)\n"
  " (:init (at r1 kitchen)) (:goal (at r1 attic)))\n";

const char * kTimed =
  "(define (domain timed) (:requirements :strips :durative-actions) (:predicates (p) (q))\n"
  " (:durative-action a :parameters () :duration (= ?duration 1)\n"
  "  :condition (at start (p)) :effect (at end (q))))\n";

const char * kTimedProblem = "(define (problem x) (:domain timed) (:init (p)) (:goal (q)))\n";

const char * kBroken = "(define (domain robots)\n (:predicates (at ?r)\n";

// Twelve pigeons, eleven holes: unsolvable and slow to prove
std::string holes(int pigeons, int n)
{
  std::string objects, init, goal;
  for (int i = 0; i < pigeons; ++i) {
    objects += " p" + std::to_string(i);
    init += " (out p" + std::to_string(i) + ")";
    goal += " (in p" + std::to_string(i) + ")";
  }
  objects += " - pigeon";
  for (int i = 0; i < n; ++i) {
    objects += " h" + std::to_string(i);
    init += " (free h" + std::to_string(i) + ")";
  }
  return "(define (problem x) (:domain holes) (:objects" + objects + " - hole) (:init" + init +
         ") (:goal (and" + goal + ")))";
}

const char * kHoles =
  "(define (domain holes) (:requirements :strips :typing) (:types pigeon hole)\n"
  " (:predicates (out ?p - pigeon) (in ?p - pigeon) (free ?h - hole))\n"
  " (:action put :parameters (?p - pigeon ?h - hole) :precondition (and (out ?p) (free ?h))\n"
  "  :effect (and (not (out ?p)) (in ?p) (not (free ?h)))))\n";

struct Outcome
{
  int code = -1;
  std::string out;
  std::string err;
};

class CliTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = fs::temp_directory_path() /
      ("kairos_cli_test_" + std::string(
        ::testing::UnitTest::GetInstance()->current_test_info()->name()));
    fs::remove_all(dir_);
    fs::create_directories(dir_);
    write("domain.pddl", kDomain);
    write("problem.pddl", kProblem);
  }

  void TearDown() override {fs::remove_all(dir_);}

  std::string path(const std::string & name) const {return (dir_ / name).string();}

  std::string write(const std::string & name, const std::string & text) const
  {
    std::ofstream(dir_ / name) << text;
    return path(name);
  }

  static std::string slurp(const fs::path & file)
  {
    std::ifstream in(file);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
  }

  // Runs the executable with these arguments; `prefix` goes before it (e.g. a timeout command)
  Outcome run(const std::string & args, const std::string & prefix = "") const
  {
    std::string err_file = path("stderr.txt");
    std::string cmd = prefix + " '" + std::string(KAIROS_CLI) + "' " + args + " 2>'" +
      err_file + "'";
    Outcome r;
    FILE * pipe = popen(cmd.c_str(), "r");
    if (pipe == nullptr) {
      return r;
    }
    char buffer[4096];
    size_t n;
    while ((n = fread(buffer, 1, sizeof(buffer), pipe)) > 0) {
      r.out.append(buffer, n);
    }
    int status = pclose(pipe);
    r.code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    r.err = slurp(err_file);
    return r;
  }

  std::string solve_args() const
  {
    return "'" + path("domain.pddl") + "' '" + path("problem.pddl") +
           "'";
  }

  fs::path dir_;
};

std::vector<std::string> lines(const std::string & text)
{
  std::vector<std::string> out;
  std::stringstream ss(text);
  for (std::string l; std::getline(ss, l); ) {
    out.push_back(l);
  }
  return out;
}

bool contains(const std::string & text, const std::string & what)
{
  return text.find(what) != std::string::npos;
}

// --- solve ------------------------------------------------------------------------------------

TEST_F(CliTest, SolvePrintsOnlyThePlanOnStdout)
{
  Outcome r = run(solve_args());
  EXPECT_EQ(r.code, 0) << r.err;
  EXPECT_EQ(r.out, "; cost = 2\n(move r1 kitchen corridor)\n(move r1 corridor hall)\n");
  EXPECT_TRUE(r.err.empty()) << r.err;
  // "solve" is the default command
  EXPECT_EQ(run("solve " + solve_args()).out, r.out);
}

TEST_F(CliTest, SearchPresets)
{
  for (const char * name : {"auto", "ff", "lama", "eager-lama", "novelty", "lama-novelty",
      "ehc"})
  {
    Outcome r = run(solve_args() + " --search " + name);
    EXPECT_EQ(r.code, 0) << name << ": " << r.err;
    EXPECT_EQ(r.out, "; cost = 2\n(move r1 kitchen corridor)\n(move r1 corridor hall)\n") << name;
  }
}

TEST_F(CliTest, ThePrintedPlanValidates)
{
  Outcome r = run(solve_args() + " --no-anytime --output '" + path("plan.txt") + "'");
  ASSERT_EQ(r.code, 0) << r.err;
  EXPECT_TRUE(r.out.empty());
  Outcome v = run("validate " + solve_args() + " '" + path("plan.txt") + "'");
  EXPECT_EQ(v.code, 0) << v.err;
  EXPECT_TRUE(contains(v.err, "plan valid"));
}

TEST_F(CliTest, AnytimeWritesNumberedFiles)
{
  Outcome r = run(solve_args() + " --output '" + path("plan") + "'");
  ASSERT_EQ(r.code, 0) << r.err;
  EXPECT_TRUE(fs::exists(dir_ / "plan.1"));
  EXPECT_FALSE(fs::exists(dir_ / "plan"));
  EXPECT_EQ(slurp(dir_ / "plan.1"),
    "; cost = 2\n(move r1 kitchen corridor)\n(move r1 corridor hall)\n");
}

TEST_F(CliTest, PopfFormatHasTheMarkerAndNoBlankLines)
{
  Outcome r = run(solve_args() + " --format popf");
  ASSERT_EQ(r.code, 0) << r.err;
  auto ls = lines(r.out);
  ASSERT_GE(ls.size(), 3u);
  EXPECT_EQ(ls[0], ";;;; Solution Found");
  for (const auto & l : ls) {
    EXPECT_FALSE(l.empty());
  }
  EXPECT_TRUE(contains(r.out, "0.000: (move r1 kitchen corridor)  [0.000]\n"));
  EXPECT_TRUE(contains(r.out, "1.000: (move r1 corridor hall)  [0.000]\n"));
  // Our own plan parser reads it back
  auto plan = kairos::parse_plan(r.out);
  ASSERT_TRUE(plan.ok());
  EXPECT_EQ(plan->steps.size(), 2u);
}

TEST_F(CliTest, PopfFormatPrintsOnlyTheBestOfAnAnytimeSearch)
{
  // BLOCKS-5-1 improves twice; a POPF reader sees a single plan
  write(
    "blocks.pddl",
    "(define (domain blocks) (:requirements :strips)\n"
    " (:predicates (on ?x ?y) (ontable ?x) (clear ?x) (handempty) (holding ?x))\n"
    " (:action pick-up :parameters (?x) :precondition (and (clear ?x) (ontable ?x) (handempty))\n"
    "  :effect (and (not (ontable ?x)) (not (clear ?x)) (not (handempty)) (holding ?x)))\n"
    " (:action put-down :parameters (?x) :precondition (holding ?x)\n"
    "  :effect (and (not (holding ?x)) (clear ?x) (handempty) (ontable ?x)))\n"
    " (:action stack :parameters (?x ?y) :precondition (and (holding ?x) (clear ?y))\n"
    "  :effect (and (not (holding ?x)) (not (clear ?y)) (clear ?x) (handempty) (on ?x ?y)))\n"
    " (:action unstack :parameters (?x ?y) :precondition (and (on ?x ?y) (clear ?x) (handempty))\n"
    "  :effect (and (holding ?x) (clear ?y) (not (clear ?x)) (not (handempty))"
    " (not (on ?x ?y)))))");
  write(
    "b5.pddl",
    "(define (problem b) (:domain blocks) (:objects a d c e b)\n"
    " (:init (clear b) (clear e) (clear c) (ontable d) (ontable e) (ontable c) (on b a)\n"
    "  (on a d) (handempty)) (:goal (and (on d c) (on c b) (on b a) (on a e))))");
  std::string args = "'" + path("blocks.pddl") + "' '" + path("b5.pddl") + "'";
  Outcome ipc = run(args);
  ASSERT_EQ(ipc.code, 0) << ipc.err;
  EXPECT_TRUE(contains(ipc.out, "; better plan"));
  Outcome popf = run(args + " --format popf");
  ASSERT_EQ(popf.code, 0) << popf.err;
  size_t first = popf.out.find(";;;; Solution Found");
  ASSERT_NE(first, std::string::npos);
  EXPECT_EQ(popf.out.find(";;;; Solution Found", first + 1), std::string::npos);
  EXPECT_TRUE(contains(popf.out, "; Cost: 10.000"));
}

TEST_F(CliTest, JsonFormatIsOneObjectPerPlan)
{
  Outcome r = run(solve_args() + " --format json");
  ASSERT_EQ(r.code, 0) << r.err;
  EXPECT_EQ(
    r.out,
    "{\"temporal\":false,\"cost\":2,\"makespan\":2,\"steps\":["
    "{\"time\":0,\"duration\":0,\"action\":\"(move r1 kitchen corridor)\"},"
    "{\"time\":1,\"duration\":0,\"action\":\"(move r1 corridor hall)\"}]}\n");
}

TEST_F(CliTest, VerboseReportsOnStderrOnly)
{
  Outcome r = run(solve_args() + " -v");
  ASSERT_EQ(r.code, 0);
  EXPECT_EQ(lines(r.out).size(), 3u);
  EXPECT_TRUE(contains(r.err, "[orchestrator]"));
  EXPECT_TRUE(contains(r.err, "kairos: solved (proven optimal), cost 2"));
  EXPECT_TRUE(contains(r.err, "expanded"));
}

TEST_F(CliTest, DebugChannels)
{
  Outcome all = run(solve_args() + " --debug");
  ASSERT_EQ(all.code, 0);
  EXPECT_TRUE(contains(all.err, "[search]"));
  Outcome some = run(solve_args() + " --debug=ground");
  ASSERT_EQ(some.code, 0);
  EXPECT_TRUE(contains(some.err, "[ground]"));
  Outcome bogus = run(solve_args() + " --debug=bogus");
  EXPECT_EQ(bogus.code, 2);
  EXPECT_TRUE(contains(bogus.err, "unknown debug channel"));
  EXPECT_TRUE(bogus.out.empty());
}

TEST_F(CliTest, DumpsToADirectory)
{
  fs::create_directories(dir_ / "dump");
  Outcome r = run(solve_args() + " --dump=sas,ground --dump-dir '" + path("dump") + "'");
  ASSERT_EQ(r.code, 0) << r.err;
  EXPECT_TRUE(fs::exists(dir_ / "dump" / "sas.txt"));
  EXPECT_TRUE(fs::exists(dir_ / "dump" / "ground.txt"));
  EXPECT_FALSE(fs::exists(dir_ / "dump" / "lifted.txt"));
  Outcome all = run(solve_args() + " --dump --dump-dir '" + path("dump") + "'");
  ASSERT_EQ(all.code, 0) << all.err;
  EXPECT_TRUE(fs::exists(dir_ / "dump" / "lifted.txt"));
  EXPECT_TRUE(fs::exists(dir_ / "dump" / "plan-trace.txt"));
  EXPECT_EQ(run(solve_args() + " --dump=nonsense").code, 2);
}

// --- exit codes -------------------------------------------------------------------------------

TEST_F(CliTest, UnsolvableExitsTen)
{
  write("u.pddl", kUnsolvable);
  Outcome r = run("'" + path("domain.pddl") + "' '" + path("u.pddl") + "'");
  EXPECT_EQ(r.code, 10);
  EXPECT_TRUE(r.out.empty());
  EXPECT_TRUE(contains(r.err, "unsolvable"));
}

TEST_F(CliTest, TimeoutExitsEleven)
{
  write("h.pddl", kHoles);
  write("hp.pddl", holes(12, 11));
  Outcome r = run("'" + path("h.pddl") + "' '" + path("hp.pddl") + "' --timeout 0.2");
  EXPECT_EQ(r.code, 11);
  EXPECT_TRUE(r.out.empty());
  EXPECT_TRUE(contains(r.err, "timeout"));
}

TEST_F(CliTest, MemoryLimitExitsTwelve)
{
  write("h.pddl", kHoles);
  write("hp.pddl", holes(20, 19));  // fills 8 MB fast, even in coverage builds
  Outcome r = run(
    "'" + path("h.pddl") + "' '" + path("hp.pddl") + "' --memory 8 --timeout 60 --search ff");
  EXPECT_EQ(r.code, 12);
  EXPECT_TRUE(contains(r.err, "out of memory"));
}

TEST_F(CliTest, UnsupportedExitsThirteen)
{
  write("t.pddl", kTimed);
  write("tp.pddl", kTimedProblem);
  Outcome r = run("'" + path("t.pddl") + "' '" + path("tp.pddl") + "'");
  EXPECT_EQ(r.code, 13);
  EXPECT_TRUE(contains(r.err, "unsupported"));
  EXPECT_EQ(run(solve_args() + " --engine m2").code, 13);
}

TEST_F(CliTest, InvalidInputExitsTwo)
{
  write("broken.pddl", kBroken);
  Outcome r = run("'" + path("broken.pddl") + "' '" + path("problem.pddl") + "'");
  EXPECT_EQ(r.code, 2);
  EXPECT_TRUE(r.out.empty());
  // The error points at the file and shows the line
  EXPECT_TRUE(contains(r.err, path("broken.pddl") + ":"));
  // POPF readers get 0 and no marker
  Outcome popf = run("'" + path("broken.pddl") + "' '" + path("problem.pddl") + "' --format popf");
  EXPECT_EQ(popf.code, 0);
  EXPECT_FALSE(contains(popf.out, "Solution Found"));

  EXPECT_EQ(run("'" + path("nope.pddl") + "' '" + path("problem.pddl") + "'").code, 2);
  EXPECT_EQ(run(solve_args() + " --timeout").code, 2);         // missing value
  EXPECT_EQ(run(solve_args() + " --timeout abc").code, 2);     // bad value
  EXPECT_EQ(run(solve_args() + " --timeout -1").code, 2);
  EXPECT_EQ(run(solve_args() + " --timeot 5").code, 2);        // unknown option
  EXPECT_EQ(run(solve_args() + " --no-anytime=1").code, 2);    // a flag with a value
  EXPECT_EQ(run(solve_args() + " --format xml").code, 2);
  EXPECT_EQ(run(solve_args() + " --engine m9").code, 2);
  EXPECT_EQ(run(solve_args() + " --search bogus").code, 2);
  EXPECT_EQ(run("'" + path("domain.pddl") + "'").code, 2);     // one file
  EXPECT_EQ(run("").code, 2);
}

TEST_F(CliTest, InterruptStopsTheSearch)
{
  write("h.pddl", kHoles);
  write("hp.pddl", holes(12, 11));
  Outcome r = run(
    "'" + path("h.pddl") + "' '" + path("hp.pddl") + "' --timeout 60",
    "timeout --preserve-status -s INT 0.3");
  EXPECT_EQ(r.code, 1);  // cancelled without a plan
  EXPECT_TRUE(contains(r.err, "cancelled"));
}

TEST_F(CliTest, HelpAndVersion)
{
  Outcome h = run("--help");
  EXPECT_EQ(h.code, 0);
  EXPECT_TRUE(contains(h.out, "Usage:"));
  EXPECT_EQ(run("solve --help").code, 0);
  Outcome v = run("--version");
  EXPECT_EQ(v.code, 0);
  EXPECT_EQ(v.out, "kairos " + std::string(kairos::version()) + "\n");
}

// --- check and validate -----------------------------------------------------------------------

TEST_F(CliTest, CheckVariants)
{
  Outcome d = run("check '" + path("domain.pddl") + "'");
  EXPECT_EQ(d.code, 0);
  EXPECT_TRUE(contains(d.err, "valid, plannable"));
  EXPECT_TRUE(contains(d.err, "features:"));
  EXPECT_TRUE(d.out.empty());

  Outcome p = run("check " + solve_args());
  EXPECT_EQ(p.code, 0);
  EXPECT_TRUE(contains(p.err, "valid, plannable"));

  Outcome alone = run("check --problem '" + path("problem.pddl") + "'");
  EXPECT_EQ(alone.code, 0);
  EXPECT_TRUE(contains(alone.err, "valid, not plannable yet"));

  write("t.pddl", kTimed);
  Outcome t = run("check '" + path("t.pddl") + "'");
  EXPECT_EQ(t.code, 0);
  EXPECT_TRUE(contains(t.err, "warning"));
  EXPECT_TRUE(contains(t.err, "valid, not plannable yet"));

  write("broken.pddl", kBroken);
  Outcome b = run("check '" + path("broken.pddl") + "'");
  EXPECT_EQ(b.code, 2);
  EXPECT_TRUE(contains(b.err, "invalid"));
  EXPECT_EQ(run("check").code, 2);
  EXPECT_EQ(run("check '" + path("nope.pddl") + "'").code, 2);
}

TEST_F(CliTest, ValidateVerdicts)
{
  write("good.plan", "(move r1 kitchen corridor)\n(move r1 corridor hall)\n");
  write("bad.plan", "(move r1 kitchen hall)\n");
  write("short.plan", "(move r1 kitchen corridor)\n");
  write("garbage.plan", "(move r1\n");
  Outcome g = run("validate " + solve_args() + " '" + path("good.plan") + "'");
  EXPECT_EQ(g.code, 0);
  EXPECT_TRUE(contains(g.err, "plan valid, makespan 2"));
  Outcome b = run("validate " + solve_args() + " '" + path("bad.plan") + "'");
  EXPECT_EQ(b.code, 1);
  EXPECT_TRUE(contains(b.err, "(connected kitchen hall)"));
  Outcome s = run("validate " + solve_args() + " '" + path("short.plan") + "'");
  EXPECT_EQ(s.code, 1);
  EXPECT_TRUE(contains(s.err, "goal"));
  EXPECT_EQ(run("validate " + solve_args() + " '" + path("garbage.plan") + "'").code, 2);
  EXPECT_EQ(run("validate " + solve_args()).code, 2);
}

TEST_F(CliTest, PlansSuaveInMilliseconds)
{
  // The PlanSys2 example with derived predicates, end to end in the executable
  std::string data = std::string(KAIROS_TEST_DATA_DIR) + "/plansys2/";
  Outcome r = run(
    "'" + data + "plansys2_executor__suave_domain.pddl' '" + data +
    "plansys2_executor__suave_problem.pddl' -v");
  ASSERT_EQ(r.code, 0) << r.err;
  EXPECT_EQ(lines(r.out).size(), 8u);  // the cost and seven actions
  EXPECT_TRUE(contains(r.out, "(inspect_pipeline pipeline bluerov)"));
}

}  // namespace
