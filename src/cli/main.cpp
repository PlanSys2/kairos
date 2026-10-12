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

// The kairos command: plans to stdout, everything else to stderr. Uses the public API only.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "completion.hpp"
#include "kairos/kairos.hpp"

namespace
{

using kairos::PlanFormat;

constexpr int kSolved = 0;
constexpr int kInternal = 1;
constexpr int kInvalidInput = 2;
constexpr int kUnsolvable = 10;
constexpr int kTimeout = 11;
constexpr int kMemory = 12;
constexpr int kUnsupported = 13;

std::atomic<bool> g_interrupted{false};

extern "C" void on_signal([[maybe_unused]] int signal)
{
  g_interrupted = true;
}

const char * kUsage =
  "Usage:\n"
  "  kairos [solve] DOMAIN PROBLEM [options]   plan; the plan goes to stdout\n"
  "  kairos validate DOMAIN PROBLEM PLAN [--epsilon E]\n"
  "  kairos check DOMAIN [PROBLEM]            check a domain, or a problem against it\n"
  "  kairos check --problem PROBLEM           check a problem alone\n"
  "  kairos --version | --help\n"
  "  kairos --completion bash|zsh|fish        shell completion, e.g. in ~/.bashrc:\n"
  "                                           eval \"$(kairos --completion bash)\"\n"
  "\n"
  "Options for solve:\n"
  "  --timeout S             seconds (default 30)\n"
  "  --anytime | --no-anytime  keep improving after the first plan (default: anytime)\n"
  "  --engine auto|m1|m2|m3\n"
  "  --search NAME           M1 search: auto (default), ff, lama, eager-lama, novelty,\n"
  "                          lama-novelty, ehc\n"
  "  --format ipc|popf|json  plan format (default ipc; popf on stdout: only the best plan)\n"
  "  --output FILE           write plans to FILE (anytime: FILE.1, FILE.2, ...)\n"
  "  --memory MB             memory limit (default: half of what is available)\n"
  "  --threads N             phases of the search at once (default: the cores, up to 4;\n"
  "                          1 for reproducible runs)\n"
  "  --epsilon E  --seed N\n"
  "  --target-cost C         stop at a plan this good\n"
  "  --no-improvement S      stop when no better plan comes in S seconds\n"
  "  -v                      progress and statistics on stderr\n"
  "  --debug[=CHANNELS]      debug messages (all channels, or e.g. search,ground)\n"
  "  --dump=WHAT --dump-dir DIR  write ast, lifted, ground, sas, plan-trace to DIR\n"
  "\n"
  "Exit codes: 0 solved, 10 unsolvable, 11 timeout, 12 memory, 13 unsupported,\n"
  "2 invalid input, 1 internal error. With --format popf: 0 also without a plan.\n";

std::optional<std::string> read(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return std::nullopt;
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// Diagnostics with the line of the file they point to
void print(
  const std::vector<kairos::Diagnostic> & ds, const std::map<std::string, std::string> & sources)
{
  for (const auto & d : ds) {
    auto it = d.location ? sources.find(d.location->file) : sources.end();
    std::cerr << (it != sources.end() ? kairos::to_string(d, it->second) : kairos::to_string(d))
              << "\n";
  }
}

int exit_code(kairos::Status s)
{
  switch (s) {
    case kairos::Status::Solved: return kSolved;
    case kairos::Status::Unsolvable: return kUnsolvable;
    case kairos::Status::Timeout: return kTimeout;
    case kairos::Status::OutOfMemory:
    case kairos::Status::GroundingTooLarge: return kMemory;
    case kairos::Status::Unsupported: return kUnsupported;
    case kairos::Status::InvalidInput: return kInvalidInput;
    default: return kInternal;
  }
}

struct Args
{
  std::vector<std::string> positional;
  std::map<std::string, std::string> options;  // --name value, or --name=value
  std::vector<std::string> flags;
  bool verbose = false;
};

// Options that take a value after a space
const std::vector<std::string> kWithValue = {
  "--timeout", "--engine", "--format", "--output", "--memory", "--threads", "--epsilon",
  "--seed", "--target-cost", "--no-improvement", "--dump-dir", "--problem", "--search"};
// Options with an optional "=value", and plain flags
const std::vector<std::string> kOptionalValue = {"--debug", "--dump"};
const std::vector<std::string> kFlags = {"--anytime", "--no-anytime"};
// Values of some options
const std::vector<std::string> kFormats = {"ipc", "popf", "json"};
const std::vector<std::string> kDumps = {"ast", "lifted", "ground", "sas", "plan-trace"};

// What shell completion offers, from the lists the parser uses
kairos::cli::CompletionSpec completion_spec()
{
  kairos::cli::CompletionSpec spec;
  spec.commands = {"solve", "validate", "check"};
  spec.options[""] = {"--help", "--version", "--completion"};
  auto & solve = spec.options["solve"];
  for (const auto & o : kWithValue) {
    if (o != "--problem") {
      solve.push_back(o);
    }
  }
  solve.insert(solve.end(), kOptionalValue.begin(), kOptionalValue.end());
  solve.insert(solve.end(), kFlags.begin(), kFlags.end());
  solve.push_back("-v");
  spec.options["validate"] = {"--epsilon"};
  spec.options["check"] = {"--problem"};
  spec.values["--engine"] = {"auto", "m1", "m2", "m3"};
  spec.values["--search"] = {"auto"};
  for (const auto & p : kairos::search_presets()) {
    spec.values["--search"].push_back(p);
  }
  spec.values["--format"] = kFormats;
  spec.values["--completion"] = {"bash", "zsh", "fish"};
  spec.lists["--debug"] = {"all"};
  for (size_t c = 0; c < static_cast<size_t>(kairos::LogChannel::Count); ++c) {
    spec.lists["--debug"].emplace_back(kairos::to_string(static_cast<kairos::LogChannel>(c)));
  }
  spec.lists["--dump"] = kDumps;
  spec.file_options = {"--output", "--problem"};
  spec.dir_options = {"--dump-dir"};
  spec.number_options = {"--timeout", "--memory", "--threads", "--epsilon", "--seed",
    "--target-cost", "--no-improvement"};
  return spec;
}

bool listed(const std::vector<std::string> & names, const std::string & name)
{
  return std::find(names.begin(), names.end(), name) != names.end();
}

std::optional<Args> parse_args(const std::vector<std::string> & argv, std::string & error)
{
  Args a;
  for (size_t i = 0; i < argv.size(); ++i) {
    const std::string & s = argv[i];
    if (s == "-v") {
      a.verbose = true;
    } else if (s.rfind("--", 0) == 0) {
      auto eq = s.find('=');
      std::string name = s.substr(0, eq);
      bool takes_value = listed(kWithValue, name) || listed(kOptionalValue, name);
      if (!takes_value && !listed(kFlags, name)) {
        error = "unknown option " + name;
        return std::nullopt;
      }
      if (eq != std::string::npos && !takes_value) {
        error = name + " takes no value";
        return std::nullopt;
      }
      if (eq != std::string::npos) {
        a.options[name] = s.substr(eq + 1);
      } else if (listed(kWithValue, name)) {
        if (i + 1 >= argv.size()) {
          error = "missing value for " + name;
          return std::nullopt;
        }
        a.options[name] = argv[++i];
      } else {
        a.flags.push_back(name);
      }
    } else {
      a.positional.push_back(s);
    }
  }
  return a;
}

bool has(const Args & a, const std::string & flag)
{
  return std::find(a.flags.begin(), a.flags.end(), flag) != a.flags.end() ||
         a.options.count(flag);
}

std::optional<double> number(const Args & a, const std::string & name, std::string & error)
{
  auto it = a.options.find(name);
  if (it == a.options.end()) {
    return std::nullopt;
  }
  try {
    size_t used = 0;
    double v = std::stod(it->second, &used);
    if (used == it->second.size() && v >= 0.0) {
      return v;
    }
  } catch (...) {
  }
  error = "bad value for " + name + ": " + it->second;
  return std::nullopt;
}

int check_command(const Args & a)
{
  std::map<std::string, std::string> sources;
  kairos::CheckReport report;
  if (auto it = a.options.find("--problem"); it != a.options.end()) {
    auto text = read(it->second);
    if (!text) {
      std::cerr << "kairos: cannot read " << it->second << "\n";
      return kInvalidInput;
    }
    sources[it->second] = *text;
    report = kairos::check_problem(*text, it->second);
  } else if (a.positional.size() == 1 || a.positional.size() == 2) {
    auto dtext = read(a.positional[0]);
    if (!dtext) {
      std::cerr << "kairos: cannot read " << a.positional[0] << "\n";
      return kInvalidInput;
    }
    sources[a.positional[0]] = *dtext;
    if (a.positional.size() == 1) {
      report = kairos::check_domain(*dtext, a.positional[0]);
    } else {
      auto ptext = read(a.positional[1]);
      if (!ptext) {
        std::cerr << "kairos: cannot read " << a.positional[1] << "\n";
        return kInvalidInput;
      }
      sources[a.positional[1]] = *ptext;
      report = kairos::check_problem(*ptext, *dtext, a.positional[1], a.positional[0]);
    }
  } else {
    std::cerr << kUsage;
    return kInvalidInput;
  }
  print(report.diagnostics, sources);
  std::cerr << (report.valid ? "valid" : "invalid");
  if (report.valid) {
    std::cerr << (report.plannable ? ", plannable" : ", not plannable yet");
  }
  std::cerr << "\n";
  if (!report.features.empty()) {
    std::cerr << "features:";
    for (const auto & f : report.features) {
      std::cerr << " " << f;
    }
    std::cerr << "\n";
  }
  return report.valid ? kSolved : kInvalidInput;
}

int validate_command(const Args & a)
{
  if (a.positional.size() != 3) {
    std::cerr << kUsage;
    return kInvalidInput;
  }
  std::map<std::string, std::string> sources;
  std::vector<std::optional<std::string>> texts;
  for (const auto & p : a.positional) {
    texts.push_back(read(p));
    if (!texts.back()) {
      std::cerr << "kairos: cannot read " << p << "\n";
      return kInvalidInput;
    }
    sources[p] = *texts.back();
  }
  std::string error;
  kairos::ValidateOptions vo;
  if (auto e = number(a, "--epsilon", error)) {
    vo.epsilon = *e;
  }
  if (!error.empty()) {
    std::cerr << "kairos: " << error << "\n";
    return kInvalidInput;
  }
  auto plan = kairos::parse_plan(*texts[2], a.positional[2]);
  if (!plan.ok()) {
    print(plan.diagnostics(), sources);
    return kInvalidInput;
  }
  vo.plan_filename = a.positional[2];
  auto r = kairos::validate_plan(*texts[0], *texts[1], *plan, vo, a.positional[0],
      a.positional[1]);
  print(r.diagnostics, sources);
  std::cerr << "plan " << kairos::to_string(r.status);
  if (r.valid()) {
    std::cerr << ", makespan " << r.makespan;
    if (r.metric) {
      std::cerr << ", metric " << *r.metric;
    }
  }
  std::cerr << "\n";
  if (r.status == kairos::ValidationStatus::Valid) {
    return kSolved;
  }
  return r.status == kairos::ValidationStatus::BadInput ? kInvalidInput : kInternal;
}

int solve_command(const Args & a)
{
  if (a.positional.size() != 2) {
    std::cerr << kUsage;
    return kInvalidInput;
  }
  std::string error;
  kairos::SearchOptions o;
  if (auto t = number(a, "--timeout", error)) {
    o.timeout = std::chrono::milliseconds(static_cast<int64_t>(*t * 1000));
  }
  if (auto m = number(a, "--memory", error)) {
    o.memory_limit_mb = static_cast<size_t>(*m);
  }
  // The phases of the search at once: up to 4 threads, never more than the cores (the library
  // keeps 1 unless asked; Kairos uses no more threads than phases)
  o.threads = std::clamp(std::thread::hardware_concurrency(), 1u, 4u);
  if (auto t = number(a, "--threads", error)) {
    o.threads = std::max(1u, static_cast<unsigned>(*t));
  }
  if (auto e = number(a, "--epsilon", error)) {
    o.epsilon = *e;
  }
  if (auto s = number(a, "--seed", error)) {
    o.seed = static_cast<uint64_t>(*s);
  }
  if (auto c = number(a, "--target-cost", error)) {
    o.target_cost = *c;
  }
  if (auto n = number(a, "--no-improvement", error)) {
    o.stop_after_no_improvement = std::chrono::milliseconds(static_cast<int64_t>(*n * 1000));
  }
  if (has(a, "--no-anytime")) {
    o.anytime = false;
  }
  if (auto it = a.options.find("--engine"); it != a.options.end()) {
    const std::map<std::string, kairos::Engine> engines = {
      {"auto", kairos::Engine::Auto}, {"m1", kairos::Engine::M1}, {"m2", kairos::Engine::M2},
      {"m3", kairos::Engine::M3}};
    auto e = engines.find(it->second);
    if (e == engines.end()) {
      error = "unknown engine " + it->second;
    } else {
      o.engine = e->second;
    }
  }
  if (auto it = a.options.find("--search"); it != a.options.end()) {
    auto names = kairos::search_presets();
    if (it->second != "auto" &&
      std::find(names.begin(), names.end(), it->second) == names.end())
    {
      error = "unknown search " + it->second;
    }
    o.search = it->second;
  }
  PlanFormat format = PlanFormat::Ipc;
  if (auto it = a.options.find("--format"); it != a.options.end()) {
    if (it->second == kFormats[0]) {
      format = PlanFormat::Ipc;
    } else if (it->second == kFormats[1]) {
      format = PlanFormat::Popf;
    } else if (it->second == kFormats[2]) {
      format = PlanFormat::Json;
    } else {
      error = "unknown format " + it->second;
    }
  }
  // Logging to stderr: -v for progress, --debug for the debug channels
  auto t0 = std::chrono::steady_clock::now();
  o.log.level = a.verbose ? kairos::LogLevel::Info : kairos::LogLevel::Warning;
  if (has(a, "--debug")) {
    o.log.level = kairos::LogLevel::Debug;
    auto it = a.options.find("--debug");
    if (it != a.options.end()) {
      auto channels = kairos::LogChannels::parse(it->second);
      if (!channels) {
        error = "unknown debug channel in " + it->second;
      } else {
        o.log.debug_channels = *channels;
      }
    }
  }
  o.log.callback = [t0](kairos::LogLevel, kairos::LogChannel c, std::string_view m) {
      double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      char head[64];
      std::snprintf(head, sizeof(head), "[%8.3fs] [", s);
      std::cerr << head << kairos::to_string(c) << "] " << m << "\n";
    };
  if (auto it = a.options.find("--dump-dir"); it != a.options.end()) {
    o.dump_dir = it->second;
  }
  if (has(a, "--dump") && !a.options.count("--dump") && !o.dump_dir) {
    o.dump_dir = ".";  // --dump alone: everything, here
  }
  if (auto it = a.options.find("--dump"); it != a.options.end()) {
    std::stringstream ss(it->second);
    std::string w;
    while (std::getline(ss, w, ',')) {
      if (!listed(kDumps, w)) {
        error = "unknown dump " + w;
      }
      o.dump.insert(w);
    }
    if (!o.dump_dir) {
      o.dump_dir = ".";
    }
  }
  if (!error.empty()) {
    std::cerr << "kairos: " << error << "\n";
    return kInvalidInput;
  }

  std::map<std::string, std::string> sources;
  auto dtext = read(a.positional[0]);
  auto ptext = read(a.positional[1]);
  if (!dtext || !ptext) {
    std::cerr << "kairos: cannot read " << (dtext ? a.positional[1] : a.positional[0]) << "\n";
    return kInvalidInput;
  }
  sources[a.positional[0]] = *dtext;
  sources[a.positional[1]] = *ptext;
  auto domain = kairos::parse_domain(*dtext, a.positional[0]);
  if (!domain.ok()) {
    print(domain.diagnostics(), sources);
    return format == PlanFormat::Popf ? kSolved : kInvalidInput;
  }
  auto problem = kairos::parse_problem(*ptext, *domain, a.positional[1]);
  if (!problem.ok()) {
    print(problem.diagnostics(), sources);
    return format == PlanFormat::Popf ? kSolved : kInvalidInput;
  }
  if (a.verbose) {
    print(domain.diagnostics(), sources);
    print(problem.diagnostics(), sources);
  }

  // Each better plan as it comes
  std::optional<std::string> output;
  if (auto it = a.options.find("--output"); it != a.options.end()) {
    output = it->second;
  }
  // Plans as they come, unless the reader expects a single one (POPF format on stdout)
  bool anytime = o.anytime && (output || format != PlanFormat::Popf);
  // Times rounded to 3 decimals can break a valid plan (an action that starts exactly when
  // another ends, with durations such as 121.9775): then more decimals, as many as it needs
  auto precision = [&](const kairos::Plan & plan) {
      if (!plan.temporal || format == PlanFormat::Json) {
        return 3;
      }
      for (int p : {3, 6, 9}) {
        auto back = kairos::parse_plan(kairos::to_string(plan, PlanFormat::Ipc, p));
        if (back.ok() && kairos::validate(*problem, *back).valid()) {
          return p;
        }
      }
      return 9;
    };
  auto write = [&](const kairos::Plan & plan, uint32_t index) {
      std::string text = kairos::to_string(plan, format, precision(plan));
      if (output) {
        std::string file = anytime ? *output + "." + std::to_string(index + 1) : *output;
        std::ofstream(file) << text;
        return;
      }
      if (index > 0 && format != PlanFormat::Json) {
        std::cout << "; better plan " << index + 1 << "\n";
      }
      std::cout << text << std::flush;
    };
  kairos::Planner planner;
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  auto search = planner.start(*problem, o, [&](const kairos::PlanUpdate & u) {
        if (anytime) {
          write(u.plan, u.index);
        }
      });
  while (search.running()) {
    if (g_interrupted) {
      search.stop();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  kairos::SolveResult r = search.wait();
  if (!anytime && r.plan) {
    write(*r.plan, 0);
  }
  print(r.diagnostics, sources);
  if (a.verbose || r.status != kairos::Status::Solved) {
    std::cerr << "kairos: " << kairos::to_string(r.status) << " (" <<
      kairos::to_string(r.stop_reason) << ")";
    if (r.plan && r.plan->cost) {
      std::cerr << ", cost " << *r.plan->cost;
    }
    std::cerr << "\n";
  }
  if (a.verbose) {
    const auto & s = r.stats;
    std::fprintf(
      stderr,
      "kairos: parse %.1f ms, normalize %.1f ms, ground %.1f ms, sas %.1f ms, search %.1f ms, "
      "total %.1f ms, first plan %.1f ms\n"
      "kairos: %zu atoms, %zu actions, %zu variables (%zu mutex groups), %zu states, "
      "%zu expanded, %zu evaluated, %zu plans, %.1f bytes per state, %zu MB peak\n",
      s.parse_ms, s.normalize_ms, s.ground_ms, s.sas_ms, s.search_ms, s.total_ms,
      s.first_plan_ms.value_or(-1.0), s.atoms,
      s.actions, s.variables, s.mutex_groups, s.states, s.expanded, s.evaluated, s.plans,
      s.bytes_per_state, s.peak_memory_bytes >> 20);
  }
  if (format == PlanFormat::Popf) {
    return kSolved;  // the PlanSys2 plugin expects 0 and looks for the marker
  }
  if (r.status == kairos::Status::Cancelled && r.plan) {
    return kSolved;
  }
  return exit_code(r.status);
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> args(argv + 1, argv + argc);
  if (args.empty()) {
    std::cerr << kUsage;
    return kInvalidInput;
  }
  if (std::find(args.begin(), args.end(), "--help") != args.end() ||
    std::find(args.begin(), args.end(), "-h") != args.end())
  {
    std::cout << kUsage;
    return kSolved;
  }
  if (args[0] == "--completion") {
    auto script = args.size() == 2 ?
      kairos::cli::completion_script(args[1], completion_spec()) : std::nullopt;
    if (!script) {
      std::cerr << "kairos: --completion takes bash, zsh or fish\n";
      return kInvalidInput;
    }
    std::cout << *script;
    return kSolved;
  }
  if (args[0] == "--version") {
    std::cout << "kairos " << kairos::version() << "\n";
    return kSolved;
  }
  std::string command = "solve";
  if (args[0] == "solve" || args[0] == "validate" || args[0] == "check") {
    command = args[0];
    args.erase(args.begin());
  }
  std::string error;
  auto a = parse_args(args, error);
  if (!a) {
    std::cerr << "kairos: " << error << "\n";
    return kInvalidInput;
  }
  try {
    if (command == "check") {
      return check_command(*a);
    }
    if (command == "validate") {
      return validate_command(*a);
    }
    return solve_command(*a);
  } catch (const std::exception & e) {
    std::cerr << "kairos: internal error: " << e.what() << "\n";
    return kInternal;
  }
}
