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

// The public API of libkairos for Python (module kairos._kairos; kairos/__init__.py adds the
// Pythonic layer). Planning releases the GIL; callbacks from Kairos' threads take it.

#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/chrono.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <stop_token>  // NOLINT(build/include_order): C++20, unknown to cpplint
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "kairos/kairos.hpp"

namespace nb = nanobind;
using nb::literals::operator""_a;

namespace
{

using kairos::Diagnostic;
using Millis = std::chrono::milliseconds;

// Seconds (float) to milliseconds, and back
Millis to_ms(double seconds) {return Millis(static_cast<int64_t>(seconds * 1000.0));}
double to_s(Millis ms) {return static_cast<double>(ms.count()) / 1000.0;}

// A Python callable that may be dropped from any thread: the GIL is taken to release it
using Callable = std::shared_ptr<nb::object>;

Callable hold(nb::object f)
{
  if (f.is_none()) {
    return nullptr;
  }
  return Callable(new nb::object(std::move(f)), [](nb::object * o) {
             nb::gil_scoped_acquire gil;
             delete o;
           });
}

// The log callback of the options: Python's, called with the GIL
void attach_log(kairos::LogOptions & log, const Callable & on_log)
{
  if (!on_log) {
    log.callback = {};
    return;
  }
  log.callback = [on_log](kairos::LogLevel l, kairos::LogChannel c, std::string_view m) {
      nb::gil_scoped_acquire gil;
      try {
        (*on_log)(l, c, std::string(m));
      } catch (nb::python_error & e) {
        e.discard_as_unraisable("kairos log callback");
      }
    };
}

// A value or the diagnostics of the failure
template<class T>
std::pair<std::optional<T>, std::vector<Diagnostic>> unpack(kairos::Result<T> r)
{
  if (r.ok()) {
    auto diagnostics = r.diagnostics();
    return {std::move(r).value(), std::move(diagnostics)};
  }
  return {std::nullopt, r.take_diagnostics()};
}

// Runs `work` without the GIL in a thread of its own, while this one answers signals: Ctrl-C
// requests a stop (and the KeyboardInterrupt is raised once the work returns)
template<class Work>
auto interruptible(Work work) -> decltype(work(std::stop_token {}))
{
  using R = decltype(work(std::stop_token {}));
  std::stop_source stop;
  std::optional<R> result;
  std::exception_ptr failure;
  std::atomic<bool> done{false};
  bool interrupted = false;
  {
    std::jthread worker;
    {
      nb::gil_scoped_release release;
      worker = std::jthread([&]() {
            try {
              result.emplace(work(stop.get_token()));
            } catch (...) {
              failure = std::current_exception();
            }
            done = true;
          });
    }
    while (!done) {
      {
        nb::gil_scoped_release release;
        for (int i = 0; i < 10 && !done; ++i) {
          std::this_thread::sleep_for(Millis(5));
        }
      }
      if (!done && !interrupted && PyErr_CheckSignals() != 0) {
        interrupted = true;
        stop.request_stop();
      }
    }
    nb::gil_scoped_release release;
    worker.join();
  }
  if (failure) {
    std::rethrow_exception(failure);
  }
  if (interrupted) {
    throw nb::python_error();  // the KeyboardInterrupt PyErr_CheckSignals set
  }
  return std::move(*result);
}

// An anytime search with its callbacks; stopped and joined without the GIL (its thread may be
// waiting for it in a callback)
struct PySearch
{
  kairos::Search search;
  Callable on_plan, on_log;

  ~PySearch()
  {
    nb::gil_scoped_release release;
    search.stop();
    search = kairos::Search();
  }
};

}  // namespace

NB_MODULE(_kairos, m)
{
  m.doc() = "Kairos, a PDDL planner: the C++ API (see the kairos package)";
  m.def("version", []() {return std::string(kairos::version());},
    "The version of libkairos, e.g. '1.0.2'.");
  m.def("search_presets", &kairos::search_presets, "The search configurations of engine M1.");

  // --- Diagnostics ---------------------------------------------------------------------------
  nb::enum_<kairos::Severity>(m, "Severity")
  .value("ERROR", kairos::Severity::Error)
  .value("WARNING", kairos::Severity::Warning)
  .value("NOTE", kairos::Severity::Note);

  nb::class_<kairos::SourceLocation>(m, "SourceLocation")
  .def_ro("file", &kairos::SourceLocation::file)
  .def_ro("line", &kairos::SourceLocation::line)
  .def_ro("column", &kairos::SourceLocation::column)
  .def("__repr__", [](const kairos::SourceLocation & l) {
      return l.file + ":" + std::to_string(l.line) + ":" + std::to_string(l.column);
    });

  nb::class_<Diagnostic>(m, "Diagnostic",
    "A message about the input or the planning process; `code` is stable, `message` is for "
    "humans.")
  .def_ro("severity", &Diagnostic::severity)
  .def_ro("code", &Diagnostic::code)
  .def_ro("message", &Diagnostic::message)
  .def_ro("location", &Diagnostic::location)
  .def("format", [](const Diagnostic & d, std::string_view source) {
      return kairos::to_string(d, source);
    }, "source"_a, "As compilers print it, with the source line and a caret under the column.")
  .def("__str__", [](const Diagnostic & d) {return kairos::to_string(d);})
  .def("__repr__", [](const Diagnostic & d) {return "<Diagnostic " + kairos::to_string(d) + ">";});

  // --- Plans ---------------------------------------------------------------------------------
  nb::class_<kairos::PlanStep>(m, "PlanStep")
  .def(nb::init<>())
  .def("__init__", [](kairos::PlanStep * s, std::string name, std::vector<std::string> args,
    double time, double duration) {
      new (s) kairos::PlanStep{time, duration, std::move(name), std::move(args), 0};
    }, "name"_a, "args"_a = std::vector<std::string>{}, "time"_a = 0.0, "duration"_a = 0.0)
  .def_rw("time", &kairos::PlanStep::time)
  .def_rw("duration", &kairos::PlanStep::duration)
  .def_rw("name", &kairos::PlanStep::name)
  .def_rw("args", &kairos::PlanStep::args)
  .def_ro("line", &kairos::PlanStep::line)
  .def("action", &kairos::PlanStep::action, "'(move r1 kitchen hall)'.")
  .def(nb::self == nb::self)
  .def("__repr__", [](const kairos::PlanStep & s) {
      return "<PlanStep " + std::to_string(s.time) + ": " + s.action() + " [" +
             std::to_string(s.duration) + "]>";
    });

  nb::enum_<kairos::PlanFormat>(m, "PlanFormat")
  .value("IPC", kairos::PlanFormat::Ipc)
  .value("POPF", kairos::PlanFormat::Popf)
  .value("JSON", kairos::PlanFormat::Json);

  nb::class_<kairos::Plan>(m, "Plan")
  .def(nb::init<>())
  .def_rw("steps", &kairos::Plan::steps)
  .def_rw("temporal", &kairos::Plan::temporal)
  .def_rw("cost", &kairos::Plan::cost)
  .def_rw("end", &kairos::Plan::end)
  .def("makespan", &kairos::Plan::makespan)
  .def("to_string", [](const kairos::Plan & p, kairos::PlanFormat f, int precision) {
      return kairos::to_string(p, f, precision);
    }, "format"_a = kairos::PlanFormat::Ipc, "precision"_a = 3)
  .def(nb::self == nb::self)
  .def("__len__", [](const kairos::Plan & p) {return p.steps.size();})
  .def("__str__", [](const kairos::Plan & p) {
      return kairos::to_string(p, kairos::PlanFormat::Ipc);
    })
  .def("__repr__", [](const kairos::Plan & p) {
      return "<Plan of " + std::to_string(p.steps.size()) + " steps" +
             (p.cost ? ", cost " + std::to_string(*p.cost) : std::string()) + ">";
    });
  m.def("_parse_plan", [](std::string_view text, std::string_view filename) {
      return unpack(kairos::parse_plan(text, filename));
    }, "text"_a, "filename"_a = "plan");

  // --- Domains and problems ------------------------------------------------------------------
  nb::class_<kairos::Domain>(m, "Domain")
  .def_prop_ro("name", &kairos::Domain::name)
  .def("features", &kairos::Domain::features, "The PDDL features the domain uses.")
  .def("__repr__", [](const kairos::Domain & d) {return "<Domain " + d.name() + ">";});
  nb::class_<kairos::Problem>(m, "Problem")
  .def_prop_ro("name", &kairos::Problem::name)
  .def_prop_ro("domain", &kairos::Problem::domain)
  .def("features", &kairos::Problem::features, "The PDDL features the problem uses.")
  .def("__repr__", [](const kairos::Problem & p) {return "<Problem " + p.name() + ">";});
  m.def("_parse_domain", [](std::string_view text, std::string_view filename) {
      return unpack(kairos::parse_domain(text, filename));
    }, "text"_a, "filename"_a = "domain.pddl");
  m.def("_parse_problem", [](std::string_view text, const kairos::Domain & d,
    std::string_view filename) {
      return unpack(kairos::parse_problem(text, d, filename));
    }, "text"_a, "domain"_a, "filename"_a = "problem.pddl");
  m.def("_load_domain", [](const std::filesystem::path & f) {
      return unpack(kairos::load_domain(f));
    }, "file"_a);
  m.def("_load_problem", [](const std::filesystem::path & f, const kairos::Domain & d) {
      return unpack(kairos::load_problem(f, d));
    }, "file"_a, "domain"_a);

  // --- Checking ------------------------------------------------------------------------------
  nb::enum_<kairos::SupportLevel>(m, "SupportLevel")
  .value("FULL", kairos::SupportLevel::Full)
  .value("PARTIAL", kairos::SupportLevel::Partial)
  .value("NONE", kairos::SupportLevel::None);
  nb::class_<kairos::FeatureSupport>(m, "FeatureSupport")
  .def_ro("feature", &kairos::FeatureSupport::feature)
  .def_ro("requirements", &kairos::FeatureSupport::requirements)
  .def_ro("level", &kairos::FeatureSupport::level)
  .def_ro("note", &kairos::FeatureSupport::note)
  .def("__repr__", [](const kairos::FeatureSupport & f) {
      return "<FeatureSupport " + f.feature + ": " + std::string(kairos::to_string(f.level)) + ">";
    });
  m.def("pddl_support", &kairos::pddl_support, "What Kairos plans with, feature by feature.");
  nb::class_<kairos::CheckReport>(m, "CheckReport")
  .def_ro("valid", &kairos::CheckReport::valid)
  .def_ro("plannable", &kairos::CheckReport::plannable)
  .def_ro("plannable_known", &kairos::CheckReport::plannable_known)
  .def_ro("diagnostics", &kairos::CheckReport::diagnostics)
  .def_ro("features", &kairos::CheckReport::features)
  .def_ro("support", &kairos::CheckReport::support);
  m.def("check_domain", [](std::string_view text, std::string_view filename) {
      return kairos::check_domain(text, filename);
    }, "text"_a, "filename"_a = "domain.pddl", "Checks a domain: errors, warnings, features.");
  m.def("check_problem", [](std::string_view problem, std::optional<std::string_view> domain,
    std::string_view problem_filename, std::string_view domain_filename) {
      return domain ?
             kairos::check_problem(problem, *domain, problem_filename, domain_filename) :
             kairos::check_problem(problem, problem_filename);
    }, "problem"_a, "domain"_a = nb::none(), "problem_filename"_a = "problem.pddl",
    "domain_filename"_a = "domain.pddl",
    "Checks a problem, against its domain (text) if given.");

  // --- Validation ----------------------------------------------------------------------------
  nb::enum_<kairos::ValidationStatus>(m, "ValidationStatus")
  .value("VALID", kairos::ValidationStatus::Valid)
  .value("INVALID", kairos::ValidationStatus::Invalid)
  .value("UNSUPPORTED", kairos::ValidationStatus::Unsupported)
  .value("BAD_INPUT", kairos::ValidationStatus::BadInput);
  nb::class_<kairos::ValidateOptions>(m, "ValidateOptions")
  .def(nb::init<>())
  .def_rw("epsilon", &kairos::ValidateOptions::epsilon)
  .def_rw("tolerance", &kairos::ValidateOptions::tolerance)
  .def_rw("duration_tolerance", &kairos::ValidateOptions::duration_tolerance)
  .def_rw("strict_epsilon", &kairos::ValidateOptions::strict_epsilon)
  .def_rw("check_goal", &kairos::ValidateOptions::check_goal)
  .def_rw("continuous_step", &kairos::ValidateOptions::continuous_step)
  .def_rw("plan_filename", &kairos::ValidateOptions::plan_filename);
  nb::class_<kairos::ValidationReport>(m, "ValidationReport")
  .def_ro("status", &kairos::ValidationReport::status)
  .def_ro("diagnostics", &kairos::ValidationReport::diagnostics)
  .def_ro("makespan", &kairos::ValidationReport::makespan)
  .def_ro("metric", &kairos::ValidationReport::metric)
  .def_ro("violated_preferences", &kairos::ValidationReport::violated_preferences)
  .def_ro("failed_step", &kairos::ValidationReport::failed_step)
  .def_prop_ro("valid", &kairos::ValidationReport::valid)
  .def("__bool__", &kairos::ValidationReport::valid)
  .def("__repr__", [](const kairos::ValidationReport & r) {
      return "<ValidationReport " + std::string(kairos::to_string(r.status)) + ">";
    });
  m.def("validate", [](const kairos::Problem & p, const kairos::Plan & plan,
    const kairos::ValidateOptions & o) {
      nb::gil_scoped_release release;
      return kairos::validate(p, plan, o);
    }, "problem"_a, "plan"_a, "options"_a = kairos::ValidateOptions(),
    "Checks that the plan solves the problem (VAL's semantics; also PDDL+ and PDDL3).");
  m.def("validate_plan", [](std::string_view domain, std::string_view problem,
    const kairos::Plan & plan, const kairos::ValidateOptions & o) {
      nb::gil_scoped_release release;
      return kairos::validate_plan(domain, problem, plan, o);
    }, "domain"_a, "problem"_a, "plan"_a, "options"_a = kairos::ValidateOptions(),
    "validate() from the texts of the domain and the problem.");

  // --- Planning ------------------------------------------------------------------------------
  nb::enum_<kairos::Status>(m, "Status")
  .value("SOLVED", kairos::Status::Solved)
  .value("UNSOLVABLE", kairos::Status::Unsolvable)
  .value("TIMEOUT", kairos::Status::Timeout)
  .value("CANCELLED", kairos::Status::Cancelled)
  .value("OUT_OF_MEMORY", kairos::Status::OutOfMemory)
  .value("UNSUPPORTED", kairos::Status::Unsupported)
  .value("INVALID_INPUT", kairos::Status::InvalidInput)
  .value("GROUNDING_TOO_LARGE", kairos::Status::GroundingTooLarge)
  .value("INTERNAL_ERROR", kairos::Status::InternalError)
  .value("NO_PLAN_FOUND", kairos::Status::NoPlanFound);
  nb::enum_<kairos::StopReason>(m, "StopReason")
  .value("EXHAUSTED", kairos::StopReason::Exhausted)
  .value("PROVEN_OPTIMAL", kairos::StopReason::ProvenOptimal)
  .value("TIMEOUT", kairos::StopReason::Timeout)
  .value("NO_IMPROVEMENT", kairos::StopReason::NoImprovement)
  .value("TARGET_REACHED", kairos::StopReason::TargetReached)
  .value("STOPPED", kairos::StopReason::Stopped)
  .value("MEMORY_LIMIT", kairos::StopReason::MemoryLimit);
  nb::enum_<kairos::Engine>(m, "Engine")
  .value("AUTO", kairos::Engine::Auto)
  .value("M1", kairos::Engine::M1)
  .value("M2", kairos::Engine::M2)
  .value("M3", kairos::Engine::M3);
  nb::enum_<kairos::LogLevel>(m, "LogLevel")
  .value("DEBUG", kairos::LogLevel::Debug)
  .value("INFO", kairos::LogLevel::Info)
  .value("WARNING", kairos::LogLevel::Warning)
  .value("ERROR", kairos::LogLevel::Error);
  nb::enum_<kairos::LogChannel>(m, "LogChannel")
  .value("GENERAL", kairos::LogChannel::General)
  .value("PARSE", kairos::LogChannel::Parse)
  .value("SEMANTIC", kairos::LogChannel::Semantic)
  .value("NORMALIZE", kairos::LogChannel::Normalize)
  .value("GROUND", kairos::LogChannel::Ground)
  .value("SAS", kairos::LogChannel::Sas)
  .value("SEARCH", kairos::LogChannel::Search)
  .value("HEURISTIC", kairos::LogChannel::Heuristic)
  .value("TEMPORAL", kairos::LogChannel::Temporal)
  .value("POSTPROCESS", kairos::LogChannel::Postprocess)
  .value("VALIDATE", kairos::LogChannel::Validate)
  .value("ORCHESTRATOR", kairos::LogChannel::Orchestrator)
  .value("MEMORY", kairos::LogChannel::Memory);

  nb::class_<kairos::SolveOptions>(m, "SolveOptions")
  .def(nb::init<>())
  .def_prop_rw("timeout",
    [](const kairos::SolveOptions & o) {return to_s(o.timeout);},
    [](kairos::SolveOptions & o, double s) {o.timeout = to_ms(s);}, "Seconds.")
  .def_rw("memory_limit_mb", &kairos::SolveOptions::memory_limit_mb,
    "0: half of what is available.")
  .def_rw("anytime", &kairos::SolveOptions::anytime)
  .def_rw("engine", &kairos::SolveOptions::engine)
  .def_rw("search", &kairos::SolveOptions::search)
  .def_rw("threads", &kairos::SolveOptions::threads)
  .def_rw("epsilon", &kairos::SolveOptions::epsilon)
  .def_rw("seed", &kairos::SolveOptions::seed)
  .def_prop_rw("log_level",
    [](const kairos::SolveOptions & o) {return o.log.level;},
    [](kairos::SolveOptions & o, kairos::LogLevel l) {o.log.level = l;})
  .def_rw("dump_dir", &kairos::SolveOptions::dump_dir)
  .def_rw("dump", &kairos::SolveOptions::dump);
  nb::class_<kairos::SearchOptions, kairos::SolveOptions>(m, "SearchOptions")
  .def(nb::init<>())
  .def_prop_rw("stop_after_no_improvement",
    [](const kairos::SearchOptions & o) -> std::optional<double> {
      return o.stop_after_no_improvement ?
             std::optional<double>(to_s(*o.stop_after_no_improvement)) :
             std::nullopt;
    },
    [](kairos::SearchOptions & o, std::optional<double> s) {
      o.stop_after_no_improvement = s ? std::optional<Millis>(to_ms(*s)) : std::nullopt;
    }, "Seconds without a better plan before stopping (None: never).")
  .def_rw("target_cost", &kairos::SearchOptions::target_cost);

  nb::class_<kairos::Statistics>(m, "Statistics")
  .def_ro("parse_ms", &kairos::Statistics::parse_ms)
  .def_ro("normalize_ms", &kairos::Statistics::normalize_ms)
  .def_ro("ground_ms", &kairos::Statistics::ground_ms)
  .def_ro("sas_ms", &kairos::Statistics::sas_ms)
  .def_ro("search_ms", &kairos::Statistics::search_ms)
  .def_ro("total_ms", &kairos::Statistics::total_ms)
  .def_ro("first_plan_ms", &kairos::Statistics::first_plan_ms)
  .def_ro("atoms", &kairos::Statistics::atoms)
  .def_ro("actions", &kairos::Statistics::actions)
  .def_ro("durative_actions", &kairos::Statistics::durative_actions)
  .def_ro("variables", &kairos::Statistics::variables)
  .def_ro("mutex_groups", &kairos::Statistics::mutex_groups)
  .def_ro("expanded", &kairos::Statistics::expanded)
  .def_ro("generated", &kairos::Statistics::generated)
  .def_ro("evaluated", &kairos::Statistics::evaluated)
  .def_ro("states", &kairos::Statistics::states)
  .def_ro("plans", &kairos::Statistics::plans)
  .def_ro("memory_limit_bytes", &kairos::Statistics::memory_limit_bytes)
  .def_ro("peak_memory_bytes", &kairos::Statistics::peak_memory_bytes)
  .def_ro("bytes_per_state", &kairos::Statistics::bytes_per_state);

  nb::class_<kairos::SolveResult>(m, "SolveResult")
  .def_ro("status", &kairos::SolveResult::status)
  .def_ro("plan", &kairos::SolveResult::plan)
  .def_ro("stop_reason", &kairos::SolveResult::stop_reason)
  .def_ro("diagnostics", &kairos::SolveResult::diagnostics)
  .def_ro("stats", &kairos::SolveResult::stats)
  .def("__repr__", [](const kairos::SolveResult & r) {
      return "<SolveResult " + std::string(kairos::to_string(r.status)) + " (" +
             std::string(kairos::to_string(r.stop_reason)) + ")>";
    });

  nb::class_<PySearch>(m, "Search",
    "An anytime search in its own thread; stopped when it goes away.")
  .def("wait_first", [](PySearch & s, double timeout) {
      nb::gil_scoped_release release;
      return s.search.wait_first(to_ms(timeout));
    }, "timeout"_a, "The first plan, waiting at most `timeout` seconds (None if there is none).")
  .def("next_plan", [](PySearch & s, double timeout) {
      nb::gil_scoped_release release;
      return s.search.next_plan(to_ms(timeout));
    }, "timeout"_a, "The next better plan not returned yet, waiting at most `timeout` seconds.")
  .def("best", [](PySearch & s) {return s.search.best();})
  .def("running", [](PySearch & s) {return s.search.running();})
  .def("stop", [](PySearch & s) {
      nb::gil_scoped_release release;
      s.search.stop();
    })
  .def("wait", [](PySearch & s) {
      return interruptible([&s](std::stop_token) {return s.search.wait();});
    }, "Waits until the search ends; its result.")
  .def_prop_ro("stop_reason", [](PySearch & s) {return s.search.stop_reason();});

  nb::class_<kairos::Planner>(m, "_Planner")
  .def(nb::init<>())
  .def("solve", [](kairos::Planner & planner, const kairos::Problem & problem,
    kairos::SolveOptions options, nb::object on_log) {
      Callable log = hold(std::move(on_log));
      attach_log(options.log, log);
      return interruptible([&](std::stop_token stop) {
               return planner.solve(problem, options, stop);
             });
    }, "problem"_a, "options"_a = kairos::SolveOptions(), "on_log"_a = nb::none())
  .def("start", [](kairos::Planner & planner, const kairos::Problem & problem,
    kairos::SearchOptions options, nb::object on_plan, nb::object on_log) {
      auto s = std::make_unique<PySearch>();
      s->on_plan = hold(std::move(on_plan));
      s->on_log = hold(std::move(on_log));
      attach_log(options.log, s->on_log);
      std::function<void(const kairos::PlanUpdate &)> cb;
      if (s->on_plan) {
        cb = [f = s->on_plan](const kairos::PlanUpdate & u) {
          nb::gil_scoped_acquire gil;
          try {
            (*f)(u.plan, u.index, to_s(u.elapsed), u.proven_optimal);
          } catch (nb::python_error & e) {
            e.discard_as_unraisable("kairos on_plan callback");
          }
        };
      }
      s->search = planner.start(problem, options, std::move(cb));
      return s;
    }, "problem"_a, "options"_a = kairos::SearchOptions(), "on_plan"_a = nb::none(),
    "on_log"_a = nb::none());
}
