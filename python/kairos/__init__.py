# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Kairos, a temporal and numeric PDDL planner, in Python.

    import kairos
    domain = kairos.parse_domain(domain_text)
    problem = kairos.parse_problem(problem_text, domain)
    result = kairos.Planner().solve(problem, timeout=10)
    if result.status == kairos.Status.SOLVED:
        print(result.plan)

PDDL errors raise PddlError, with the diagnostics (file, line, column) of each.
"""

from datetime import timedelta
from os import PathLike
from typing import Callable, Iterator, Optional, Union

from ._kairos import (
    CheckReport,
    Diagnostic,
    Domain,
    Engine,
    FeatureSupport,
    LogChannel,
    LogLevel,
    Plan,
    PlanFormat,
    PlanStep,
    Problem,
    Search,
    SearchOptions,
    Severity,
    SolveOptions,
    SolveResult,
    SourceLocation,
    Statistics,
    Status,
    StopReason,
    SupportLevel,
    ValidateOptions,
    ValidationReport,
    ValidationStatus,
    _Planner,
    check_domain,
    check_problem,
    pddl_support,
    search_presets,
    validate,
    validate_plan,
    version,
)
from . import _kairos

__version__ = version()

__all__ = [
    "CheckReport", "Diagnostic", "Domain", "Engine", "FeatureSupport", "LogChannel", "LogLevel",
    "PddlError", "Plan", "PlanFormat", "PlanStep", "Planner", "Problem", "Search",
    "SearchOptions", "Severity", "SolveOptions", "SolveResult", "SourceLocation", "Statistics",
    "Status", "StopReason", "SupportLevel", "ValidateOptions", "ValidationReport",
    "ValidationStatus", "check_domain", "check_problem", "load_domain", "load_problem",
    "parse_domain", "parse_plan", "parse_problem", "pddl_support", "search_presets", "solve",
    "validate", "validate_plan", "version",
]


class PddlError(ValueError):
    """The domain, problem or plan has errors; `diagnostics` says where and why."""

    def __init__(self, diagnostics):
        self.diagnostics = list(diagnostics)
        errors = [d for d in self.diagnostics if d.severity == Severity.ERROR]
        super().__init__("\n".join(str(d) for d in (errors or self.diagnostics)))


def _value(pair):
    value, diagnostics = pair
    if value is None:
        raise PddlError(diagnostics)
    return value


def parse_domain(text: str, filename: str = "domain.pddl") -> Domain:
    """Parses and checks a domain; raises PddlError on errors."""
    return _value(_kairos._parse_domain(text, filename))


def parse_problem(text: str, domain: Domain, filename: str = "problem.pddl") -> Problem:
    """Parses and checks a problem against its domain; raises PddlError on errors."""
    return _value(_kairos._parse_problem(text, domain, filename))


def load_domain(file: Union[str, PathLike]) -> Domain:
    return _value(_kairos._load_domain(file))


def load_problem(file: Union[str, PathLike], domain: Domain) -> Problem:
    return _value(_kairos._load_problem(file, domain))


def parse_plan(text: str, filename: str = "plan") -> Plan:
    """Reads a plan (IPC/VAL, sequential, or the whole output of POPF or OPTIC)."""
    return _value(_kairos._parse_plan(text, filename))


def _seconds(value):
    return value.total_seconds() if isinstance(value, timedelta) else float(value)


def _options(cls, options, kwargs):
    """Options from an options object and keywords (timeout=10, engine="m3", ...)."""
    o = cls()
    if options is not None:
        for name in dir(options):
            if not name.startswith("_") and hasattr(o, name) and \
                    not callable(getattr(options, name)):
                setattr(o, name, getattr(options, name))
    for name, value in kwargs.items():
        if not hasattr(o, name) or name.startswith("_"):
            raise TypeError(f"unknown option '{name}'")
        if name in ("timeout", "stop_after_no_improvement") and value is not None:
            value = _seconds(value)
        elif name == "engine" and isinstance(value, str):
            value = Engine[value.upper()]
        elif name == "log_level" and isinstance(value, str):
            value = LogLevel[value.upper()]
        setattr(o, name, value)
    return o


LogCallback = Callable[[LogLevel, LogChannel, str], None]
PlanCallback = Callable[[Plan, int, float, bool], None]


class Planner:
    """Plans problems; options as an object or as keywords (timeout in seconds)."""

    def __init__(self):
        self._planner = _Planner()

    def solve(self, problem: Problem, options: Optional[SolveOptions] = None, *,
              on_log: Optional[LogCallback] = None, **kwargs) -> SolveResult:
        """Plans until the timeout, a proof or (anytime) the optimum; the best plan.

        Ctrl-C stops it and raises KeyboardInterrupt.
        """
        return self._planner.solve(problem, _options(SolveOptions, options, kwargs), on_log)

    def start(self, problem: Problem, options: Optional[SearchOptions] = None, *,
              on_plan: Optional[PlanCallback] = None, on_log: Optional[LogCallback] = None,
              **kwargs) -> Search:
        """An anytime search in its own thread.

        on_plan(plan, index, elapsed_seconds, proven_optimal) runs in that thread for each
        better plan.
        """
        return self._planner.start(problem, _options(SearchOptions, options, kwargs), on_plan,
                                   on_log)

    def plans(self, problem: Problem, options: Optional[SearchOptions] = None, *,
              on_log: Optional[LogCallback] = None, **kwargs) -> Iterator[Plan]:
        """Each better plan as it comes; leaving the loop stops the search."""
        search = self.start(problem, options, on_log=on_log, **kwargs)
        try:
            yield from _search_plans(search)
        finally:
            search.stop()


def _search_plans(search: Search, poll: float = 0.1) -> Iterator[Plan]:
    """Each better plan the search finds, until it ends."""
    while True:
        plan = search.next_plan(poll)
        if plan is not None:
            yield plan
        elif not search.running():
            plan = search.next_plan(0)
            if plan is None:
                return
            yield plan


Search.plans = _search_plans


def solve(domain: Union[str, PathLike, Domain], problem: Union[str, PathLike, Problem],
          **kwargs) -> SolveResult:
    """Plans a problem given as texts, files or parsed objects; keywords as Planner.solve."""
    if not isinstance(domain, Domain):
        domain = _read(domain, load_domain, parse_domain)
    if not isinstance(problem, Problem):
        problem = _read(problem, lambda f: load_problem(f, domain),
                        lambda t: parse_problem(t, domain))
    return Planner().solve(problem, **kwargs)


def _read(source, load, parse):
    if isinstance(source, PathLike) or ("(" not in str(source)):
        return load(source)
    return parse(source)
