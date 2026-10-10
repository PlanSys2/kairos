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

#include "semantic/features.hpp"

#include <array>
#include <string>
#include <vector>

namespace kairos::sem
{

namespace
{

constexpr size_t kCount = static_cast<size_t>(Feature::Count);

constexpr std::array<std::string_view, kCount> kNames = {
  "typing", "negative-preconditions", "disjunctive-preconditions", "equality",
  "existential-preconditions", "universal-preconditions", "conditional-effects",
  "numeric-fluents", "object-fluents", "non-linear-numeric", "action-costs",
  "durative-actions", "duration-inequalities", "variable-duration", "duration-in-effects",
  "continuous-effects", "derived-predicates", "timed-initial-literals",
  "timed-initial-fluents", "preferences", "trajectory-constraints", "processes", "events",
  "global-constraints", "metric", "metric-total-time", "metric-fluents"};

}  // namespace

std::vector<Feature> FeatureSet::list() const
{
  std::vector<Feature> out;
  for (size_t i = 0; i < kCount; ++i) {
    if (contains(static_cast<Feature>(i))) {
      out.push_back(static_cast<Feature>(i));
    }
  }
  return out;
}

std::vector<std::string> FeatureSet::names() const
{
  std::vector<std::string> out;
  for (Feature f : list()) {
    out.emplace_back(to_string(f));
  }
  return out;
}

std::string_view to_string(Feature f)
{
  auto i = static_cast<size_t>(f);
  return i < kCount ? kNames[i] : std::string_view("count");
}

std::optional<Feature> parse_feature(std::string_view name)
{
  for (size_t i = 0; i < kCount; ++i) {
    if (kNames[i] == name) {
      return static_cast<Feature>(i);
    }
  }
  return std::nullopt;
}

std::vector<std::string_view> covering_requirements(Feature f)
{
  switch (f) {
    case Feature::Typing: return {":typing", ":adl"};
    case Feature::NegativePreconditions: return {":negative-preconditions", ":adl"};
    case Feature::DisjunctivePreconditions: return {":disjunctive-preconditions", ":adl"};
    case Feature::Equality: return {":equality", ":adl"};
    case Feature::ExistentialPreconditions:
      return {":existential-preconditions", ":quantified-preconditions", ":adl"};
    case Feature::UniversalPreconditions:
      return {":universal-preconditions", ":quantified-preconditions", ":adl"};
    case Feature::ConditionalEffects: return {":conditional-effects", ":adl"};
    case Feature::NumericFluents: return {":numeric-fluents", ":fluents"};
    case Feature::ObjectFluents: return {":object-fluents", ":fluents"};
    case Feature::NonLinearNumeric: return {":numeric-fluents", ":fluents"};
    case Feature::ActionCosts: return {":action-costs", ":numeric-fluents", ":fluents"};
    case Feature::DurativeActions: return {":durative-actions"};
    case Feature::DurationInequalities: return {":duration-inequalities"};
    case Feature::VariableDuration: return {":durative-actions"};
    case Feature::DurationInEffects: return {":durative-actions"};
    case Feature::ContinuousEffects: return {":continuous-effects", ":time"};
    case Feature::DerivedPredicates: return {":derived-predicates"};
    case Feature::TimedInitialLiterals: return {":timed-initial-literals"};
    case Feature::TimedInitialFluents:
      return {":timed-initial-fluents", ":timed-initial-literals"};
    case Feature::Preferences: return {":preferences"};
    case Feature::TrajectoryConstraints: return {":constraints"};
    case Feature::Processes: return {":time"};
    case Feature::Events: return {":time"};
    case Feature::GlobalConstraints:
    case Feature::Metric:
    case Feature::MetricTotalTime:
    case Feature::MetricFluents:
    case Feature::Count:
      break;
  }
  return {};
}

}  // namespace kairos::sem
