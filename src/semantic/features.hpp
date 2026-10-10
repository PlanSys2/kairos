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
/// \brief FeatureSet: the PDDL features a domain and problem actually use.

#ifndef KAIROS__SEMANTIC__FEATURES_HPP_
#define KAIROS__SEMANTIC__FEATURES_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace kairos::sem
{

enum class Feature : uint8_t
{
  Typing,                    // typed parameters or objects
  NegativePreconditions,     // an atom under negative polarity in a condition
  DisjunctivePreconditions,  // or / imply (or and under negation)
  Equality,                  // (= t1 t2) between objects
  ExistentialPreconditions,  // exists (or forall under negation)
  UniversalPreconditions,    // forall (or exists under negation)
  ConditionalEffects,        // when, or forall in effects
  NumericFluents,
  ObjectFluents,
  NonLinearNumeric,          // products/divisions of changing quantities, ^
  ActionCosts,               // total-cost increased by actions
  DurativeActions,
  DurationInequalities,      // ?duration bounded by <= / >=
  VariableDuration,          // duration depends on fluents
  DurationInEffects,         // ?duration used in effects or conditions
  ContinuousEffects,         // #t in durative actions or processes
  DerivedPredicates,
  TimedInitialLiterals,
  TimedInitialFluents,
  Preferences,
  TrajectoryConstraints,     // :constraints with modal operators
  Processes,
  Events,
  GlobalConstraints,         // ENHSP :constraint
  Metric,                    // a :metric section
  MetricTotalTime,           // the metric uses total-time
  MetricFluents,             // the metric uses fluents
  Count                      // number of features, not a feature
};

/// @brief Set of features, as a bit mask.
class FeatureSet
{
public:
  void add(Feature f) {bits_ |= bit(f);}
  bool contains(Feature f) const {return (bits_ & bit(f)) != 0;}
  bool empty() const {return bits_ == 0;}

  void merge(const FeatureSet & other) {bits_ |= other.bits_;}

  /// @brief Features in this set and not in `other`.
  FeatureSet minus(const FeatureSet & other) const
  {
    FeatureSet r;
    r.bits_ = bits_ & ~other.bits_;
    return r;
  }

  std::vector<Feature> list() const;
  std::vector<std::string> names() const;

  bool operator==(const FeatureSet &) const = default;

private:
  static uint64_t bit(Feature f) {return uint64_t{1} << static_cast<uint8_t>(f);}

  uint64_t bits_ = 0;
};

/// @brief Stable lower-case name, e.g. "durative-actions" (used in diagnostics and reports).
std::string_view to_string(Feature f);

std::optional<Feature> parse_feature(std::string_view name);

/**
 * @brief Requirements that cover a feature, e.g. ":negative-preconditions" or ":adl".
 *
 * Empty for features that need no requirement (Metric, MetricTotalTime, MetricFluents).
 */
std::vector<std::string_view> covering_requirements(Feature f);

}  // namespace kairos::sem

#endif  // KAIROS__SEMANTIC__FEATURES_HPP_
