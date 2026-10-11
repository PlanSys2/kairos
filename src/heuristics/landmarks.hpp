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
/// \brief Fact landmarks with greedy-necessary orderings, and the landmark count heuristic.

#ifndef KAIROS__HEURISTICS__LANDMARKS_HPP_
#define KAIROS__HEURISTICS__LANDMARKS_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include "core/budget.hpp"
#include "heuristics/heuristic.hpp"
#include "heuristics/relaxed.hpp"

namespace kairos::heuristics
{

struct Landmark
{
  sas::Fact fact;
  uint32_t node = 0;                  // fact node of the relaxed graph
  double cost = 0.0;                  // cheapest achiever (0 if an axiom achieves it)
  bool goal = false;
  bool initially_true = false;
  std::vector<uint32_t> achievers;    // actions that add it
  std::vector<uint32_t> parents;      // landmarks ordered greedy-necessarily before it
  std::vector<uint32_t> children;
};

/**
 * @brief Fact landmarks of a task from a state, found backwards from the goal as in LAMA's
 * RHW generation (without disjunctive landmarks): for each landmark not true in the state, its
 * first achievers are the achievers reachable without it in the relaxation; the facts all of
 * them need are landmarks, ordered greedy-necessarily before it.
 *
 * Every landmark is true at some point of every plan from the state (sound). Numeric
 * conditions and negated facts are crossed but never become landmarks. An ordering a < b means
 * a holds just before b first does, or at the same time when b is derived (an axiom makes b
 * true in the state where its body holds).
 */
class LandmarkGraph
{
public:
  static LandmarkGraph build(
    const sas::Task & task, RelaxedGraph & graph, const EvalContext & state,
    const std::vector<double> & costs, Budget * budget = nullptr);

  const std::vector<Landmark> & landmarks() const {return landmarks_;}
  size_t orderings() const;

private:
  std::vector<Landmark> landmarks_;
};

/**
 * @brief The landmark count heuristic (LAMA): the cost of the landmarks not reached yet, plus
 * those reached but needed again (false now, and a goal or ordered before one not reached).
 *
 * Path dependent: a landmark is reached on a path when it holds after a transition and every
 * landmark ordered before it was reached; a state reached on several paths keeps what all of
 * them reached. Preferred operators: applicable achievers of landmarks not reached whose
 * parents are, or needed again.
 */
std::unique_ptr<Heuristic> make_landmark_count(
  const sas::Task & task, const std::vector<double> & costs, Budget * budget = nullptr);

}  // namespace kairos::heuristics

#endif  // KAIROS__HEURISTICS__LANDMARKS_HPP_
