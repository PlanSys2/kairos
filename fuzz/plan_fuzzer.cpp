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

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string_view>

#include "kairos/plan.hpp"

namespace
{

[[noreturn]] void fail(const char * what)
{
  std::fprintf(stderr, "plan reader invariant violated: %s\n", what);
  std::abort();
}

}  // namespace

// A plan that reads must print in every format, and its IPC text must read back to the same
// actions in the same order
extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size)
{
  std::string_view text(reinterpret_cast<const char *>(data), size);
  auto plan = kairos::parse_plan(text);
  if (!plan.ok()) {
    if (!kairos::has_errors(plan.diagnostics())) {
      fail("failure without an error");
    }
    return 0;
  }
  (void)kairos::to_string(*plan, kairos::PlanFormat::Popf);
  (void)kairos::to_string(*plan, kairos::PlanFormat::Json);
  auto again = kairos::parse_plan(kairos::to_string(*plan, kairos::PlanFormat::Ipc, 9));
  if (!again.ok()) {
    fail("printed plan does not read");
  }
  if (again->steps.size() != plan->steps.size() || again->temporal != plan->temporal) {
    fail("printed plan reads differently");
  }
  for (size_t i = 0; i < plan->steps.size(); ++i) {
    if (again->steps[i].action() != plan->steps[i].action()) {
      fail("actions changed");
    }
  }
  return 0;
}
