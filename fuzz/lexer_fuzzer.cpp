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

#include "support/lexer_invariants.hpp"

// Aborts on any violated invariant, so the fuzzer reports it as a crash
extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size)
{
  std::string_view source(reinterpret_cast<const char *>(data), size);
  if (auto violation = kairos::pddl::testing::check_lexer_invariants(source)) {
    std::fprintf(stderr, "lexer invariant violated: %s\n", violation->c_str());
    std::abort();
  }
  return 0;
}
