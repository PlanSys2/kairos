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

#include <iostream>

#include "kairos/kairos.hpp"

// Exit code 0 only if the installed headers and library agree
int main()
{
  if (kairos::version() != KAIROS_VERSION_STRING) {
    std::cerr << "header/library version mismatch\n";
    return 1;
  }
  auto d = kairos::Diagnostic::error("e", "msg");
  if (!kairos::has_errors({d})) {
    return 1;
  }
  std::cout << "kairos " << kairos::version() << " OK\n";
  return 0;
}
