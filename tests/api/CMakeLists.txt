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

# Links only against the public library: checks what users actually get
kairos_add_test(api_tests
  SOURCES
    check_test.cpp
    examples_test.cpp
    planner_test.cpp
    public_api_test.cpp
  DEPENDS
    kairos::kairos)
target_compile_definitions(api_tests PRIVATE
  KAIROS_EXAMPLES_DIR="${PROJECT_SOURCE_DIR}/docs/examples/pddls")
