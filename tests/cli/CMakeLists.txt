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

# Runs the kairos executable: exit codes, stdout and stderr
kairos_add_test(cli_tests
  SOURCES
    cli_test.cpp
  DEPENDS
    kairos::kairos)
add_dependencies(cli_tests kairos_cli)
target_compile_definitions(cli_tests PRIVATE
  KAIROS_CLI="$<TARGET_FILE:kairos_cli>"
  KAIROS_TEST_DATA_DIR="${PROJECT_SOURCE_DIR}/tests/data"
  KAIROS_EXAMPLES_DIR="${PROJECT_SOURCE_DIR}/docs/examples/pddls")
