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

kairos_add_test(sas_tests
  SOURCES
    invariants_test.cpp
    translate_test.cpp
  DEPENDS
    kairos_sas)
target_compile_definitions(sas_tests PRIVATE
  KAIROS_TEST_DATA_DIR="${PROJECT_SOURCE_DIR}/tests/data"
  KAIROS_FD_GROUPS_SCRIPT="${CMAKE_CURRENT_SOURCE_DIR}/fd_groups.py"
  KAIROS_FD_CHECK_SCRIPT="${CMAKE_CURRENT_SOURCE_DIR}/fd_check_invariants.py")
