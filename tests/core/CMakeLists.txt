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

kairos_add_test(core_tests
  SOURCES
    budget_test.cpp
    diagnostics_test.cpp
    id_test.cpp
    log_test.cpp
    logger_test.cpp
    memory_info_test.cpp
    result_test.cpp
    suggest_test.cpp
    symbol_table_test.cpp
  DEPENDS
    kairos_core)
