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

# Uses the internal normalizer and the public validator (as an independent judge)
kairos_add_test(normalize_tests
  SOURCES
    normalizer_test.cpp
  DEPENDS
    kairos_normalize
    kairos_validate)
target_sources(normalize_tests PRIVATE ${PROJECT_SOURCE_DIR}/src/api/validate.cpp)
target_compile_definitions(normalize_tests PRIVATE
  KAIROS_TEST_DATA_DIR="${PROJECT_SOURCE_DIR}/tests/data")
