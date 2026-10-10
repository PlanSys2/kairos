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

# cpplint and uncrustify as ctest tests (label "lint"), with the ament rules.

set(_lint_dirs
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/tests)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
  add_test(NAME cpplint
    COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tools/lint/kairos_cpplint.py ${_lint_dirs})
  set_tests_properties(cpplint PROPERTIES LABELS lint)
else()
  message(WARNING "python3 not found: the cpplint test is disabled")
endif()

find_program(KAIROS_UNCRUSTIFY uncrustify)
if(KAIROS_UNCRUSTIFY)
  # Glob at configure time; re-run cmake after adding files
  file(GLOB_RECURSE _lint_files CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/include/*.hpp
    ${PROJECT_SOURCE_DIR}/src/*.hpp ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/tests/*.hpp ${PROJECT_SOURCE_DIR}/tests/*.cpp)
  add_test(NAME uncrustify
    COMMAND ${KAIROS_UNCRUSTIFY} -c ${PROJECT_SOURCE_DIR}/tools/lint/uncrustify.cfg
      --check -q ${_lint_files})
  set_tests_properties(uncrustify PROPERTIES LABELS lint)
else()
  message(WARNING "uncrustify not found: the uncrustify test is disabled")
endif()
