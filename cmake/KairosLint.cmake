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
  ${PROJECT_SOURCE_DIR}/tests
  ${PROJECT_SOURCE_DIR}/fuzz)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
  add_test(NAME cpplint
    COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tools/lint/kairos_cpplint.py ${_lint_dirs})
  set_tests_properties(cpplint PROPERTIES LABELS lint)
else()
  message(WARNING "python3 not found: the cpplint test is disabled")
endif()

find_program(KAIROS_UNCRUSTIFY uncrustify)
# The configuration uses options of uncrustify 0.78 (older ones reject it: Ubuntu 22.04 has 0.72)
set(_uncrustify_min 0.78)
if(KAIROS_UNCRUSTIFY)
  execute_process(COMMAND ${KAIROS_UNCRUSTIFY} --version OUTPUT_VARIABLE _uncrustify_out
    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  string(REGEX MATCH "[0-9]+\\.[0-9]+(\\.[0-9]+)?" _uncrustify_version "${_uncrustify_out}")
  if(_uncrustify_version VERSION_LESS _uncrustify_min)
    message(WARNING "uncrustify ${_uncrustify_version} is older than ${_uncrustify_min}: "
      "the uncrustify test is disabled")
    unset(KAIROS_UNCRUSTIFY CACHE)
    set(KAIROS_UNCRUSTIFY "")
  endif()
endif()
if(KAIROS_UNCRUSTIFY)
  # Glob at configure time; re-run cmake after adding files
  file(GLOB_RECURSE _lint_files CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/include/*.hpp
    ${PROJECT_SOURCE_DIR}/src/*.hpp ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/tests/*.hpp ${PROJECT_SOURCE_DIR}/tests/*.cpp
    ${PROJECT_SOURCE_DIR}/fuzz/*.cpp)
  add_test(NAME uncrustify
    COMMAND ${KAIROS_UNCRUSTIFY} -c ${PROJECT_SOURCE_DIR}/tools/lint/uncrustify.cfg
      --check -q ${_lint_files})
  set_tests_properties(uncrustify PROPERTIES LABELS lint)
else()
  message(WARNING "uncrustify not found: the uncrustify test is disabled")
endif()
