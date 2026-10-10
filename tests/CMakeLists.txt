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

find_package(GTest QUIET)
if(NOT GTest_FOUND)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  add_library(GTest::gtest_main ALIAS gtest_main)
endif()
include(GoogleTest)

# kairos_add_test(<name> SOURCES ... [DEPENDS ...] [LABELS ...])
function(kairos_add_test name)
  cmake_parse_arguments(ARG "" "" "SOURCES;DEPENDS;LABELS" ${ARGN})
  add_executable(${name} ${ARG_SOURCES})
  target_link_libraries(${name} PRIVATE ${ARG_DEPENDS} GTest::gtest_main)
  target_include_directories(${name} PRIVATE ${PROJECT_SOURCE_DIR}/tests)
  kairos_configure_target(${name})
  if(NOT ARG_LABELS)
    set(ARG_LABELS unit)
  endif()
  gtest_discover_tests(${name}
    PROPERTIES LABELS "${ARG_LABELS}"
    DISCOVERY_TIMEOUT 30)
endfunction()

add_subdirectory(core)
add_subdirectory(plan)
add_subdirectory(pddl)
add_subdirectory(semantic)
add_subdirectory(validate)
add_subdirectory(normalize)
add_subdirectory(ground)
add_subdirectory(api)
