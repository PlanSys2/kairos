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

cmake_minimum_required(VERSION 3.24)

project(kairos
  VERSION 0.1.0
  DESCRIPTION "Modern temporal and numeric PDDL planner"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(KAIROS_BUILD_TESTS "Build the gtest suites and lint tests" ON)
option(KAIROS_BUILD_CLI "Build the kairos executable" ON)
option(KAIROS_WITH_HIGHS "Use HiGHS for LP-based temporal reasoning" OFF)
option(KAIROS_BUILD_FUZZERS "Build libFuzzer targets (requires Clang)" OFF)
option(KAIROS_COVERAGE "Instrument for coverage" OFF)
option(KAIROS_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
set(KAIROS_SANITIZE "" CACHE STRING "Sanitizers: empty, 'address;undefined' or 'thread'")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(KairosCompilerOptions)
include(GNUInstallDirs)

add_subdirectory(src)

if(KAIROS_BUILD_TESTS)
  include(CTest)  # also writes DartConfiguration.tcl, expected by colcon test
  add_subdirectory(tests)
  include(KairosLint)
endif()

if(KAIROS_BUILD_TESTS OR KAIROS_BUILD_FUZZERS)
  add_subdirectory(fuzz)
endif()

# Installation and package config
include(CMakePackageConfigHelpers)

install(DIRECTORY include/kairos DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  PATTERN "*.in" EXCLUDE)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/include/kairos/version.hpp
  ${CMAKE_CURRENT_BINARY_DIR}/include/kairos/export.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kairos)

install(EXPORT kairosTargets
  NAMESPACE kairos::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kairos)

configure_package_config_file(cmake/kairosConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/kairosConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kairos)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/kairosConfigVersion.cmake
  COMPATIBILITY SameMinorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/kairosConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/kairosConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kairos)
