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

cmake_minimum_required(VERSION 3.22)

project(kairos
  VERSION 1.0.2
  DESCRIPTION "Modern temporal and numeric PDDL planner"
  LANGUAGES CXX)

# The version lives in package.xml and pixi.toml too
foreach(manifest package.xml pixi.toml)
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/${manifest}" _text)
  if(manifest STREQUAL "package.xml")
    string(REGEX MATCH "<version>([^<]+)</version>" _found "${_text}")
  else()
    string(REGEX MATCH "\nversion = \"([^\"]+)\"" _found "${_text}")
  endif()
  if(NOT CMAKE_MATCH_1 STREQUAL PROJECT_VERSION)
    message(FATAL_ERROR
      "${manifest} has version '${CMAKE_MATCH_1}', CMakeLists.txt has ${PROJECT_VERSION}")
  endif()
endforeach()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# The tests follow BUILD_TESTING when it is given (packaging builds turn it off: nothing to
# download, no lint)
if(DEFINED BUILD_TESTING)
  set(_kairos_tests_default ${BUILD_TESTING})
else()
  set(_kairos_tests_default ON)
endif()
option(KAIROS_BUILD_TESTS "Build the gtest suites and lint tests" ${_kairos_tests_default})
option(KAIROS_BUILD_CLI "Build the kairos executable" ON)
option(KAIROS_BUILD_PYTHON "Build the Python module (nanobind)" OFF)
option(KAIROS_BUILD_FUZZERS "Build libFuzzer targets (requires Clang)" OFF)
option(KAIROS_COVERAGE "Instrument for coverage" OFF)
option(KAIROS_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
set(KAIROS_SANITIZE "" CACHE STRING "Sanitizers: empty, 'address;undefined' or 'thread'")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(KairosCompilerOptions)
# A ROS install (the buildfarm's packages, /opt/ros/DISTRO) finds libraries in lib only, not in
# lib64 (RHEL, Fedora) or lib/<multiarch> (Debian)
if(CMAKE_INSTALL_PREFIX MATCHES "^/opt/ros/")
  set(CMAKE_INSTALL_LIBDIR lib)  # over the cache (packaging tools pass their own)
endif()
include(GNUInstallDirs)

add_subdirectory(src)

if(KAIROS_BUILD_TESTS)
  include(CTest)  # also writes DartConfiguration.tcl, expected by colcon test
  add_subdirectory(tests)
  include(KairosLint)
endif()

if(KAIROS_BUILD_PYTHON)
  add_subdirectory(python)
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
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/kairosConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/kairosConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kairos)
