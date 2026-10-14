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

include(GenerateExportHeader)

option(KAIROS_SHARED "Build libkairos as a shared library" ON)

set(KAIROS_GENERATED_INCLUDE_DIR ${PROJECT_BINARY_DIR}/include)

# Public facade. Internal modules are static libraries linked into it.
if(KAIROS_SHARED)
  add_library(kairos SHARED)
else()
  add_library(kairos STATIC)
endif()
add_library(kairos::kairos ALIAS kairos)

generate_export_header(kairos
  BASE_NAME KAIROS
  EXPORT_MACRO_NAME KAIROS_API
  EXPORT_FILE_NAME ${KAIROS_GENERATED_INCLUDE_DIR}/kairos/export.hpp)
configure_file(${PROJECT_SOURCE_DIR}/include/kairos/version.hpp.in
  ${KAIROS_GENERATED_INCLUDE_DIR}/kairos/version.hpp @ONLY)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)

# Adds an internal module: its objects (compiled once; libkairos is made of them) and a static
# library of them for the tests, with src/ and the public headers in its include path
function(kairos_add_module name)
  cmake_parse_arguments(ARG "" "" "SOURCES;DEPENDS" ${ARGN})
  add_library(${name}_objects OBJECT ${ARG_SOURCES})
  add_library(${name} STATIC $<TARGET_OBJECTS:${name}_objects>)
  foreach(target ${name}_objects ${name})
    target_include_directories(${target} PUBLIC
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${KAIROS_GENERATED_INCLUDE_DIR}>
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>)
    target_compile_definitions(${target} PUBLIC
      $<$<NOT:$<BOOL:${KAIROS_SHARED}>>:KAIROS_STATIC_DEFINE>)
    kairos_configure_target(${target})
  endforeach()
  # Public symbols defined in modules are exported from libkairos
  target_compile_definitions(${name}_objects PRIVATE kairos_EXPORTS)
  target_link_libraries(${name}_objects PUBLIC Threads::Threads)
  target_link_libraries(${name} PUBLIC ${ARG_DEPENDS})
  set_property(GLOBAL APPEND PROPERTY KAIROS_MODULES ${name})
  # Only a static libkairos needs the module archives at link time
  if(NOT KAIROS_SHARED)
    install(TARGETS ${name} EXPORT kairosTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
  endif()
endfunction()

add_subdirectory(core)
add_subdirectory(plan)
add_subdirectory(pddl)
add_subdirectory(semantic)
add_subdirectory(validate)
add_subdirectory(normalize)
add_subdirectory(ground)
add_subdirectory(sas)
add_subdirectory(state)
add_subdirectory(heuristics)
add_subdirectory(search)
add_subdirectory(temporal)
add_subdirectory(lift)
add_subdirectory(orchestrator)

target_sources(kairos PRIVATE api/check.cpp api/planner.cpp api/validate.cpp api/version.cpp)
target_include_directories(kairos PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${KAIROS_GENERATED_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(kairos PRIVATE ${PROJECT_SOURCE_DIR}/src)
if(NOT KAIROS_SHARED)
  target_compile_definitions(kairos PUBLIC KAIROS_STATIC_DEFINE)
endif()
# The objects of every module: public functions defined in modules end up in libkairos.so
# (no whole-archive linking, which CMake before 3.30 cannot mix with the modules' own links)
get_property(KAIROS_MODULES GLOBAL PROPERTY KAIROS_MODULES)
if(KAIROS_SHARED)
  foreach(module ${KAIROS_MODULES})
    target_sources(kairos PRIVATE $<TARGET_OBJECTS:${module}_objects>)
  endforeach()
  target_link_libraries(kairos PRIVATE Threads::Threads)
else()
  target_link_libraries(kairos PRIVATE ${KAIROS_MODULES})
endif()
set_target_properties(kairos PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR})
kairos_configure_target(kairos)

install(TARGETS kairos EXPORT kairosTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_subdirectory(cli)
