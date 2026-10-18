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

# kairos._kairos (nanobind, stable ABI from Python 3.12) and, in a wheel, the kairos command
find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module
  OPTIONAL_COMPONENTS Development.SABIModule)
execute_process(COMMAND ${Python_EXECUTABLE} -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(_kairos STABLE_ABI NB_STATIC src/kairos_python.cpp)
target_link_libraries(_kairos PRIVATE kairos::kairos)
target_compile_features(_kairos PRIVATE cxx_std_20)

# The package next to the module, for tests from the build tree
file(GLOB _kairos_py ${CMAKE_CURRENT_SOURCE_DIR}/kairos/*.py ${CMAKE_CURRENT_SOURCE_DIR}/kairos/py.typed)
set(_kairos_pkg ${CMAKE_CURRENT_BINARY_DIR}/kairos)
set_target_properties(_kairos PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${_kairos_pkg})
foreach(f ${_kairos_py})
  get_filename_component(n ${f} NAME)
  configure_file(${f} ${_kairos_pkg}/${n} COPYONLY)
endforeach()

if(DEFINED SKBUILD_PROJECT_NAME)
  install(TARGETS _kairos LIBRARY DESTINATION kairos COMPONENT python)
  if(KAIROS_BUILD_CLI)
    install(TARGETS kairos_cli RUNTIME DESTINATION ${SKBUILD_SCRIPTS_DIR} COMPONENT python)
  endif()
endif()

if(KAIROS_BUILD_TESTS)
  add_test(NAME python_tests
    COMMAND ${Python_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  set_tests_properties(python_tests PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR};KAIROS_EXAMPLES_DIR=${PROJECT_SOURCE_DIR}/docs/examples/pddls"
    LABELS python)
endif()
