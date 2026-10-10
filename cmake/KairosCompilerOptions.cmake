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

# Warnings, sanitizers and coverage for every Kairos target.
function(kairos_configure_target target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE
      -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
      -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual)
    if(KAIROS_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()

  if(KAIROS_SANITIZE)
    string(REPLACE ";" "," _sanitizers "${KAIROS_SANITIZE}")
    target_compile_options(${target} PRIVATE -fsanitize=${_sanitizers} -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=${_sanitizers})
  endif()

  # Fuzzing: instrument library code too, so libFuzzer gets coverage feedback from it
  if(KAIROS_BUILD_FUZZERS)
    target_compile_options(${target} PRIVATE -fsanitize=fuzzer-no-link,address,undefined -g)
    target_link_options(${target} PRIVATE -fsanitize=address,undefined)
  endif()

  if(KAIROS_COVERAGE)
    target_compile_options(${target} PRIVATE --coverage -O0 -g)
    target_link_options(${target} PRIVATE --coverage)
  endif()
endfunction()
