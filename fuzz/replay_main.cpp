// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Runs a fuzz target over files and directories (the seed corpus) without libFuzzer, so the
// targets also work as regular tests with any compiler.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size);

namespace
{

void run_file(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary);
  std::vector<char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  LLVMFuzzerTestOneInput(reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size());
}

}  // namespace

int main(int argc, char ** argv)
{
  size_t count = 0;
  for (int i = 1; i < argc; ++i) {
    std::filesystem::path arg(argv[i]);
    if (std::filesystem::is_directory(arg)) {
      for (const auto & entry : std::filesystem::recursive_directory_iterator(arg)) {
        if (entry.is_regular_file()) {
          run_file(entry.path());
          ++count;
        }
      }
    } else {
      run_file(arg);
      ++count;
    }
  }
  if (count == 0) {
    std::cerr << "no inputs found\n";
    return 1;
  }
  std::cout << count << " input(s) replayed\n";
  return 0;
}
