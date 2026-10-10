#!/usr/bin/env bash
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

# Downloads the public PDDL collections used by the external corpus tests, at pinned commits.
# They are not stored in the repository (most have no explicit license).
#
#   tools/fetch_corpus.sh [DIR]          default DIR: ~/.cache/kairos/corpus
#   KAIROS_PDDL_CORPUS=DIR ctest --test-dir build -R ExternalCorpus

set -euo pipefail

dest="${1:-$HOME/.cache/kairos/corpus}"
mkdir -p "$dest"

# name  url  commit
repos=(
  "potassco_pddl-instances https://github.com/potassco/pddl-instances cf19edf7c53d1540ddbb396c642595e0926ee552"
  "ipc2023-classical_ipc2023-dataset https://github.com/ipc2023-classical/ipc2023-dataset 1e29a0e38112263df7b48be677c0ca4337295bbe"
  "ipc2023-numeric_ipc2023-dataset https://github.com/ipc2023-numeric/ipc2023-dataset 0799944d7135ceff9d4b504db8f4ee15ffb7c9fc"
  "ipc2026-numeric_ipc2026-dataset https://github.com/ipc2026-numeric/ipc2026-dataset 85195dbbcccd90ef6359605e43baf7ceb4b5a786"
  "hstairs_pddl_plus_domains https://github.com/hstairs/pddl_plus_domains 23a8b790c947e69488452a308ba75d1d17a7fde1"
  "hstairs_time2processes https://github.com/hstairs/time2processes 7197802b47cd17b8d7b4c3a042e49648cda6cc2a"
)

for entry in "${repos[@]}"; do
  read -r name url commit <<< "$entry"
  dir="$dest/$name"
  if [ ! -d "$dir/.git" ]; then
    echo "cloning $name"
    git clone --quiet --filter=blob:none "$url" "$dir"
  fi
  git -C "$dir" fetch --quiet origin "$commit" 2>/dev/null || true
  git -C "$dir" checkout --quiet "$commit"
  echo "$name at ${commit:0:12}"
done

echo
echo "$(find "$dest" -name '*.pddl' | wc -l) PDDL files in $dest"
echo "Run: KAIROS_PDDL_CORPUS=$dest ctest --test-dir build -R ExternalCorpus --output-on-failure"
