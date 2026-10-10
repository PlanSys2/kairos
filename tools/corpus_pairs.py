#!/usr/bin/env python3
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

"""Prints "domain problem" for every problem of the corpus whose domain can be found.

Same matching as tests/semantic/semantic_corpus_test.cpp: X/domains/domain-N.pddl, then
X/domain.pddl, then the parent's domain.pddl.
"""

import os
import re
import sys

PROBLEM = re.compile(r'\(\s*define\s*\(\s*problem\b', re.IGNORECASE)


def domain_for(problem):
    folder = os.path.dirname(problem)
    base = os.path.dirname(folder) if os.path.basename(folder) == 'instances' else folder
    candidates = []
    m = re.search(r'(\d+)', os.path.basename(problem))
    if m:
        candidates.append(os.path.join(base, 'domains', 'domain-%s.pddl' % m.group(1)))
    candidates.append(os.path.join(base, 'domain.pddl'))
    candidates.append(os.path.join(os.path.dirname(base), 'domain.pddl'))
    for c in candidates:
        if os.path.exists(c):
            return c
    return None


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else os.environ.get(
        'KAIROS_PDDL_CORPUS', os.path.expanduser('~/.cache/kairos/corpus'))
    for folder, _, files in sorted(os.walk(root)):
        for f in sorted(files):
            if not f.endswith('.pddl'):
                continue
            path = os.path.join(folder, f)
            with open(path, errors='replace') as fh:
                head = fh.read(4096)
            if not PROBLEM.search(head):
                continue
            d = domain_for(path)
            if d:
                print(d, path)


if __name__ == '__main__':
    main()
