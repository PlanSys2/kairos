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

"""Run the vendored cpplint with the same rules as ament_cpplint.

Header guards use two underscores between path parts and always start with
KAIROS__: include/kairos/result.hpp -> KAIROS__RESULT_HPP_,
src/core/budget.hpp -> KAIROS__CORE__BUDGET_HPP_.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cpplint  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

EXTENSIONS = ['c', 'cc', 'cpp', 'cxx']
HEADERS = ['h', 'hh', 'hpp', 'hxx']

# Same filters as ament_cpplint
FILTERS = [
    '-build/c++11',
    '-runtime/references',
    '-whitespace/braces',
    '-whitespace/indent',
    '-whitespace/parens',
    '-whitespace/semicolon',
]


def header_guard(filename):
    rel = os.path.relpath(os.path.abspath(filename), REPO_ROOT).replace(os.sep, '/')
    if rel.startswith('include/kairos/'):
        rel = rel[len('include/kairos/'):]
    elif rel.startswith('src/'):
        rel = rel[len('src/'):]
    parts = ['kairos'] + rel.split('/')
    return '__'.join(re.sub(r'[^a-zA-Z0-9]', '_', p) for p in parts).upper() + '_'


def main(paths):
    cpplint.GetHeaderGuardCPPVariable = header_guard
    files = []
    for path in paths:
        if os.path.isdir(path):
            for root, _, names in os.walk(path):
                for name in sorted(names):
                    if name.rsplit('.', 1)[-1] in EXTENSIONS + HEADERS:
                        files.append(os.path.join(root, name))
        else:
            files.append(path)
    if not files:
        print('kairos_cpplint: no files to check')
        return 0
    argv = [
        '--extensions=' + ','.join(EXTENSIONS),
        '--headers=' + ','.join(HEADERS),
        '--filter=' + ','.join(FILTERS),
        '--linelength=100',
        '--counting=detailed',
    ] + sorted(files)
    filenames = cpplint.ParseArguments(argv)
    cpplint._cpplint_state.ResetErrorCounts()
    for filename in filenames:
        cpplint.ProcessFile(filename, cpplint._cpplint_state.verbose_level)
    errors = cpplint._cpplint_state.error_count
    print('kairos_cpplint: %d file(s) checked, %d error(s)' % (len(filenames), errors))
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
