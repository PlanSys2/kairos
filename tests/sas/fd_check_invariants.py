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

"""Checks invariants with Fast Downward's balance checker.

Usage: fd_check_invariants.py <fd>/src/translate domain.pddl problem.pddl < invariants
Each input line is an invariant: parts separated by ";", a part is "predicate arg arg ..." with
"*" at the counted position. Prints "OK <line>" or "REJECTED <line>".
"""

import contextlib
import io
import sys


def main():
    sys.path.insert(0, sys.argv[1])
    domain, problem = sys.argv[2], sys.argv[3]
    from fast_downward.translate import options
    options.set_options([domain, problem])
    from fast_downward.translate import instantiate, invariant_finder, invariants, normalize
    from fast_downward.translate import pddl_parser
    with contextlib.redirect_stdout(io.StringIO()):
        task = pddl_parser.open(domain, problem)
        normalize.normalize(task)
        _, _, _, _, _, params = instantiate.explore(task)
    checker = invariant_finder.BalanceChecker(task, params)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        parts = []
        for text in line.split(';'):
            words = text.split()
            args = [invariants.COUNTED if w == '*' else int(w) for w in words[1:]]
            omitted = args.index(invariants.COUNTED) if invariants.COUNTED in args else None
            parts.append(invariants.InvariantPart(words[0], args, omitted))
        inv = invariants.Invariant(parts)
        ok = inv.check_balance(checker, lambda i: None)
        print(('OK ' if ok else 'REJECTED ') + line)


if __name__ == '__main__':
    main()
