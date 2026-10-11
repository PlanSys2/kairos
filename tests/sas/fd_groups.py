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

"""Mutex groups according to Fast Downward's translator.

Usage: fd_groups.py <fd>/src/translate domain.pddl problem.pddl
Prints "GROUP a|b|c" for every instantiated mutex group with two atoms or more and
"CHOSEN a|b|c" for the groups chosen as variables; atoms are sorted.
"""

import contextlib
import io
import sys


def main():
    sys.path.insert(0, sys.argv[1])
    domain, problem = sys.argv[2], sys.argv[3]
    from fast_downward.translate import options
    options.set_options([domain, problem])
    from fast_downward.translate import fact_groups, instantiate, normalize, pddl, pddl_parser
    with contextlib.redirect_stdout(io.StringIO()):
        task = pddl_parser.open(domain, problem)
        normalize.normalize(task)
        _, atoms, _, goal_list, _, params = instantiate.explore(task)
        negative = {g.negate() for g in (goal_list or []) if g.negated}
        groups, mutex_groups, _ = fact_groups.compute_groups(task, atoms, params, negative)

    def text(atom):
        return '(%s)' % ' '.join([atom.predicate] + list(atom.args))

    for kind, gs in (('GROUP', mutex_groups), ('CHOSEN', groups)):
        for g in gs:
            if len(g) >= 2:
                print(kind + ' ' + '|'.join(sorted(text(a) for a in g)))


if __name__ == '__main__':
    main()
