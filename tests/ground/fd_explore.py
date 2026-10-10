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

"""Reachable atoms and actions according to Fast Downward's translator.

Usage: fd_explore.py <fd>/src/translate domain.pddl problem.pddl
Prints "ATOM (p a b)" and "ACTION (name a b)" lines (what grounding is compared against);
derived atoms are left out.
"""

import contextlib
import io
import sys


def main():
    sys.path.insert(0, sys.argv[1])
    domain, problem = sys.argv[2], sys.argv[3]
    from fast_downward.translate import options
    options.set_options([domain, problem])
    from fast_downward.translate import instantiate, normalize, pddl_parser
    with contextlib.redirect_stdout(io.StringIO()):
        task = pddl_parser.open(domain, problem)
        normalize.normalize(task)
        _, atoms, actions, _, _, _ = instantiate.explore(task)
    # Derived atoms are left out: each planner names its auxiliary predicates its own way
    derived = {ax.name for ax in task.axioms}
    for a in atoms:
        if a.predicate not in derived:
            print('ATOM (%s)' % ' '.join([a.predicate] + list(a.args)))
    for a in actions:
        print('ACTION ' + a.name.replace(' )', ')'))


if __name__ == '__main__':
    main()
