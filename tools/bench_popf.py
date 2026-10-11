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

"""Kairos against POPF on the PlanSys2 examples (B0): time from text to plan, and memory.

Usage: bench_popf.py KAIROS POPF [--data tests/data/plansys2] [--runs 5] [--timeout 60]

Pairs each problem with the domain of the same name (same package first). Prints a Markdown
table: median wall time over the runs, peak memory, plan length and cost for each planner.
"""

import argparse
import os
import re
import statistics
import subprocess
import tempfile
import time

DOMAIN = re.compile(r'\(\s*define\s*\(\s*domain\s+([^\s()]+)', re.IGNORECASE)
PROBLEM = re.compile(r'\(\s*define\s*\(\s*problem\b', re.IGNORECASE)
USES = re.compile(r'\(\s*:domain\s+([^\s()]+)', re.IGNORECASE)


def pairs(data):
    domains, problems = {}, []
    for f in sorted(os.listdir(data)):
        path = os.path.join(data, f)
        text = open(path, errors='ignore').read()
        m = DOMAIN.search(text)
        if m:
            domains.setdefault(m.group(1).lower(), []).append(path)
        elif PROBLEM.search(text):
            u = USES.search(text)
            if u:
                problems.append((path, u.group(1).lower()))
    out = []
    for p, name in problems:
        candidates = domains.get(name, [])
        package = os.path.basename(p).split('__')[0]
        same = [d for d in candidates if os.path.basename(d).startswith(package + '__')]
        for d in (same or candidates)[:1]:
            out.append((d, p))
    return out


def run(cmd, timeout, cwd):
    """(wall seconds, stdout, exit code) of one run, or None on timeout."""
    t0 = time.perf_counter()
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, cwd=cwd)
    try:
        out, _ = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.communicate()
        return None
    return time.perf_counter() - t0, out.decode(errors='ignore'), proc.returncode


def peak_mb(cmd, timeout, cwd):
    r = subprocess.run(['/usr/bin/time', '-f', '%M'] + cmd, capture_output=True, cwd=cwd,
                       timeout=timeout)
    lines = r.stderr.decode(errors='ignore').strip().splitlines()
    try:
        return int(lines[-1]) / 1024.0
    except (IndexError, ValueError):
        return 0.0


def steps(output, popf):
    if popf:
        if ';;;; Solution Found' not in output:
            return None
        body = output.split(';;;; Solution Found', 1)[1]
        return len(re.findall(r'^\s*[\d.]+:\s*\(', body, re.MULTILINE))
    acts = [ln for ln in output.splitlines() if ln.startswith('(')]
    return len(acts) if output.strip() else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('kairos')
    ap.add_argument('popf')
    ap.add_argument('--data', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'tests', 'data', 'plansys2'))
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--timeout', type=float, default=60.0)
    args = ap.parse_args()
    args.kairos = os.path.abspath(args.kairos)
    args.popf = os.path.abspath(args.popf)
    args.data = os.path.abspath(args.data)

    work = tempfile.mkdtemp(prefix='kairos_bench_')
    rows = []
    for d, p in pairs(args.data):
        check = subprocess.run([args.kairos, 'check', d, p], capture_output=True, text=True)
        if 'valid, plannable' not in check.stderr:
            continue
        planners = {
            'kairos (first)': [args.kairos, d, p, '--no-anytime'],
            'kairos (anytime)': [args.kairos, d, p, '--timeout', str(args.timeout)],
            'popf': [args.popf, d, p],
        }
        row = {'pair': os.path.basename(p)}
        for name, cmd in planners.items():
            times, out = [], ''
            for _ in range(args.runs):
                r = run(cmd, args.timeout, work)
                if r is None:
                    times = None
                    break
                times.append(r[0])
                out = r[1]
            if times is None:
                row[name] = ('timeout', '', '')
                continue
            n = steps(out, name == 'popf')
            mem = peak_mb(cmd, args.timeout, work)
            row[name] = ('%.1f ms' % (1000 * statistics.median(times)), '%.0f MB' % mem,
                         '-' if n is None else str(n))
        rows.append(row)

    names = ['kairos (first)', 'kairos (anytime)', 'popf']
    print('| problem | ' + ' | '.join('%s time / mem / steps' % n for n in names) + ' |')
    print('|---|' + '---|' * len(names))
    for row in rows:
        print('| %s | ' % row['pair'] + ' | '.join(' / '.join(row[n]) for n in names) + ' |')


if __name__ == '__main__':
    main()
