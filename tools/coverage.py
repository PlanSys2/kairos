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

"""Coverage and time of the kairos executable on benchmark suites of the corpus.

Run:      coverage.py run KAIROS SUITE --out results.csv [--timeout 30] [--jobs 6] [-- ARGS]
          coverage.py run KAIROS SUITE --out popf.csv --popf POPF   (POPF, validated by KAIROS)
Compare:  coverage.py compare base.csv new.csv

Suites (under $KAIROS_PDDL_CORPUS, default ~/.cache/kairos/corpus):
  b3         IPC 2002, 2023 and 2026 numeric, 10 instances per domain (1, 3, 5, ... 19)
  classical  IPC 2023 satisficing and IPC 1998-2014 satisficing domains, 10 per domain
  b1         IPC 2002-2014 temporal domains, 5 instances per domain
  b2         required concurrency and temporal numeric (IPC 2011/2014, time2processes), 5 each
  quick      2 instances per domain of b3 and classical, for a fast check

Extra ARGS after "--" go to kairos (for instance --search lama). Every plan kairos prints has
been validated by it. The agile score of a solved instance is 1 if it took at most 1 s, else
1 - log(t) / log(timeout), as in the IPC agile track.
"""

import argparse
import concurrent.futures
import csv
import math
import os
import re
import subprocess
import sys
import tempfile
import time

CORPUS = os.environ.get('KAIROS_PDDL_CORPUS', os.path.expanduser('~/.cache/kairos/corpus'))
POTASSCO = 'potassco_pddl-instances'
CLASSICAL_OLD = [
    'ipc-1998/domains/logistics-round-1-strips',
    'ipc-2000/domains/blocks-strips-typed',
    'ipc-2000/domains/freecell-strips-typed',
    'ipc-2002/domains/depots-strips-automatic',
    'ipc-2002/domains/driverlog-strips-automatic',
    'ipc-2002/domains/rovers-strips-automatic',
    'ipc-2004/domains/airport-nontemporal-strips',
    'ipc-2004/domains/pipesworld-no-tankage-nontemporal-strips',
    'ipc-2006/domains/tpp-propositional-strips',
    'ipc-2006/domains/trucks-propositional-strips',
    'ipc-2008/domains/elevators-sequential-satisficing-strips',
    'ipc-2008/domains/sokoban-sequential-satisficing-strips',
    'ipc-2008/domains/woodworking-sequential-satisficing-strips',
    'ipc-2011/domains/barman-sequential-satisficing',
    'ipc-2011/domains/nomystery-sequential-satisficing',
    'ipc-2011/domains/parking-sequential-satisficing',
    'ipc-2011/domains/visitall-sequential-satisficing',
    'ipc-2014/domains/childsnack-sequential-satisficing',
    'ipc-2014/domains/floortile-sequential-satisficing',
    'ipc-2014/domains/hiking-sequential-satisficing',
]


NUMERIC_OLD = [
    'ipc-2002/domains/depots-numeric-automatic',
    'ipc-2002/domains/driverlog-numeric-automatic',
    'ipc-2002/domains/rovers-numeric-automatic',
    'ipc-2002/domains/satellite-numeric-automatic',
    'ipc-2002/domains/zenotravel-numeric-automatic',
    'ipc-2002/domains/settlers-numeric-automatic',
]


TEMPORAL = [
    'ipc-2002/domains/depots-time-automatic',
    'ipc-2002/domains/driverlog-time-automatic',
    'ipc-2002/domains/rovers-time-automatic',
    'ipc-2002/domains/satellite-time-automatic',
    'ipc-2002/domains/zenotravel-time-automatic',
    'ipc-2004/domains/airport-temporal-strips',
    'ipc-2004/domains/airport-temporal-time-windows-strips',
    'ipc-2004/domains/pipesworld-no-tankage-temporal-strips',
    'ipc-2004/domains/pipesworld-no-tankage-temporal-deadlines-strips',
    'ipc-2004/domains/satellite-time-strips',
    'ipc-2004/domains/satellite-time-time-windows-strips',
    'ipc-2004/domains/umts-temporal-strips',
    'ipc-2004/domains/umts-temporal-time-windows-strips',
    'ipc-2006/domains/openstacks-time-strips',
    'ipc-2006/domains/pathways-metric-time',
    'ipc-2006/domains/rovers-metric-time',
    'ipc-2006/domains/storage-time',
    'ipc-2006/domains/tpp-metric-time',
    'ipc-2006/domains/trucks-time-strips',
    'ipc-2006/domains/trucks-time-constraints-timed-initial-literals',
    'ipc-2008/domains/crew-planning-temporal-satisficing-strips',
    'ipc-2008/domains/elevator-temporal-satisficing-strips',
    'ipc-2008/domains/model-train-temporal-satisficing-numeric-fluents',
    'ipc-2008/domains/openstacks-temporal-satisficing-strips',
    'ipc-2008/domains/parc-printer-temporal-satisficing-strips',
    'ipc-2008/domains/peg-solitaire-temporal-satisficing-strips',
    'ipc-2008/domains/sokoban-temporal-satisficing-strips',
    'ipc-2008/domains/transport-temporal-satisficing-numeric-fluents',
    'ipc-2008/domains/woodworking-temporal-satisficing-numeric-fluents',
    'ipc-2011/domains/crew-planning-temporal-satisficing',
    'ipc-2011/domains/elevator-temporal-satisficing',
    'ipc-2011/domains/floor-tile-temporal-satisficing',
    'ipc-2011/domains/openstacks-temporal-satisficing',
    'ipc-2011/domains/parc-printer-temporal-satisficing',
    'ipc-2011/domains/parking-temporal-satisficing',
    'ipc-2011/domains/peg-solitaire-temporal-satisficing',
    'ipc-2011/domains/sokoban-temporal-satisficing',
    'ipc-2011/domains/storage-temporal-satisficing',
    'ipc-2014/domains/driver-log-temporal-satisficing',
    'ipc-2014/domains/floor-tile-temporal-satisficing',
    'ipc-2014/domains/map-analyzer-temporal-satisficing',
    'ipc-2014/domains/parking-temporal-satisficing',
    'ipc-2014/domains/road-traffic-accident-management-temporal-satisficing',
    'ipc-2014/domains/satellite-temporal-satisficing',
    'ipc-2014/domains/storage-temporal-satisficing',
]
CONCURRENCY = [
    'ipc-2011/domains/match-cellar-temporal-satisficing',
    'ipc-2011/domains/turn-and-open-temporal-satisficing',
    'ipc-2011/domains/temporal-machine-shop-temporal-satisficing',
    'ipc-2014/domains/match-cellar-temporal-satisficing',
    'ipc-2014/domains/turn-and-open-temporal-satisficing',
    'ipc-2014/domains/temporal-machine-shop-temporal-satisficing',
    'ipc-2004/domains/airport-temporal-time-windows-compiled-strips',
    'ipc-2004/domains/satellite-time-time-windows-compiled-strips',
]
TIME2PROCESSES = ['MatchCellar', 'temporal_plant_watering', 'temporal_sailing']


def natural(name):
    return [int(t) if t.isdigit() else t for t in re.split(r'(\d+)', name)]


def numeric_domains():
    out = []
    for dataset in ('ipc2023-numeric_ipc2023-dataset', 'ipc2026-numeric_ipc2026-dataset'):
        base = os.path.join(CORPUS, dataset)
        if not os.path.isdir(base):
            continue
        for d in sorted(os.listdir(base)):
            folder = os.path.join(base, d)
            inst = os.path.join(folder, 'instances')
            if not os.path.isdir(inst):
                continue
            domains = [f for f in os.listdir(folder) if f.endswith('.pddl')]
            if len(domains) != 1:
                continue
            problems = sorted(os.listdir(inst), key=natural)
            out.append((dataset.split('_')[0] + '/' + d, os.path.join(folder, domains[0]),
                        [os.path.join(inst, p) for p in problems]))
    for rel in NUMERIC_OLD:
        d = potassco(rel)
        if d:
            out.append(d)
    return out


def potassco(rel):
    folder = os.path.join(CORPUS, POTASSCO, rel)
    inst = os.path.join(folder, 'instances')
    if not os.path.isdir(inst):
        return None
    pairs = []
    for p in sorted(os.listdir(inst), key=natural):
        m = re.search(r'(\d+)', p)
        per = os.path.join(folder, 'domains', 'domain-%s.pddl' % m.group(1)) if m else ''
        domain = per if os.path.exists(per) else os.path.join(folder, 'domain.pddl')
        pairs.append((domain, os.path.join(inst, p)))
    return (rel.split('/')[0] + '/' + rel.split('/')[-1], pairs, None)


def classical_domains():
    out = []
    sat = os.path.join(CORPUS, 'ipc2023-classical_ipc2023-dataset', 'sat')
    if os.path.isdir(sat):
        for d in sorted(os.listdir(sat)):
            folder = os.path.join(sat, d)
            problems = sorted((f for f in os.listdir(folder)
                               if f.endswith('.pddl') and f != 'domain.pddl'), key=natural)
            out.append(('ipc2023/' + d, os.path.join(folder, 'domain.pddl'),
                        [os.path.join(folder, p) for p in problems]))
    for rel in CLASSICAL_OLD:
        d = potassco(rel)
        if d:
            out.append(d)
    return out


def time2processes():
    out = []
    base = os.path.join(CORPUS, 'hstairs_time2processes', 'benchmarks')
    for d in TIME2PROCESSES:
        folder = os.path.join(base, d)
        inst = os.path.join(folder, 'instances')
        if not os.path.isdir(inst):
            continue
        problems = sorted(os.listdir(inst), key=natural)
        out.append(('time2processes/' + d, os.path.join(folder, 'domain.pddl'),
                    [os.path.join(inst, p) for p in problems]))
    return out


def pick(items, count):
    # 1, 3, 5... of a 20-instance domain: easy to hard
    if len(items) <= count:
        return items
    step = len(items) / count
    return [items[int(i * step)] for i in range(count)]


def suite(name):
    per = 2 if name == 'quick' else (5 if name in ('b1', 'b2') else 10)
    tasks = []
    groups = []
    if name == 'b1':
        groups = [d for d in (potassco(r) for r in TEMPORAL) if d]
    if name == 'b2':
        groups = [d for d in (potassco(r) for r in CONCURRENCY) if d] + time2processes()
    for label, a, b in groups:
        if b is None:
            tasks += [(label, d, p) for d, p in pick(a, per)]
        else:
            tasks += [(label, a, p) for p in pick(b, per)]
    if name in ('b3', 'quick'):
        for label, a, b in numeric_domains():
            if b is None:
                tasks += [(label, d, p) for d, p in pick(a, per)]
            else:
                tasks += [(label, a, p) for p in pick(b, per)]
    if name in ('classical', 'quick'):
        for label, a, b in classical_domains():
            if b is None:
                tasks += [(label, d, p) for d, p in pick(a, per)]
            else:
                tasks += [(label, a, p) for p in pick(b, per)]
    return tasks


def run_popf(popf, kairos, task, timeout, memory):
    """POPF on a task; solved only if Kairos validates its plan."""
    label, domain, problem = task
    t0 = time.perf_counter()
    try:
        # The memory of a cgroup: POPF links OpenBLAS, which reserves address space at start
        # (ulimit -v kills it before it begins)
        r = subprocess.run(['systemd-run', '--user', '--scope', '-q', '-p',
                            'MemoryMax=%dM' % memory, '-p', 'MemorySwapMax=0',
                            popf, domain, problem], capture_output=True, text=True,
                           timeout=timeout)
        out, hung = r.stdout, False
    except subprocess.TimeoutExpired:
        out, hung = '', True
    wall = time.perf_counter() - t0
    row = {'domain': label, 'problem': os.path.basename(problem), 'status': 'timeout',
           'time': '%.3f' % wall, 'steps': '', 'cost': '', 'expanded': ''}
    if hung or ';;;; Solution Found' not in out:
        if not hung:
            row['status'] = 'unsolvable' if 'unsolvable' in out.lower() else 'error'
        return row
    body = out.split(';;;; Solution Found', 1)[1]
    with tempfile.NamedTemporaryFile('w', suffix='.plan', delete=False) as f:
        f.write(body)
        plan = f.name
    v = subprocess.run([kairos, 'validate', domain, problem, plan], capture_output=True,
                       text=True)
    os.unlink(plan)
    row['status'] = 'solved' if v.returncode == 0 else 'invalid'
    row['steps'] = str(len(re.findall(r'^\s*[\d.]+:\s*\(', body, re.MULTILINE)))
    mk = re.search(r'makespan ([\d.]+)', v.stderr)
    row['cost'] = mk.group(1) if mk else ''
    return row


def run_one(kairos, task, timeout, memory, extra, anytime=False):
    label, domain, problem = task
    cmd = [kairos, domain, problem] + ([] if anytime else ['--no-anytime']) + [
        '--timeout', str(timeout), '--memory', str(memory), '-v'] + extra
    t0 = time.perf_counter()
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout + 30)
        code, err, out = r.returncode, r.stderr, r.stdout
    except subprocess.TimeoutExpired:
        code, err, out = -1, '', ''
    wall = time.perf_counter() - t0
    first = re.search(r'first plan ([\d.]+) ms', err)
    if anytime and first and float(first.group(1)) >= 0:
        wall = float(first.group(1)) / 1000.0  # agile: when the first plan came
    expanded = re.search(r'(\d+) expanded', err)
    status = {0: 'solved', 10: 'unsolvable', 11: 'timeout', 12: 'memory', 13: 'unsupported',
              2: 'invalid', 1: 'error', -1: 'hung'}.get(code, 'error')
    steps = len([ln for ln in out.splitlines() if ln.startswith('(')]) if code == 0 else ''
    costs = re.findall(r'; makespan = ([\d.e+-]+)', out) or re.findall(r'; cost = ([\d.e+-]+)', out)
    return {'domain': label, 'problem': os.path.basename(problem), 'status': status,
            'time': '%.3f' % wall, 'steps': steps, 'cost': costs[-1] if costs else '',
            'expanded': expanded.group(1) if expanded else ''}


def agile(row, timeout):
    if row['status'] != 'solved':
        return 0.0
    t = float(row['time'])
    return 1.0 if t <= 1.0 else max(0.0, 1.0 - math.log(t) / math.log(timeout))


def summary(rows, timeout):
    by = {}
    for r in rows:
        by.setdefault(r['domain'], []).append(r)
    total_s = total_n = 0
    total_a = 0.0
    print('| domain | solved | agile |')
    print('|---|---|---|')
    for d in sorted(by):
        rs = by[d]
        s = sum(1 for r in rs if r['status'] == 'solved')
        a = sum(agile(r, timeout) for r in rs)
        total_s += s
        total_n += len(rs)
        total_a += a
        print('| %s | %d/%d | %.2f |' % (d, s, len(rs), a))
    print('| **total** | **%d/%d** | **%.2f** |' % (total_s, total_n, total_a))


def cmd_run(args, extra):
    tasks = suite(args.suite)
    kairos = os.path.abspath(args.kairos)
    rows = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        if args.popf:
            popf = os.path.abspath(args.popf)
            futures = [pool.submit(run_popf, popf, kairos, t, args.timeout, args.memory)
                       for t in tasks]
        else:
            futures = [pool.submit(run_one, kairos, t, args.timeout, args.memory, extra,
                                   args.anytime) for t in tasks]
        for i, f in enumerate(concurrent.futures.as_completed(futures)):
            rows.append(f.result())
            if (i + 1) % 20 == 0:
                print('%d/%d' % (i + 1, len(tasks)), file=sys.stderr, flush=True)
    rows.sort(key=lambda r: (r['domain'], natural(r['problem'])))
    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print('# timeout %g s, extra args: %s' % (args.timeout, ' '.join(extra) or '(none)'))
    summary(rows, args.timeout)


def cmd_compare(args):
    def load(path):
        with open(path) as f:
            return {(r['domain'], r['problem']): r for r in csv.DictReader(f)}
    a, b = load(args.base), load(args.new)
    keys = sorted(set(a) & set(b), key=lambda k: (k[0], natural(k[1])))
    by = {}
    for k in keys:
        by.setdefault(k[0], []).append(k)
    t = args.timeout
    def quality(k, mine, other):
        # IPC quality: the best cost of the two over this one's (minimizing; 0 if unsolved)
        if mine[k]['status'] != 'solved' or not mine[k]['cost']:
            return 0.0
        costs = [float(r[k]['cost']) for r in (mine, other)
                 if r[k]['status'] == 'solved' and r[k]['cost']]
        c = float(mine[k]['cost'])
        return 1.0 if c <= 0 else min(costs) / c
    print('| domain | solved base | solved new | agile base | agile new | quality base | '
          'quality new | expanded (both solved) |')
    print('|---|---|---|---|---|---|---|---|')
    tot = [0, 0, 0.0, 0.0, 0.0, 0.0]
    for d in sorted(by):
        ks = by[d]
        sa = sum(1 for k in ks if a[k]['status'] == 'solved')
        sb = sum(1 for k in ks if b[k]['status'] == 'solved')
        aa = sum(agile(a[k], t) for k in ks)
        ab = sum(agile(b[k], t) for k in ks)
        both = [k for k in ks if a[k]['status'] == b[k]['status'] == 'solved' and
                a[k]['expanded'] and b[k]['expanded']]
        ea = sum(int(a[k]['expanded']) for k in both)
        eb = sum(int(b[k]['expanded']) for k in both)
        ratio = '%.2fx' % (eb / ea) if ea else '-'
        qa = sum(quality(k, a, b) for k in ks)
        qb = sum(quality(k, b, a) for k in ks)
        tot = [tot[0] + sa, tot[1] + sb, tot[2] + aa, tot[3] + ab, tot[4] + qa, tot[5] + qb]
        mark = ' (+)' if sb > sa else ' (-)' if sb < sa else ''
        print('| %s | %d | %d%s | %.2f | %.2f | %.2f | %.2f | %s |' % (
            d, sa, sb, mark, aa, ab, qa, qb, ratio))
    print('| **total** | **%d** | **%d** | **%.2f** | **%.2f** | **%.2f** | **%.2f** | |' %
          tuple(tot))


def main():
    argv = sys.argv[1:]
    extra = []
    if '--' in argv:
        extra = argv[argv.index('--') + 1:]
        argv = argv[:argv.index('--')]
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='command', required=True)
    r = sub.add_parser('run')
    r.add_argument('kairos')
    r.add_argument('suite', choices=['b1', 'b2', 'b3', 'classical', 'quick'])
    r.add_argument('--popf', help='run this POPF binary instead (plans validated by KAIROS)')
    r.add_argument('--anytime', action='store_true',
                   help='let Kairos improve until the timeout (cost: the last plan)')
    r.add_argument('--out', required=True)
    r.add_argument('--timeout', type=float, default=30.0)
    r.add_argument('--memory', type=int, default=2048)
    r.add_argument('--jobs', type=int, default=6)
    c = sub.add_parser('compare')
    c.add_argument('base')
    c.add_argument('new')
    c.add_argument('--timeout', type=float, default=30.0)
    args = ap.parse_args(argv)
    if args.command == 'run':
        cmd_run(args, extra)
    else:
        cmd_compare(args)


if __name__ == '__main__':
    main()
