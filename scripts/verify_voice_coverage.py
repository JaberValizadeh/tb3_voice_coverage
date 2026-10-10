#!/usr/bin/env python3
"""
Offline check of the three things that make a request cost less driving.

`tb3_house_full_coverage/scripts/verify_subsets.py` already checks that every
room subset is fully swept when planned from the spawn pose in house order.
This checks the claims this package adds on top, and checks them the same way:
by simulating the swept band along the planned path, never by argument.

  1. ORDER CANNOT COST COVERAGE. The rooms of a request are re-ordered for the
     shortest drive, so every ordering - not just house order - has to reach
     100% of what was asked for, with no wall contacts.
  2. ORDER IS ACTUALLY CHEAPER. From a robot that is not at the spawn pose,
     `order_rooms` must plan no further than fixed house order does.
  3. SKIPPING CLEAN FLOOR CANNOT COST COVERAGE. Re-planning with the floor
     already swept fed back in must be shorter, and the union of the two
     passes must still be everything that was asked for. That is the same
     machinery the node's top-up pass uses to finish an interrupted request,
     so 4. drives a plan half way, replans, and checks the result is complete.

No ROS master needed:  python3 scripts/verify_voice_coverage.py
"""

import itertools
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), '..', 'tb3_house_full_coverage'))

from tb3_house_full_coverage import house_layout as L            # noqa: E402
from tb3_house_full_coverage import coverage_node as C           # noqa: E402

STEP = 0.02          # sampling along each leg (m)
BODY_SAMPLES = 16    # chassis rim points tested for wall overlap


def drive(pts, sim):
    """Commit the swept band along `pts` -> (distance, wall-contact samples)."""
    sim.commit(sim._cells(*pts[0]))
    dist, hits = 0.0, 0
    for a, b in zip(pts, pts[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        dist += d
        sim.commit(sim.preview(a, b))
        n = max(1, int(d / STEP))
        for s in range(n + 1):
            t = s / n
            x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            for k in range(BODY_SAMPLES):
                th = k * math.pi / BODY_SAMPLES * 2
                if L.is_occupied(x + L.ROBOT_RADIUS * math.cos(th),
                                 y + L.ROBOT_RADIUS * math.sin(th)):
                    hits += 1
                    break
    return dist, hits


def wanted(rooms):
    """Every coverable cell of `rooms`, whatever its state."""
    sim = C._CoverageSim()
    out = set()
    for n in rooms:
        out |= sim.cells_of(L.ROOMS[n]['rects'])
    return out


def run(rooms, start=None, covered=None):
    """Plan and drive one request -> (path, sim, distance, wall hits).

    `distance` includes the leg from `start` to the first waypoint, which the
    plan itself does not contain - without it a plan that simply starts
    further away looks cheaper than it is.
    """
    pts, _ = C.build_tour(list(rooms), start=start, covered=covered)
    sim = C._CoverageSim(covered)
    dist, hits = drive(pts, sim)
    if start is not None:
        dist += math.hypot(pts[0][0] - start[0], pts[0][1] - start[1])
    return pts, sim, dist, hits


def check_orders():
    """1. Every ordering of a request still covers all of it."""
    cases = [('kitchen', 'bathroom'),
             ('bedroom', 'bathroom'),
             ('living', 'store'),
             ('corridor', 'kitchen'),
             ('kitchen', 'study', 'bathroom')]
    print('1. every ordering reaches 100% of the request')
    print('   %-34s %9s %6s %6s' % ('order', 'path', 'miss', 'hits'))
    fails = 0
    for rooms in cases:
        for perm in itertools.permutations(rooms):
            need = wanted(perm)
            _pts, sim, dist, hits = run(perm)
            miss = sum(1 for k in need if not sim.covered[k])
            ok = miss == 0 and hits == 0
            fails += not ok
            print('   %-34s %8.1fm %6d %6d%s'
                  % ('+'.join(perm)[:34], dist, miss, hits,
                     '' if ok else '   <-- FAIL'))
    return fails


def check_ordering_saves():
    """2. order_rooms plans no further than fixed house order."""
    # (where the robot is, what is asked for). Each is a robot parked at one
    # room's door being asked for rooms on the other side of it.
    cases = [((2.4, 0.0), ('kitchen', 'bathroom')),
             ((2.4, 0.0), ('kitchen', 'bedroom', 'bathroom')),
             ((0.15, 0.0), ('kitchen', 'bathroom')),
             ((-3.0, 0.0), ('kitchen', 'bathroom'))]   # the spawn pose
    print('\n2. order_rooms is no further than house order')
    print('   %-30s %9s %9s %8s' % ('from / request', 'house', 'ordered', 'saved'))
    fails = 0
    for start, rooms in cases:
        house = [n for n in L.TOUR if n in rooms]
        chosen = C.order_rooms(rooms, start)
        _p, sim_h, d_house, h_h = run(house, start=start)
        _p, sim_c, d_ord, h_c = run(chosen, start=start)
        need = wanted(rooms)
        miss = sum(1 for k in need if not sim_c.covered[k])
        ok = d_ord <= d_house + 1e-6 and miss == 0 and h_c == 0 and h_h == 0
        fails += not ok
        print('   %-30s %8.1fm %8.1fm %7.1fm%s'
              % ('(%.1f) %s' % (start[0], '+'.join(rooms))[:30],
                 d_house, d_ord, d_house - d_ord, '' if ok else '   <-- FAIL'))
        print('      -> %s' % ' then '.join(chosen))
    return fails


def check_resume():
    """3. Re-planning around clean floor is shorter and still complete."""
    print('\n3. a second request skips floor already swept')
    fails = 0
    for first, second in [('kitchen', ('kitchen', 'bathroom')),
                          ('bedroom', ('bedroom', 'study'))]:
        pts, sim1, _d, _h = run([first])
        here = pts[-1]

        _p, sim_f, d_fresh, h_f = run(second, start=here)
        done = set(k for k in range(len(sim1.covered)) if sim1.covered[k])
        _p, sim_r, d_resume, h_r = run(second, start=here, covered=done)

        need = wanted(second)
        miss = sum(1 for k in need if not sim_r.covered[k])
        ok = d_resume < d_fresh and miss == 0 and h_r == 0 and h_f == 0
        fails += not ok
        print('   after %-9s ask for %-20s %6.1fm -> %6.1fm  miss %d%s'
              % (first, '+'.join(second), d_fresh, d_resume, miss,
                 '' if ok else '   <-- FAIL'))
    return fails


def check_topup():
    """4. An interrupted request is finished by re-planning the remainder."""
    print('\n4. an interrupted request is finished by a top-up pass')
    fails = 0
    for room in ('study', 'living', 'bathroom'):
        pts, _s, _d, _h = run([room])
        cut = max(2, int(len(pts) * 0.6))

        part = C._CoverageSim()
        d_part, _h = drive(pts[:cut], part)
        need = wanted([room])
        left = sum(1 for k in need if not part.covered[k])

        done = set(k for k in range(len(part.covered)) if part.covered[k])
        _p, top, d_top, h_top = run([room], start=pts[cut - 1], covered=done)
        # The top-up sim starts from `done`, so what it covers is the union.
        miss = sum(1 for k in need if not top.covered[k])
        ok = miss == 0 and h_top == 0 and left > 0
        fails += not ok
        print('   %-9s stopped at %5.1fm with %5d cells left; top-up %5.1fm '
              '-> %d left%s'
              % (room, d_part, left, d_top, miss, '' if ok else '   <-- FAIL'))
    return fails


def main():
    fails = (check_orders() + check_ordering_saves()
             + check_resume() + check_topup())
    print('\n%d failures' % fails)
    print('PASS' if not fails else 'FAIL')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
