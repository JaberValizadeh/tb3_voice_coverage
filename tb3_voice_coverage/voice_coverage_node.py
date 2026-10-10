#!/usr/bin/env python3
"""
voice_coverage_node - "clean the kitchen and the bathroom", and it does.

Listens on /voice_commands (published by tb3_voice_control's Vosk recognizer,
or by anything else - it is a plain std_msgs/String, so you can drive this
from the keyboard with rostopic pub), turns the words into a list of rooms,
and sweeps exactly those rooms using the tb3_house_full_coverage planner.

Reuses rather than reimplements:
  * house_layout      - room shapes, doors, tour order
  * build_tour(rooms) - lanes, perimeter laps and door routing for any subset
  * order_rooms()     - the cheapest order to visit them from where we are
  * steer()           - the cross-track controller the 100% result depends on

Understood commands
-------------------
  clean/cover/vacuum <room> [and <room> ...]   queue those rooms
  clean everything / the whole house / all     queue every room
  stop                                         halt now, drop the queue
  pause / resume                               hold position / carry on
  status                                       report queue and coverage

Least driving, then finish the job
----------------------------------
Three things keep the wheels turning only when they have to, and none of them
can trade away coverage:

  * the rooms are visited in the cheapest order from where the robot actually
    is, not in fixed house order (`order_rooms`);
  * the planner is told which floor is already clean, so a second request -
    or a top-up pass - plans around it (`build_tour(covered=...)`); the sweep
    picked for each rectangle still has to cover every cell that is *not*
    already accounted for, so a shorter plan is never a less complete one;
  * when the plan runs out, the live coverage grid is checked against what was
    asked for, and any floor still outstanding is re-planned and driven. The
    request is only reported finished when the grid says it is.

Leaving a room mid-sweep is done by retracing the path already driven back to
that room's door, which is safe by construction: it is a path the robot has
just travelled. Cutting straight for the door could cross a wall.

Live progress
-------------
Everything RViz needs is published from here, so the coverage picture updates
while the robot drives:
  /coverage_display  OccupancyGrid  walls, floor still to do, floor done
  /coverage_path     Path           the part of the plan not yet driven
  /coverage_markers  MarkerArray    per-room percentages and an overall banner
"""

import json
import math
import re

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, Vector3
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from tb3_house_full_coverage import house_layout as L
from tb3_house_full_coverage import coverage_node as C
from tb3_house_full_coverage.coverage_node import build_tour, order_rooms, steer

RATE_HZ = 20.0
VIZ_HZ = 2.0

# How long to let the coordinator's 1 Hz grid catch up before judging whether
# a request is actually finished. Judging immediately reads a grid from up to
# a second ago and sends the robot back out for floor it has just swept.
SETTLE = 1.5

# A request is re-planned rather than declared done while floor it asked for is
# still uncovered - but only while each pass is actually gaining ground, and
# never forever: a cell the robot physically cannot reach would otherwise loop.
MAX_TOPUPS = 3

# Values chosen against RViz's "costmap" palette, which is the only one with
# usable colours: -1 grey (wall or outside), 0 transparent, 1 blue, 99 cyan.
D_WALL, D_OTHER, D_TODO, D_DONE = -1, 0, 1, 99

# Spoken words -> room key. Longer aliases are matched first so "living room"
# does not get shortened to "room", and "bedroom"/"bathroom" stay distinct.
ROOM_ALIASES = [
    ('living room', 'living'),
    ('store room', 'store'),
    ('bathroom', 'bathroom'),
    ('bedroom', 'bedroom'),
    ('kitchen', 'kitchen'),
    ('living', 'living'),
    ('store', 'store'),
    ('study', 'study'),
    ('corridor', 'corridor'),
    ('hallway', 'corridor'),
]

START_WORDS = ('clean', 'cover', 'vacuum', 'sweep')

# Word-level vocabulary for the Vosk recognizer. A small closed word list is
# far more reliable than free dictation and still composes any phrase below.
GRAMMAR_WORDS = [
    'clean', 'cover', 'vacuum', 'sweep', 'the', 'and',
    'kitchen', 'living', 'store', 'room', 'rooms', 'bathroom', 'bedroom',
    'study', 'corridor', 'hallway',
    'everything', 'whole', 'house', 'all', 'every',
    'stop', 'cancel', 'pause', 'resume', 'status',
]


def parse_rooms(text):
    """Room keys mentioned in `text`.

    Matching is on whole words throughout. Substring matching looks fine until
    "clean the hallway" asks for the whole house, because "all" hides inside
    "h-all-way".

    The order returned is the order they were said; `order_rooms` decides the
    order they are actually driven in, which depends on where the robot is.
    """
    words = set(re.findall(r'[a-z]+', text))
    if (words & {'everything', 'all', 'every'}
            or ('whole' in words and 'house' in words)):
        return list(L.TOUR)

    found = []
    rest = text
    for alias, key in ROOM_ALIASES:
        pattern = r'\b%s\b' % re.escape(alias)
        if re.search(pattern, rest):
            if key not in found:
                found.append(key)
            rest = re.sub(pattern, ' ', rest)
    return found


def label(name):
    """"Living room" rather than "Living room  (L-shape)"."""
    return L.ROOMS[name]['label'].split('(')[0].strip()


def _room_anchor(name):
    """A point inside `name` to hang its RViz label on.

    The centre of the biggest rectangle, not of the bounding box: the study
    wraps a solid block and its bounding-box centre is inside that block.
    """
    x0, y0, x1, y1 = max(L.ROOMS[name]['rects'],
                         key=lambda r: (r[2] - r[0]) * (r[3] - r[1]))
    return (x0 + x1) / 2.0, (y0 + y1) / 2.0


class VoiceCoverage:
    def __init__(self):
        rospy.init_node('voice_coverage')
        self._model = rospy.get_param('~robot_model', 'turtlebot3')
        # Both default on; they are the "least driving" behaviours and are
        # parameters only so a demo can show the difference.
        self._reorder = rospy.get_param('~optimize_order', True)
        self._resume = rospy.get_param('~skip_covered', True)

        self._pose = None
        self._path = []
        self._marks = []
        self._i = 0
        self._paused = False
        self._planning = False      # build_tour takes about a second
        self._retreat = []          # driven before the next request
        self._queue = []            # rooms of the active plan
        self._room = None
        self._pct = {}              # room -> % from /coverage_rooms

        self._requested = []        # rooms of the whole request, top-ups aside
        self._topups = 0
        self._last_missing = None   # cells outstanding after the previous pass
        self._verify_at = None      # when to check the request is really done
        self._grid = None           # latest /coverage_grid, cell-for-cell

        # Cell geometry identical to the coordinator's - both derive it from
        # house_layout - so a grid index means the same thing in both.
        sim = C._CoverageSim()
        self._w, self._h = sim.w, sim.h
        self._room_cells = {n: sim.cells_of(L.ROOMS[n]['rects']) for n in L.TOUR}

        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._say = rospy.Publisher('/coverage_status', String,
                                    queue_size=10, latch=True)
        self._disp_pub = rospy.Publisher('/coverage_display', OccupancyGrid,
                                         queue_size=1, latch=True)
        self._path_pub = rospy.Publisher('/coverage_path', Path,
                                         queue_size=1, latch=True)
        self._mark_pub = rospy.Publisher('/coverage_markers', MarkerArray,
                                         queue_size=1, latch=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates,
                         self._on_states, queue_size=1)
        rospy.Subscriber('/voice_commands', String, self._on_voice)
        rospy.Subscriber('/coverage_rooms', String, self._on_rooms)
        rospy.Subscriber('/coverage_grid', OccupancyGrid, self._on_grid,
                         queue_size=1)
        # Structured requests, e.g. from llm_parser_node: {"rooms": [...]}.
        # The keyword path re-orders for the shortest drive, which would
        # silently drop an explicitly requested order ("the hallway THEN the
        # kitchen"), so that path asks for the order to be kept.
        rospy.Subscriber('/coverage_request', String, self._on_request)

        self._announce('Ready. Say e.g. "clean the kitchen and the bathroom".')
        rospy.Timer(rospy.Duration(1.0 / RATE_HZ), self._tick)
        rospy.Timer(rospy.Duration(1.0 / VIZ_HZ), self._publish_viz)

    # ── plumbing ────────────────────────────────────────────────────────────
    def _announce(self, msg):
        rospy.loginfo(msg)
        self._say.publish(String(data=msg))

    def _on_states(self, msg):
        try:
            i = msg.name.index(self._model)
        except ValueError:
            return
        p = msg.pose[i].position
        q = msg.pose[i].orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose = (p.x, p.y, yaw)

    def _on_rooms(self, msg):
        for part in msg.data.split():
            if ':' in part:
                k, v = part.split(':', 1)
                try:
                    self._pct[k] = float(v)
                except ValueError:
                    pass

    def _on_grid(self, msg):
        if msg.info.width != self._w or msg.info.height != self._h:
            rospy.logwarn_once(
                '/coverage_grid is %dx%d, expected %dx%d - ignoring it; '
                'progress display and the finish check need the coordinator '
                'from tb3_house_full_coverage.',
                msg.info.width, msg.info.height, self._w, self._h)
            return
        self._grid = msg.data

    def _covered_cells(self):
        if self._grid is None:
            return None
        return set(k for k, v in enumerate(self._grid) if v >= 100)

    def _missing(self, rooms):
        """Cells of `rooms` the coverage grid still shows as not swept."""
        if self._grid is None:
            return set()
        want = set()
        for n in rooms:
            want |= self._room_cells.get(n, set())
        return set(k for k in want if self._grid[k] == 0)

    # ── commands ────────────────────────────────────────────────────────────
    def _on_voice(self, msg):
        text = msg.data.lower().strip()
        if not text:
            return
        rospy.loginfo('heard: "%s"', text)

        if 'stop' in text or 'cancel' in text:
            self._stop(drop_queue=True)
            return
        if 'pause' in text:
            self._paused = True
            self._pub.publish(Twist())
            self._announce('Paused.')
            return
        if 'resume' in text:
            self._paused = False
            self._announce('Resuming.')
            return
        if 'status' in text:
            self._report_status()
            return

        rooms = parse_rooms(text)
        if not rooms:
            if any(w in text for w in START_WORDS):
                self._announce('Which room? I know: %s.'
                               % ', '.join(L.TOUR))
            else:
                rospy.logwarn('no room understood in "%s"', text)
            return
        if not any(w in text for w in START_WORDS):
            # A bare room name is still a clear enough request to act on.
            rospy.loginfo('no verb heard, treating as a clean request')
        self._start(rooms)

    def _on_request(self, msg):
        """Sweep an explicit, already-validated room list in the given order."""
        try:
            data = json.loads(msg.data)
        except ValueError:
            rospy.logwarn('bad /coverage_request payload: %r', msg.data)
            return
        rooms = [r for r in data.get('rooms', []) if r in L.ROOMS]
        if not rooms:
            rospy.logwarn('no known rooms in /coverage_request: %r', msg.data)
            return
        self._start(rooms, reorder=False)

    def _start(self, rooms, reorder=True, topup=False):
        rooms = [r for r in rooms if r in L.ROOMS]
        if not rooms:
            return

        # Planning a tour takes about a second. Stand still for it: driving on
        # under the old plan means the retreat just computed starts from a
        # pose the robot has already left.
        self._planning = True
        self._pub.publish(Twist())
        try:
            self._plan(rooms, reorder, topup)
        finally:
            self._planning = False

    def _plan(self, rooms, reorder, topup):
        # Leaving the current room safely comes first. Only recompute while a
        # sweep is actually running: after a `stop` the path is already gone,
        # and the retreat worked out back then is the one that matters.
        # Recomputing here would silently discard it and send the robot
        # straight at the nearest wall.
        if self._path and self._i > 0:
            self._retreat = self._retrace()
        if self._retreat:
            self._announce('Leaving the current room first.')

        # Plan from where the robot will be when the plan starts: the end of
        # the retreat if there is one, otherwise where it stands. Planning
        # from the spawn pose instead makes every order and skip decision on
        # behalf of a robot that is not there.
        if self._retreat:
            start = self._retreat[-1]
        elif self._pose is not None:
            start = self._pose[:2]
        else:
            start = None

        if reorder and self._reorder:
            rooms = order_rooms(rooms, start)
        covered = self._covered_cells() if self._resume else None

        plan, marks = build_tour(rooms, start=start, covered=covered)
        offset = len(self._retreat)
        self._path = self._retreat + plan
        self._marks = [(n, a + offset, b + offset) for n, a, b in marks]
        self._retreat = []
        self._i = 0
        self._queue = list(rooms)
        self._paused = False
        self._room = None
        self._verify_at = None
        if not topup:
            self._requested = list(rooms)
            self._topups = 0
            self._last_missing = None
            names = ', '.join(label(r) for r in rooms)
            if self._resume and self._grid is not None and not self._missing(rooms):
                # The plan the planner just made is nearly empty for the same
                # reason; say so rather than pretending to sweep.
                self._announce('%s already fully covered - nothing to sweep.'
                               % names)
            else:
                self._announce('Cleaning: %s.' % names)
        self._publish_path()

    def _retrace(self):
        """The driven path back to the current room's door, reversed.

        Safe by construction - it is exactly where the robot has just been.
        Only the current room is retraced, not earlier rooms in the queue.
        """
        if not self._path or self._i <= 0:
            return []
        start = 0
        for _n, a, b in self._marks:
            if a <= self._i <= b:
                start = a
                break
        i = min(self._i, len(self._path) - 1)
        return list(reversed(self._path[start:i + 1]))

    def _stop(self, drop_queue):
        # Remember how to get out; the next request drives this first.
        self._retreat = self._retrace()
        if drop_queue:
            self._queue = []
            self._requested = []
        self._path = []
        self._marks = []
        self._i = 0
        self._room = None
        self._paused = False
        self._verify_at = None
        self._pub.publish(Twist())
        self._publish_path()
        self._announce('Stopped.')

    def _report_status(self):
        if self._queue:
            self._announce('Cleaning %s; %d of %d waypoints done. Coverage: %s'
                           % (self._room or self._queue[0], self._i,
                              len(self._path), self._pct_summary()))
        else:
            self._announce('Idle. Coverage: %s' % self._pct_summary())

    def _pct_summary(self):
        if self._grid is None and not self._pct:
            return 'not reported yet'
        swept = [(n, self._room_pct(n)) for n in L.TOUR]
        return ', '.join('%s %.0f%%' % (n, v) for n, v in swept if v > 0) \
            or 'nothing swept yet'

    def _cells_pct(self, cells):
        if not cells or self._grid is None:
            return None
        return 100.0 * sum(1 for k in cells if self._grid[k] >= 100) / len(cells)

    def _request_pct(self):
        """How much of what was actually asked for is done, 0..100."""
        want = set()
        for n in self._requested:
            want |= self._room_cells.get(n, set())
        return self._cells_pct(want)

    def _room_pct(self, name):
        """Per-room percentage, from the grid so it tracks it cell for cell.

        /coverage_rooms says the same thing but only every LOG_PERIOD (5 s),
        which is too coarse to watch a room fill in.
        """
        pct = self._cells_pct(self._room_cells.get(name))
        return self._pct.get(name, 0.0) if pct is None else pct

    # ── driving ─────────────────────────────────────────────────────────────
    def _tick(self, _event):
        if self._verify_at is not None and rospy.Time.now() >= self._verify_at:
            self._verify_at = None
            self._verify()
            return

        if self._pose is None or self._paused or self._planning:
            return

        # One snapshot per tick: a new request replaces the path from another
        # thread, and an index read before the swap must not be used after it.
        path, i = self._path, self._i
        if not path:
            return

        if i >= len(path):
            self._plan_done()
            return

        room = None
        for n, a, b in self._marks:
            if a <= i <= b:
                room = n
                break
        if room != self._room:
            if self._room is not None:
                self._announce('%s done (%.0f%%).'
                               % (self._room, self._room_pct(self._room)))
            self._room = room
            # Rooms with nothing left in them are only being crossed. Saying
            # "now cleaning the store" right after "the store is already
            # covered" is worse than saying nothing.
            if room and (self._grid is None or self._missing([room])):
                self._announce('Now cleaning the %s.' % room)

        leg_start = path[i - 1] if i > 0 else None
        cmd, reached = steer(self._pose, leg_start, path[i])
        if reached:
            if self._path is path:      # still the plan this tick started on
                self._i = i + 1
            return
        self._pub.publish(cmd)

    def _plan_done(self):
        """The waypoints ran out - park, then check the job against the grid."""
        self._path = []
        self._marks = []
        self._i = 0
        self._room = None
        self._pub.publish(Twist())
        self._publish_path()
        self._verify_at = rospy.Time.now() + rospy.Duration(SETTLE)

    def _verify(self):
        """Finish, or go back out for whatever the grid says is still dirty."""
        if not self._requested:
            self._queue = []
            self._announce('Idle. Coverage: %s' % self._pct_summary())
            return

        if self._grid is None:
            # Nothing to check the request against, so do not claim it is
            # covered. /coverage_grid comes from the coordinator.
            done = list(self._requested)
            self._queue = []
            self._requested = []
            self._announce('Drove the whole plan for %s, but no coverage grid '
                           'was published - cannot confirm it is covered.'
                           % ', '.join(label(r) for r in done))
            return

        missing = self._missing(self._requested)
        gaining = self._last_missing is None or len(missing) < self._last_missing

        if missing and self._topups < MAX_TOPUPS and gaining:
            self._topups += 1
            self._last_missing = len(missing)
            rooms = [n for n in self._requested
                     if self._room_cells[n] & missing]
            area = len(missing) * L.RESOLUTION ** 2
            self._announce('%.2f m2 of %s still uncovered - going back for it '
                           '(pass %d of %d).'
                           % (area, ', '.join(label(r) for r in rooms),
                              self._topups, MAX_TOPUPS))
            self._start(rooms, topup=True)
            return

        done = list(self._requested)
        self._queue = []
        self._requested = []
        self._topups = 0
        self._last_missing = None
        if missing:
            self._announce('Finished %s, but %.2f m2 could not be reached. '
                           'Coverage: %s'
                           % (', '.join(label(r) for r in done),
                              len(missing) * L.RESOLUTION ** 2,
                              self._pct_summary()))
        else:
            self._announce('Finished %s - fully covered. Coverage: %s'
                           % (', '.join(label(r) for r in done),
                              self._pct_summary()))

    # ── live view ───────────────────────────────────────────────────────────
    def _remaining(self):
        """(waypoints not yet driven, index, length) read consistently.

        The viz timer and the driving timer are different threads, and a new
        request swaps the whole path out from under them, so everything here
        works off one snapshot rather than re-reading self._path per line.
        """
        path, i = self._path, min(self._i, len(self._path))
        return path[i:], i, len(path)

    def _publish_path(self):
        """The part of the plan not yet driven, so RViz shows it shrinking."""
        rest, _i, _n = self._remaining()
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        for x, y in rest:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._path_pub.publish(msg)

    def _publish_display(self):
        """/coverage_grid recoloured so RViz shows what is left, not just done.

        The raw grid marks pending floor 0, which the costmap palette draws
        fully transparent - so the covered area appears out of nowhere with
        nothing to measure it against. Here pending floor of the rooms that
        were actually asked for is blue and shrinks as cyan takes over.
        """
        if self._grid is None:
            return
        want = set()
        for n in self._requested or self._queue:
            want |= self._room_cells.get(n, set())

        data = []
        for k, v in enumerate(self._grid):
            if v < 0:
                data.append(D_WALL)
            elif v >= 100:
                data.append(D_DONE)
            elif k in want:
                data.append(D_TODO)
            else:
                data.append(D_OTHER)

        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = rospy.Time.now()
        g.info.resolution = L.RESOLUTION
        g.info.width = self._w
        g.info.height = self._h
        g.info.origin.position.x = L.X_MIN - L.MAP_PAD
        g.info.origin.position.y = L.Y_MIN - L.MAP_PAD
        g.info.origin.orientation.w = 1.0
        g.data = data
        self._disp_pub.publish(g)

    def _text_marker(self, mid, xy, text, rgba, height):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.ns = 'coverage'
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose = Pose()
        m.pose.position = Point(xy[0], xy[1], 0.4)
        m.pose.orientation.w = 1.0
        m.scale = Vector3(0.0, 0.0, height)
        m.color = ColorRGBA(*rgba)
        m.text = text
        return m

    def _publish_markers(self):
        arr = MarkerArray()
        active = set(self._requested) | set(self._queue)
        for i, name in enumerate(L.TOUR):
            pct = self._room_pct(name)
            if name in active:
                rgba = (0.3, 1.0, 0.4, 1.0) if pct >= 99.95 else (1.0, 0.85, 0.2, 1.0)
            else:
                rgba = (0.65, 0.65, 0.65, 0.8)
            arr.markers.append(self._text_marker(
                i, _room_anchor(name), '%s\n%.0f%%' % (label(name), pct),
                rgba, 0.22))

        rest, i, n = self._remaining()
        req = self._request_pct()
        if self._queue:
            head = 'Cleaning %s' % ', '.join(label(r) for r in self._queue)
            if self._paused:
                head = 'Paused - ' + head
        elif self._requested:
            head = 'Checking %s' % ', '.join(label(r) for r in self._requested)
        else:
            head = 'Idle'
        if req is not None:
            head += '   |   request %.0f%% covered' % req
        if n:
            head += '   |   waypoint %d/%d' % (min(i + 1, n), n)
        arr.markers.append(self._text_marker(
            90, (0.0, L.Y_MAX + 0.45), head, (1.0, 1.0, 1.0, 1.0), 0.28))

        # Where the robot is heading right now.
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.ns = 'coverage'
        m.id = 91
        m.type = Marker.SPHERE
        m.action = Marker.ADD if rest else Marker.DELETE
        m.pose = Pose()
        if rest:
            m.pose.position = Point(rest[0][0], rest[0][1], 0.05)
        m.pose.orientation.w = 1.0
        m.scale = Vector3(0.16, 0.16, 0.16)
        m.color = ColorRGBA(1.0, 0.35, 0.1, 0.9)
        arr.markers.append(m)

        self._mark_pub.publish(arr)

    def _publish_viz(self, _event=None):
        self._publish_display()
        self._publish_markers()
        self._publish_path()


def main():
    VoiceCoverage()
    rospy.spin()


if __name__ == '__main__':
    main()
