# tb3_voice_coverage

Ask a **TurtleBot3** to clean the rooms you name — *"clean the kitchen and the
bathroom"* — and it sweeps exactly those, driving as little as it can get away
with, and does not call the job done until the coverage grid agrees. RViz
shows it happening.

This package is the glue between two existing ones and adds no geometry or
control of its own:

| From | Reused for |
|---|---|
| [`tb3_voice_control`](../tb3_voice_control) | offline Vosk speech → `/voice_commands` |
| [`tb3_house_full_coverage`](../tb3_house_full_coverage) | house world + map, room shapes, sweep planner, 100% coverage accounting |

Two understanding modes:

| Mode | How you must speak | Needs |
|---|---|---|
| **keyword** (default) | roughly `"clean the kitchen"` — a fixed 26-word vocabulary | nothing, works offline |
| **LLM** (`llm:=true`) | anything, in any language — *"tidy up where I sleep"* | an API key, one call per utterance |

## Commands

| Say | Effect |
|---|---|
| `clean the kitchen` | sweep one room (`cover` / `vacuum` / `sweep` also work) |
| `clean the kitchen and the bathroom` | sweep several, in whichever order drives least |
| `clean everything` / `clean the whole house` / `clean all rooms` | sweep all seven |
| `stop` / `cancel` | halt now and drop the queue |
| `pause` / `resume` | hold position / carry on |
| `status` | report the queue and coverage so far |

Rooms: `kitchen`, `living room`, `store room`, `bathroom`, `bedroom`,
`study`, `corridor` (also `hallway`).

## Run

Three stages, in three terminals.

```bash
# Terminal 1 - house world + robot
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch tb3_voice_coverage environment.launch          # or gui:=false

# Terminal 2 - map + AMCL
source ~/catkin_ws/devel/setup.bash
source ~/nav_underlay/setup_nav.bash
roslaunch tb3_voice_coverage localization.launch

# Terminal 3 - listen, clean, and show it happening
source ~/catkin_ws/devel/setup.bash
roslaunch tb3_voice_coverage voice_coverage.launch       # rviz:=false to skip
```

Then speak a command. Progress is logged per room, the node's replies go to
`/coverage_status`, and RViz fills in behind the robot.

**RViz starts in stage 3**, with this package's own config, not in stage 2 —
it belongs with the thing it is showing, and nothing it draws comes from the
navigation stack. Without terminal 2 there is no `/map`, `/scan` or AMCL, but
the house, the coverage and the room percentages still come up:

```bash
roslaunch tb3_voice_coverage voice_coverage.launch static_map_tf:=true
```

`static_map_tf` publishes the `map -> odom` transform AMCL would have
published (RViz draws nothing at all without it). Leave it off whenever
terminal 2 is running — two publishers of one transform fight each other.

## Live progress in RViz

Four layers, all published by `voice_coverage_node`:

| Layer | Topic | Shows |
|---|---|---|
| Coverage progress | `/coverage_display` | grey walls, **blue** floor you asked for that is still to do, **cyan** floor swept |
| Remaining plan | `/coverage_path` | the part of the route not yet driven, shrinking as it goes |
| Coverage labels | `/coverage_markers` | each room's percentage, and a banner with the request total and waypoint count |
| Scan / map | `/scan`, `/map` | only these two need stage 2 |

The blue layer is the point. `/coverage_grid` marks floor still to do as `0`,
which RViz's costmap palette draws **fully transparent** — so the swept area
appears out of nowhere with nothing to measure it against, and a request that
is 40% done looks the same as one that is nearly finished.
`/coverage_display` is the same grid recoloured against that palette so the
blue you asked for visibly turns cyan, and it covers the whole house, so a
room nobody asked for stays transparent rather than looking incomplete.

Percentages come from the grid rather than from `/coverage_rooms`, which the
coordinator publishes only every 5 s — too coarse to watch a room fill in.

### Without a microphone

`/voice_commands` is a plain `std_msgs/String`, so the whole package works by
typing — useful for testing, headless machines, or when the mic is flaky:

```bash
roslaunch tb3_voice_coverage voice_coverage.launch voice:=false

rostopic pub -1 /voice_commands std_msgs/String "data: 'clean the kitchen'"
rostopic pub -1 /voice_commands std_msgs/String "data: 'stop'"
```

## LLM understanding (`llm:=true`)

The keyword parser only matches a fixed vocabulary — you must say roughly
*"clean the kitchen"*. With `llm:=true` the recognizer switches to free-form
dictation and an LLM maps whatever you actually said onto room keys:

```bash
roslaunch tb3_voice_coverage voice_coverage.launch llm:=true
```

| You say | It does |
|---|---|
| "tidy up where I sleep" | bedroom |
| "give the loo a going over" | bathroom |
| "the room with the desk please" | study |
| "do everywhere except the study" | the other six rooms |
| "do the hallway then the kitchen" | corridor **then** kitchen — your order is kept |
| "آشپزخانه را تمیز کن" | kitchen, and it answers in Persian |
| "clean up please" | asks *which room?* instead of guessing |

Credentials come from the same `OAI_CONFIG_LIST.json` that `tb3_llm_control`
already uses, so keys live in one place:

```bash
roslaunch tb3_voice_coverage voice_coverage.launch llm:=true llm_model:=gpt-4o
roslaunch tb3_voice_coverage voice_coverage.launch llm:=true \
    llm_config:=~/my_configs.json llm_model:=gpt-oss:20b   # local Ollama
```

### The LLM cannot steer the robot on its own

Two properties keep a language model from being a liability:

- **Its room names are never trusted.** Everything it returns is filtered
  against `house_layout.TOUR`, so a hallucinated `"garage"` is dropped before
  it reaches the planner. If nothing valid survives, the request becomes a
  question rather than a drive.
- **Nothing depends on it.** Missing config, no network, an API error or
  unparseable JSON all fall back to the keyword parser, which is offline and
  free. The worst case is the old fixed vocabulary, never a robot that ignores
  you.

It also only ever chooses *which rooms*. The route, the lane geometry and the
100% coverage guarantee come from `tb3_house_full_coverage` and are unaffected
by what the model says.

Costs one API call per utterance; the default keyword mode costs nothing.

## How a request becomes a sweep

1. **Words → rooms.** `parse_rooms()` matches room names on **whole words**.
   Substring matching is tempting and wrong: *"clean the h-**all**-way"* would
   otherwise trigger the "all rooms" phrase and clean the entire house.
2. **Rooms → order.** `order_rooms()` decides which to sweep first — see
   below.
3. **Rooms → path.** `build_tour(rooms)` from `tb3_house_full_coverage` plans
   the perimeter lap plus lanes for each requested room. Any subset works
   because every room's access route starts and ends on the corridor
   centreline, so consecutive rooms are joined by a straight run along the
   corridor.
4. **Path → motion.** The same `steer()` used by the full-house sweep, which
   corrects cross-track error rather than merely aiming at the next waypoint.
   That is what the 100%-per-room result depends on.
5. **Grid → done, or round again.** When the waypoints run out the coverage
   grid decides whether the request is actually finished — see *Until it is
   actually covered* below.

### Interrupting safely

Saying `stop` mid-room, or naming a different room while one is in progress,
cannot just turn for the door — a straight line to it may cross a wall. The
node instead **retraces the path it has already driven** back to the current
room's door, which is safe by construction, and only then heads for the next
room. Only the current room is retraced, not the whole queue.

## Least driving for the coverage you asked for

Three things keep the wheels turning only when they have to. None of them can
trade away coverage, and each is checked rather than argued:

**The cheapest order, from where the robot actually is.** Every door sits on
the corridor centreline, so the travel part of a multi-room request is a
one-dimensional problem: visit a set of points on a line, starting wherever
the robot is standing. `order_rooms()` solves it exactly — run to the nearer
end of the span, then straight across — so no room is passed twice. Fixed
house order is the special case of this for a robot parked at the west end,
which is where it spawns, so a first request behaves exactly as before. It
stops being cheapest the moment the robot is somewhere else:

| Robot at | Request | House order | Ordered |
|---|---|---|---|
| bathroom door | kitchen + bathroom | 50.6 m | **46.0 m** |
| bathroom door | kitchen + bedroom + bathroom | 73.0 m | **68.4 m** |
| spawn | kitchen + bathroom | 46.8 m | 46.8 m (unchanged) |

**Floor that is already clean is planned around.** The planner is handed the
live coverage grid, so a later request costs only what is genuinely left:

| Already done | Then asked for | From scratch | Knowing what is clean |
|---|---|---|---|
| kitchen | kitchen + bathroom | 46.0 m | **28.4 m** |
| bedroom | bedroom + study | 54.7 m | **33.8 m** |

This cannot lose coverage, because the sweep chosen for each rectangle still
has to cover every cell of it that the grid does *not* already account for. A
shorter plan is never a less complete one. Ask for a room that is entirely
clean and the robot says so instead of moving. To sweep it again anyway:

```bash
roslaunch tb3_voice_coverage voice_coverage.launch skip_covered:=false
roslaunch tb3_voice_coverage voice_coverage.launch optimize_order:=false   # fixed house order
```

### Until it is actually covered

Running out of waypoints is not the same as being finished. When the plan
ends, the node waits for the coverage grid to catch up and compares it against
what was asked for. Any floor still outstanding is re-planned — the same
"skip what is clean" machinery, which reduces the new plan to just the legs
that reach the gaps — and driven. Only when the grid agrees does it report
*"fully covered"*; if a pass stops gaining ground, or after three of them, it
says how much could not be reached rather than claiming success.

That is what makes `stop` half way through a room recoverable: ask again and
the robot retraces to the door, plans around the part it had already swept,
and finishes the rest.

## Coverage reporting

The coordinator from `tb3_house_full_coverage` runs unchanged and accounts for
the whole house, so rooms you did not ask for simply stay at 0%. Requested
rooms reach **100%** on the same definition used there (a 0.40 m cleaning head,
wall interiors excluded from the denominator — see that package's README).

That is checked for *every* request, not just the full house:

```bash
python3 ../tb3_house_full_coverage/scripts/verify_subsets.py
```

Each room alone, every pair, and the whole house — 29 combinations, all at
100% of the requested cells with zero wall contacts. Subsets need their own
check because a subset transits different floor on the way to each room, so
the planner arrives with a different set of cells already swept and makes
different choices; the full-house tour passing does not imply they do.

The three shortcuts above get the same treatment — the numbers in *Least
driving* are this script's output:

```bash
python3 scripts/verify_voice_coverage.py
```

It simulates the swept band along each planned path and checks that **every
ordering** of a request still reaches 100% (not just house order), that
`order_rooms` never plans further than house order, that re-planning around
clean floor is shorter *and* still complete, and that a plan stopped 60% of
the way through is finished off completely by the top-up pass.

| Topic | Type | Meaning |
|---|---|---|
| `/voice_commands` | `std_msgs/String` | recognized speech, keyword mode (input) |
| `/voice_text` | `std_msgs/String` | free-form speech, LLM mode (input) |
| `/coverage_request` | `std_msgs/String` | `{"rooms": [...]}` — explicit, ordered |
| `/coverage_status` | `std_msgs/String` | what the robot is doing / has finished |
| `/coverage_rooms` | `std_msgs/String` | `kitchen:100.0 bedroom:0.0 ...` |
| `/coverage_percent` | `std_msgs/Float32` | whole-house percentage |
| `/coverage_grid` | `nav_msgs/OccupancyGrid` | 100 covered, 0 pending, −1 not coverable |
| `/coverage_display` | `nav_msgs/OccupancyGrid` | the same, recoloured for RViz (output) |
| `/coverage_path` | `nav_msgs/Path` | the plan not yet driven (output) |
| `/coverage_markers` | `visualization_msgs/MarkerArray` | room percentages and banner (output) |

## Notes on the two reused packages

Both needed a small, backward-compatible addition:

- `tb3_voice_control/voice_recognizer.py` gained a `~grammar` parameter, so
  this package can supply a room vocabulary instead of the teleop phrases.
  Unset, it behaves exactly as before.
- `tb3_house_full_coverage` gained `build_tour(rooms)` for subsets, an
  exported `steer()`, and `/coverage_rooms`; then `build_tour(start=,
  covered=)` and `order_rooms()` for the driving savings above. Defaults are
  unchanged — asking for every room from the spawn pose plans exactly the
  whole-house tour it always did (122 waypoints, 144.4 m), and
  `verify_subsets.py` still reports the same 29 lines.

Subsets benefit from that package's efficiency work too, since the sweep
chosen for each rectangle is picked against what is already covered: the
study alone dropped from 44.1 m to 30.0 m, kitchen + bathroom from 50.5 m to
46.0 m, both still at 100%.

Do not run this together with `tb3_house_full_coverage`'s own
`coverage.launch`: both publish `cmd_vel`.
