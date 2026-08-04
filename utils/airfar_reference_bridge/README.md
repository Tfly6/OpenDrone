# airfar_reference_bridge

Air-FAR-specific bridge from the upstream
`aerial_autonomy_development_environment/src/local_planner` implementation to
OpenDrone.

- `localPlanner.cpp` keeps the upstream path-library selection algorithm.
- `pathFollower.cpp` keeps the upstream path history, look-ahead, slow-turn,
  stop/rotate and track-point decisions.
- `planner_reference.cpp` is the semantic boundary to OpenDrone. It emits
  world-frame position, velocity, acceleration, yaw and yaw-rate references;
  it never re-labels upstream roll/pitch commands or closed-loop yaw/vertical
  feedback as trajectory fields.
- Stop/rotate uses a stable position latch and an absolute yaw target. Path
  tracking uses path-tangent velocity, curvature acceleration and yaw-rate
  feed-forward. OpenDrone controllers remain the only position/attitude
  feedback layer.
- The first track reference is initialized from the first odometry sample.
  The upstream simulator-specific `(0, 0, 1)` initial pose is not exposed to
  the OpenDrone controller.

## Local-planner scale contract

The bundled Air-FAR path library has a 6 m collision-check horizon and a 2 m
published start-path segment at scale 1.0. Therefore:

- `pathScale` selects the longest collision horizon to try.
- `minPathScale` selects the shortest permitted collision horizon.
- `pathScaleStep` controls fallback resolution.
- `pathScaleBySpeed` shortens the initial horizon at low commanded speed.

Scale is a local collision-horizon setting, not a global-map-size or LiDAR
range setting. The planner tries scales from longest to shortest and always
tries the exact configured minimum, even when speed scaling produces a value
that is not aligned with `pathScaleStep`.

The Mid360 SITL wiring keeps the 2.0 long-horizon option but permits fallback
to 0.5 in 0.25 increments. This preserves longer look-ahead in open space
while allowing the path library to operate in the dense cylinder world.

## Speed contract

`minSpeed`, `cruiseSpeed`, and `maxSpeed` have separate meanings. Autonomous
tracking starts at `cruiseSpeed`; `minSpeed` is the lower bound used for
slowdown, while `maxSpeed` is only a safety cap. OpenDrone does not expose the
upstream joystick or normalized `/speed` control interfaces.

## Arrival contract

Air-FAR's upstream outdoor profile used one horizontal metre both as collision
look-ahead padding and as a stop radius. OpenDrone separates those meanings:

- `collisionCheckPadding` only extends the obstacle-check range near a goal.
- `goalTolerance` is the three-dimensional arrival radius shared by the
  Air-FAR global planner, the path follower, and mission evaluation.

The local planner continues publishing a collision-free path until the
vehicle reaches the 3-D arrival region. The path follower then holds the exact
goal position with zero feed-forward velocity so the controller can satisfy
the configured dwell requirement.

The package name and runtime node names are Air-FAR-specific intentionally; it
is not a general-purpose OpenDrone local planner.
