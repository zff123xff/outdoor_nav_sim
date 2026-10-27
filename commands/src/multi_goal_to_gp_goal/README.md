# multi_goal_to_gp_goal

A lightweight bridge node that forwards GPS waypoints from
`msg_set_msgs/MultiGoal` (published by `execute_task_server` on
`/waypoint_goals`) to `sensor_msgs/NavSatFix` on `/gp_goal` so the existing
`nav2_input_bridge` → `straight_planner` → `controller_server` pipeline can
accept them one at a time. Multi-waypoint missions are dispatched
sequentially using odom-based arrival detection.

## Data flow

```
execute_task_server (/waypoint_goals, MultiGoal [N waypoints])
        │
        ▼
multi_goal_to_gp_goal  ← this package
        │  ┌───────────────────────────┐
        │  │ stateful queue + odom watch │
        │  └───────────────────────────┘
        │
        ├──> /gp_goal (sensor_msgs/NavSatFix, one waypoint at a time)
        │             │
        │             ▼
        │     nav2_input_bridge ── llaToEnu using reference_lat/lon/alt ──> /goal_pose
        │                                                                      │
        └──<── /mavros/local_position/odom  (arrival feedback)                 ▼
                                                                  straight_planner → controller_server
```

## Behavior

| Input MultiGoal | Action |
|---|---|
| `is_gps_aid=true`, N waypoints | Append all N to the queue. If idle, dispatch the first immediately. |
| `is_gps_aid=false` | Warning, skipped — local xy can't be expressed as lat/lon |
| empty waypoint list | Warning, skipped |

### Arrival detection

Every `check_period_s` seconds the node compares the latest odom position
to the current goal's projected ENU. When `consecutive_required` consecutive
samples are within `reach_threshold_m` of the goal, the current waypoint is
considered reached and the next one is dispatched.

New `MultiGoal` messages arriving while the queue is non-empty are
**appended to the tail** — the currently-executing goal is never disturbed.

NavSat status fields are set to `STATUS_FIX` / `SERVICE_GPS` so
`nav2_input_bridge` (`require_nav_sat_fix=true`) accepts the message.

## Build

```bash
cd ~/ww/outdoor_nav
colcon build --packages-select multi_goal_to_gp_goal
source install/setup.bash
```

## Run

```bash
ros2 launch multi_goal_to_gp_goal multi_goal_to_gp_goal.launch.py
```

`reference_latitude/longitude/altitude` are **auto-loaded from
`src/robot_launch/config/go2w_nav2.yaml`** (`nav2_input_bridge.ros__parameters`)
at launch time, so the bridge's ENU origin always matches the nav2 stack.
Edit that one YAML when you move to a new test site — no per-service
parameter bookkeeping needed.

For the wheeled-robot variant:

```bash
ros2 launch multi_goal_to_gp_goal multi_goal_to_gp_goal.launch.py \
    nav2_params_yaml:=/home/nhy/ww/outdoor_nav/src/robot_launch/config/gp_goal_nx_nav2.yaml
```

To override individual values per-session (e.g. for a one-off test),
pass CLI args; they take precedence over the YAML defaults:

```bash
ros2 launch multi_goal_to_gp_goal multi_goal_to_gp_goal.launch.py \
    reference_latitude:=31.5 reference_altitude:=15.0
```

If neither the YAML nor CLI provides a value, the bridge silently defers
goal publication and logs a throttled warning every 5 s — the only way
this happens is if `go2w_nav2.yaml` is missing or its reference fields are
all commented out.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `nav2_params_yaml` | `.../robot_launch/config/go2w_nav2.yaml` | YAML to auto-load reference_* from |
| `input_topic` | `/waypoint_goals` | Source `MultiGoal` topic |
| `output_topic` | `/gp_goal` | Destination `NavSatFix` topic |
| `frame_id` | `gps` | `header.frame_id` on the outgoing `NavSatFix` |
| `navsat_status` | `1` (STATUS_FIX) | `status.status` field. `0` = `STATUS_NO_FIX` and will be dropped by `nav2_input_bridge` |
| `navsat_service` | `1` (SERVICE_GPS) | `status.service` field |
| `reference_latitude` | from `nav2_params_yaml` | Origin lat (deg) for GPS→ENU arrival distance |
| `reference_longitude` | from `nav2_params_yaml` | Origin lon (deg) |
| `reference_altitude` | from `nav2_params_yaml` | Origin altitude (m) |
| `reach_threshold_m` | `0.5` | Within this radius (m, 2D) the goal counts as reached |
| `consecutive_required` | `5` | Number of consecutive samples within threshold to declare arrival |
| `check_period_s` | `0.1` | Odom comparison period (s); 0.1 = 10 Hz |

## Limitations

- **No timeout / failure handling**: if a waypoint is unreachable the node
  will wait forever for arrival. Add `goal_timeout_s` (and a per-waypoint
  failure callback) before using in production.
- **Append-on-new-task semantics**: a flood of `MultiGoal` messages will
  grow the queue unbounded; there is currently no cap.
- **Local-frame input is dropped**: only `is_gps_aid=true` requests are
  forwarded; callers should send GPS coordinates.
- **2D arrival only**: the threshold is computed from `hypot(east, north)`;
  vertical proximity is ignored.

## License

Apache-2.0