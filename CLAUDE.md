# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

## Project overview

Isaac Sim-backed drop-in emulator for `ak_motor_driver`'s `ak_motor_cable_control_node`
(sister package `AK40-10-ROS2-Bridge`, `../AK40-10-ROS2-Bridge`). Presents the **exact same**
ROS2 interface (topic/service names, types, state machine) as the real driver, but drives the
variable-length slung-load cable in Isaac Sim (`ROS2CableWinchBackend` in
`fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/.../cable_winch_backend_utils.py`)
instead of real CAN hardware. The ground station GUI (`fsc_ak_actuator_ground_control`) and
`cable_torque_ctrl_node` (external BLSMC/L1/UDE torque controller) run against this node
**completely unmodified**.

```
GUI / cable_torque_ctrl_node  <--ROS2-->  ak_motor_cable_control_node (this package)  <--ROS2-->  Isaac Sim (ROS2CableWinchBackend)
```

Design requirement source: `fsc_PegasusSimulator/docs/design_requirements/design_requirements.txt`
(req. #4-6) and that repo's own `CLAUDE.md` ("Part 2" of the variable-length-cable work).

## Build

```bash
cd ~/Workspaces/fsc_autopilot_ws
colcon build --packages-select ak_motor_cable_control_emulator
source install/setup.bash
```

C++17, `ament_cmake`. No tests beyond ament lint stubs.

**Environment gotcha on this machine**: `colcon build` on a freshly-configured package fails
with `ModuleNotFoundError: No module named 'catkin_pkg'` if a conda env (e.g. `fsc_isaac_env`,
used for Isaac Sim) is active — it shadows the ROS system Python that has `catkin_pkg`
installed. Fix: `PATH=/usr/bin:$PATH colcon build ...`. If a prior failed attempt already
cached the wrong Python path in `build/ak_motor_cable_control_emulator/CMakeCache.txt`, remove
`build/ak_motor_cable_control_emulator` and `install/ak_motor_cable_control_emulator` first —
CMake won't re-resolve the interpreter on an incremental build.

## Launch

```bash
# Isaac Sim scenario must already be running with the drone actually hovering (piloted) —
# a static/grounded drone has no taut vertical cable and the emulator's SPEED-mode control
# law has nothing meaningful to track. See fsc_PegasusSimulator/CLAUDE.md's WIP section for
# the full story on why "static drone" test setups gave a false-negative bug report earlier.
# scripts/start_single_drone_sitl_payload_variable_cable.sh fsc_lab_machine

ros2 launch ak_motor_cable_control_emulator cable_control_emulator.launch.py
# optionally: isaac_winch_prefix:=cable_winch_1/

# Then, unmodified:
ros2 run ak_motor_ground_station ak_motor_ground_station
ros2 launch ak_motor_driver cable_torque_ctrl.launch.py
```

## Scope — what's actually simulated

Only **SPEED mode** and **EXTERNAL mode** are actuated (drive real forces into the sim), per
explicit project scope decision. **POSITION** and **TORQUE** `mode_cmd` switches are accepted
and reflected in `~/control_mode` so the GUI never errors or gets stuck, but commands received
while in those modes are **not** applied to the cable — the poll loop falls back to the same
`kd_watchdog`-damping-to-zero-velocity behavior used for a stale/missing command. This is a
deliberate simplification, not a bug: nothing downstream currently needs real position or
torque-direct tracking against the sim cable.

## Architecture

Single `rclcpp::Node`, `ak_motor_cable_control_node.cpp`, structured to intentionally mirror
the real driver's class layout (same member/method names where the concept carries over) so
the two stay easy to diff as the real driver evolves.

- A wall timer at `poll_rate_hz` (default 100 Hz, matching the real driver) — `poll_tick()` —
  runs `check_heartbeat()`, computes and publishes the cable force command
  (`compute_and_publish_force()`), and publishes `~/control_mode`, `~/ext_mode_state`,
  `~/enabled`, `~/node_heartbeat` unconditionally every cycle (matches the real driver's
  `poll_can()` publishing those regardless of whether new hardware feedback arrived).
- `~/joint_state` and `~/cable_state` are published from `on_state_cable()`, event-driven off
  the Isaac Sim `{isaac_winch_prefix}state/cable` subscription — mirrors the real driver
  publishing those once per decoded CAN feedback frame, not off the timer.
- `~/mode` (Int8), `~/error_flags` (UInt8), `~/temperature` (Float32) are constant-zero /
  constant-25°C stubs, published every poll cycle for interface parity — there's no physical
  motor to report real values for, and nothing downstream reads them meaningfully today (the
  GUI's corresponding labels are documented as "Reserved" in `fsc_ak_actuator_ground_control`'s
  own `CLAUDE.md`), but the topics exist in case something later depends on them.

## Key design decisions

### `effective_radius` — the torque/force and rad-s/m-s bridge

The sim applies a **direct linear force** to the cable (no drum, no rotating shaft). The real
hardware's `~/ext_torque_cmd` (N·m), `~/command.velocity` (rad/s), `kd_speed`
(N·m·s/rad), and `torque_limit_upper/lower` (N·m) are all in motor-shaft units, scaled by the
real drum's `drum_radius`. `effective_radius` (default `0.036` m — the real hardware's
**deployed** value from `AK40-10-ROS2-Bridge/config/cable_control_params.yaml`, not the
`0.0175` fallback hardcoded in that package's `.cpp`) is the single conversion factor used
throughout, so all of the above parameters keep their real, calibrated meaning when reused here
unchanged. It is a *modeling choice* ("as if this sim cable were driven by the same drum as the
real hardware"), not a literal physical drum in the sim.

### Sign conventions — traced explicitly against both codebases, not assumed symmetric

| Quantity | Real driver convention | Sim (`ROS2CableWinchBackend`) convention | Conversion applied |
|---|---|---|---|
| `~/cable_state` length/velocity | length **decreases** on retract; velocity **negative** on retract | `extension` increases on extend (decreases on retract); `extension_vel` same sign | **None** — direct passthrough, `cable_length = extension - extension_offset_`, `cable_velocity = extension_vel` |
| `~/ext_torque_cmd` (N·m) → sim force (N) | positive = retract (default `motor_direction=1` in `cable_torque_ctrl_params.yaml`) | positive `command/force` = retract | **None** in sign, just scaled: `force_N = ext_torque_cmd_Nm / effective_radius` |
| `~/command.velocity[0]` (rad/s, SPEED mode) | **positive = retract**, raw motor-shaft convention | sim's `extension_vel` is **positive = extend** (opposite) | **Sign flip required** — both desired and actual velocity are converted into one internal "retract-positive" frame before the `kd_speed*(v_des−v)` law runs: `v_des_retract = command.velocity[0] * effective_radius`, `v_actual_retract = -extension_vel` |
| `~/joint_state` (synthesized, rad/rad-s) | positive = retract | — | `position = -(extension-extension_offset_) / effective_radius`, `velocity = -extension_vel / effective_radius` |

**This is the easiest place to introduce a silent bug**: `~/cable_state` and external-mode
torque are sign-aligned between the two systems already; `~/command.velocity` is not. A
positive commanded SPEED-mode velocity must **retract** the cable — if it extends instead
instead, the flip above (`v_actual_retract = -extension_vel`) or the `command.velocity[0] *
effective_radius` term got dropped or double-applied somewhere. Verify this first when
debugging any SPEED-mode behavior that looks backwards.

### Cable-length zero reference

`extension_offset_` is a plain member double, set on `~/zero_position` (rejects while
`enabled_`, same guard as the real driver) and on `~/enable_external_mode` entry. Unlike the
real driver's `reset_cable_length_tracking()`, there is **no rollover tracking** — the sim
gives a clean unwrapped float extension value directly, so no 16-bit-encoder-wraparound logic
is needed (`rollover_count_`, `theta_prev_`, `p_range_` all have no equivalent here).

### `~/enable` / `~/disable`

No CAN calls (obviously) — just toggles `enabled_`. When `!enabled_`, `compute_and_publish_force()`
commands `0.0` N (freewheel), standing in for the real driver's `exit_mit_mode` CAN frame.

### External mode state machine and authority hierarchy

Ported exactly from the real driver, including the authority split (ground station owns the
outer gate via `~/enable_external_mode` service + `~/ext_mode_cmd`; an external controller node
owns only the inner gate via `~/ext_torque_enable`) and the `RUNNING → STANDBY` /
heartbeat-lost fallback to zero-velocity SPEED mode. See the real driver's own `CLAUDE.md`
(`AK40-10-ROS2-Bridge/CLAUDE.md`, § External mode) for the full state diagram and activation
sequence — it applies here unchanged.

### Watchdogs

`command_timeout_ms` (default 500 ms) and dual `heartbeat_timeout_ms` (default 1500 ms,
matching the real driver's **deployed** yaml value, not its `.cpp` fallback of 1000 ms; either
`~/heartbeat` or `~/heartbeat_external` fresh keeps the motor enabled) — both ported verbatim,
same fallback behavior (`kd_watchdog` damping / motor auto-disable + external mode cleared).

## Parameters (`config/cable_control_emulator_params.yaml`)

| Parameter | Default | Notes |
|---|---|---|
| `isaac_winch_prefix` | `"cable_winch_0/"` | Topic prefix `ROS2CableWinchBackend` publishes/subscribes under in Isaac Sim |
| `poll_rate_hz` | 100.0 | Matches real driver |
| `effective_radius` | 0.036 (m) | Torque/force + rad-s/m-s scale factor — see above |
| `kd_speed` | 0.5 (N·m·s/rad) | Re-read from param server on `mode_cmd="speed"`, same as real driver |
| `command_timeout_ms` | 500.0 | |
| `kd_watchdog` | 0.05 (N·m·s/rad) | |
| `heartbeat_timeout_ms` | 1500.0 | |
| `torque_limit_upper` / `torque_limit_lower` | ±1.5 (N·m) | Clamped on `~/ext_torque_cmd` receipt, throttled `[WARN]` on clamp |

**Intentionally not declared** (no sim equivalent, dropped from the real driver's param set):
`can_interface`, `motor_id`, `p_min/p_max/v_min/v_max/t_min/t_max/kp_max/kd_max` (MIT protocol
limits — the sim's own travel limit lives in `max_cable_extension` on the Isaac Sim side),
`temp_limit_c` (no physical motor to overheat), `kp_pos`/`kd_pos` (POSITION mode isn't actuated
this pass — see Scope above).

## IDE false positives

Same as the real driver package: VSCode IntelliSense may report missing ROS include paths.
Ignore — `colcon build` is authoritative.

## Remote

Not yet pushed to a remote — created locally in `fsc_autopilot_ws/src/`, own git history
separate from sibling packages (each `src/` entry in this workspace is an independent repo).
