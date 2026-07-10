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

## Verification status

- **SPEED mode: confirmed working** (user-tested, live, with the drone actually hovering).
  Took three iterations to get right — see the SPEED mode section under Key design decisions
  for the full debugging story (units mismatch → too-stiff PD → gravity feedforward + soft
  gains) and the Watchdogs section for a follow-up fix (Stop was dropping the payload after
  ~500ms; fixed by removing the weak watchdog fallback in favor of always holding position).
- **EXTERNAL mode: implemented, NOT yet runtime-verified.** Next things to check when resuming:
  state transitions (`off→standby→running` via `~/enable_external_mode` + `~/ext_torque_enable`),
  the `ext_torque_cmd` sign convention (should already be correct — see the sign-convention
  table below — but hasn't been confirmed against a real hovering test), the authority
  hierarchy (ground station's `~/ext_mode_cmd="off"` should always revoke access even from an
  active external controller), and the `RUNNING→STANDBY` fallback (should hold position
  cleanly via the same fix as SPEED's stale-command case, but not yet directly observed).
  Also unvalidated: requirement #5 (`ros2 launch ak_motor_driver cable_torque_ctrl.launch.py`,
  unmodified, driving the sim cable in external mode) and #6 (GUI's Stop/Emergency-Stop/Exit-
  External buttons correctly override the emulator).

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
while in those modes are **not** applied to the cable — the poll loop just **holds the cable at
its current position** (same PD+feedforward law as SPEED mode, with the setpoint frozen), the
same behavior used for a stale/missing SPEED command. This is a deliberate simplification, not
a bug: nothing downstream currently needs real position or torque-direct tracking against the
sim cable.

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

### SPEED mode: m/s + PD — a deliberate, temporary divergence from the real driver

On real hardware, `~/command.velocity[0]` is raw motor-shaft **rad/s** and SPEED mode is a
pure velocity damper: `force = kd_speed*(v_des - v_actual)`, `kp` always `0`. That law has no
way to reject a *steady* disturbance — at steady state, `v_actual ≈ v_des -
disturbance_force/kd_speed`. For this scenario's default masses, gravity puts a constant
`~3.14 N` (`(rod_b_mass+payload_mass)*g`) load on the cable, which would require an absurdly
large `kd_speed` before any commanded velocity produced a visible response (verified empirically
— the real driver's `kd_speed=0.5`, run through `effective_radius=0.036`, produced ~0.009 N of
force for a typical command: no response at all).

So in this emulator, `~/command.velocity[0]` is instead interpreted **directly in m/s**
(retract-positive, no `effective_radius` scaling), and SPEED mode integrates that into a
position setpoint each poll tick (`p_des_retract_ += v_des_retract * poll_period_s_`) and runs a
**PD + feedforward** law against it: `force = gravity_feedforward_n + kp_speed*(p_des_retract_ -
p_actual) + kd_speed*(v_des_retract - v_actual)`.

**Why feedforward, not just a bigger `kp`**: a `kp` term alone *can* reject a constant
disturbance with arbitrarily small steady-state error (`e_p_ss = disturbance_force/kp_speed`) —
the real driver's pure-damper law fundamentally cannot, no matter how it's tuned — but making
`kp_speed` large enough to reject the ~3.14 N gravity load with small error (`kp_speed≈2000`,
first attempted) implies a natural frequency of `sqrt(kp_speed/mass) ≈ 79 rad/s`. That's far too
stiff for a control loop that has to round-trip through ROS2 *and* Isaac Sim's own
GUI-rendering, sub-100Hz physics/render loop — the mismatch between the spring's bandwidth and
the loop's effective update rate destabilized the physics solver (**symptom: the payload
suddenly moving very fast / the sim visibly glitching**, confirmed empirically). Adding
`gravity_feedforward_n` (a constant, matching `(rod_b_mass+payload_mass)*g` for the default
scenario) cancels the disturbance directly, so `kp_speed`/`kd_speed` only have to provide gentle
tracking dynamics — current defaults (`kp_speed=10`, `kd_speed=4`) target a much safer
`sqrt(10/0.32) ≈ 5.6 rad/s`.

**`p_des_retract_` is re-synced to the actual position (not integrated) whenever SPEED isn't
actively tracking a fresh command** — on `~/enable`, on `mode_cmd="speed"`, on any
watchdog/stale-command tick, and on the `RUNNING→STANDBY` external-mode fallback. Skipping this
would let the setpoint drift arbitrarily far from reality while idle/disabled/in another mode,
producing a large snap force the instant tracking resumes.

**If you retune these**: `gravity_feedforward_n` must match the actual scenario masses (it's
not derived automatically), and `kp_speed`/`kd_speed` should stay soft enough that
`sqrt(kp_speed/mass)` stays well below whatever the real achievable Isaac-Sim/ROS2 loop rate
is — if in doubt, raise gently and re-test rather than guessing a large value up front.

**Planned follow-up**: the real `AK40-10-ROS2-Bridge` driver is intended to be updated to this
same m/s + PD convention later, at which point the two will converge again. Until then, this
emulator's SPEED mode does **not** behave identically to the real driver's — don't assume
hardware-tuned `kd_speed` values transfer here, and don't assume this emulator's SPEED-mode
`~/command.velocity` units match hardware's rad/s.

### `effective_radius` — the torque/force bridge for EXTERNAL mode only

The sim applies a **direct linear force** to the cable (no drum, no rotating shaft). The real
hardware's `~/ext_torque_cmd` (N·m) is in motor-shaft units, scaled by the real drum's
`drum_radius`. `effective_radius` (default `0.036` m — the real hardware's **deployed** value
from `AK40-10-ROS2-Bridge/config/cable_control_params.yaml`, not the `0.0175` fallback
hardcoded in that package's `.cpp`) is the conversion factor used for **EXTERNAL mode**
(`force_N = ext_torque_cmd_Nm / effective_radius`) and for synthesizing `~/joint_state`, so
`torque_limit_upper/lower` keep their real, calibrated meaning. **SPEED mode does not use this
parameter at all** — see above.

### Sign conventions — traced explicitly against both codebases, not assumed symmetric

| Quantity | Real driver convention | Sim (`ROS2CableWinchBackend`) convention | Conversion applied |
|---|---|---|---|
| `~/cable_state` length/velocity | length **decreases** on retract; velocity **negative** on retract | `extension` increases on extend (decreases on retract); `extension_vel` same sign | **None** — direct passthrough, `cable_length = extension - extension_offset_`, `cable_velocity = extension_vel` |
| `~/ext_torque_cmd` (N·m) → sim force (N) | positive = retract (default `motor_direction=1` in `cable_torque_ctrl_params.yaml`) | positive `command/force` = retract | **None** in sign, just scaled: `force_N = ext_torque_cmd_Nm / effective_radius` |
| `~/command.velocity[0]` (SPEED mode) | rad/s, **positive = retract**, raw motor-shaft convention *(real driver — not yet updated to m/s)* | sim's `extension_vel` is **positive = extend** (opposite of this emulator's m/s retract-positive convention) | **Sign flip required** — both desired and actual velocity are converted into one internal "retract-positive" frame before the PD law runs: `v_des_retract = command.velocity[0]` (m/s, this emulator), `v_actual_retract = -extension_vel` |
| `~/joint_state` (synthesized, rad/rad-s) | positive = retract | — | `position = -(extension-extension_offset_) / effective_radius`, `velocity = -extension_vel / effective_radius` |

**This is the easiest place to introduce a silent bug**: `~/cable_state` and external-mode
torque are sign-aligned between the two systems already; `~/command.velocity` is not (and, as of
this emulator, isn't even in the same units as the real driver — see SPEED mode section above).
A positive commanded SPEED-mode velocity must **retract** the cable — if it extends instead, the
flip (`v_actual_retract = -extension_vel`) got dropped or double-applied somewhere. Verify this
first when debugging any SPEED-mode behavior that looks backwards.

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
heartbeat-lost fallback, which sets `control_mode_=SPEED` with `current_cmd_velocity_=0`. On
this emulator that now means **hold position** (not just "zero velocity") — it lands in the
SPEED-mode PD+feedforward path with a zero target velocity, which per the Watchdogs section
below actively holds rather than passively damping. This has not been directly observed yet
(see Verification status) — worth confirming the transition is smooth (no snap) when this is
tested. See the real driver's own `CLAUDE.md` (`AK40-10-ROS2-Bridge/CLAUDE.md`, § External
mode) for the full state diagram and activation sequence — it applies here unchanged.

### Watchdogs

`command_timeout_ms` (default 500 ms) and dual `heartbeat_timeout_ms` (default 1500 ms,
matching the real driver's **deployed** yaml value, not its `.cpp` fallback of 1000 ms; either
`~/heartbeat` or `~/heartbeat_external` fresh keeps the motor enabled) — timeout detection is
ported verbatim from the real driver. **The fallback behavior on command timeout is not**:
the real driver drops to a weak `kd_watchdog` pure damper; here, a stale command instead
freezes the SPEED-mode PD setpoint at the current position and holds there (see the SPEED
mode section above) — confirmed necessary after the real driver's fallback pattern let the
payload free-fall (this sim's cable has no mechanical friction to lean on, unlike real
hardware). Heartbeat-loss and its external-mode-cleared fallback are otherwise ported verbatim.

## Parameters (`config/cable_control_emulator_params.yaml`)

| Parameter | Default | Notes |
|---|---|---|
| `isaac_winch_prefix` | `"cable_winch_0/"` | Topic prefix `ROS2CableWinchBackend` publishes/subscribes under in Isaac Sim |
| `poll_rate_hz` | 100.0 | Matches real driver; also the fixed `dt` for the SPEED-mode setpoint integrator |
| `effective_radius` | 0.036 (m) | EXTERNAL-mode torque/force scale factor + `~/joint_state` synthesis only — **not** used by SPEED mode |
| `gravity_feedforward_n` | 3.14 (N) | Cancels the constant `(rod_b_mass+payload_mass)*g` gravity load directly so `kp_speed`/`kd_speed` don't have to. Re-read on `mode_cmd="speed"`. Retune if scenario masses change |
| `kp_speed` | 10.0 (N/m) | SPEED-mode PD position (tracking, not disturbance-rejection) gain. Re-read from param server on `mode_cmd="speed"`. Targets `sqrt(kp_speed/mass)≈5.6 rad/s` — a `kp_speed=2000` first attempt (relying on `kp` alone to also fight gravity) was stiff enough to destabilize the physics solver through the ROS2+Isaac-Sim loop; keep this soft and raise gradually if retuning |
| `kd_speed` | 4.0 (N·s/m) | SPEED-mode PD velocity gain. Close to critically damped for `kp_speed=10` and this scenario's ~0.32 kg cable+payload mass |
| `command_timeout_ms` | 500.0 | On timeout, holds position via PD+feedforward (no separate `kd_watchdog` param — see Watchdogs above) |
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
