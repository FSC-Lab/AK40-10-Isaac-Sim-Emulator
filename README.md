# AK40-10 Isaac Sim Emulator

Drop-in replacement for `ak_motor_driver`'s `ak_motor_cable_control_node`
(package `AK40-10-ROS2-Bridge`), backed by the variable-length slung-load
cable in Isaac Sim (`ROS2CableWinchBackend`, `fsc_PegasusSimulator`) instead
of real CAN hardware. The ground station GUI (`fsc_ak_actuator_ground_control`)
and `cable_torque_ctrl_node` run against this node completely unmodified.

```
GUI / cable_torque_ctrl_node  <--ROS2-->  ak_motor_cable_control_node (this package)  <--ROS2-->  Isaac Sim (ROS2CableWinchBackend)
```

## Scope

Only **SPEED mode** and **EXTERNAL mode** are actuated (drive real forces
into the sim). POSITION and TORQUE `mode_cmd` switches are accepted so the
GUI never errors, but commands received in those modes are not applied to
the cable — the poll loop just **holds the cable at its current position**
instead (see below — unlike the real driver, there's no weak damper
fallback, since this sim's frictionless cable would let the payload
free-fall under gravity without active holding).

## Build

```bash
cd ~/Workspaces/fsc_autopilot_ws
colcon build --packages-select ak_motor_cable_control_emulator
source install/setup.bash
```

## Launch

```bash
# Isaac Sim scenario must already be running with the drone hovering
# (see fsc_PegasusSimulator/scripts/start_single_drone_sitl_payload_variable_cable.sh)

ros2 launch ak_motor_cable_control_emulator cable_control_emulator.launch.py
# optionally: isaac_winch_prefix:=cable_winch_1/

# Then, unmodified:
ros2 run ak_motor_ground_station ak_motor_ground_station
ros2 launch ak_motor_driver cable_torque_ctrl.launch.py
```

## SPEED mode: m/s + PD + gravity feedforward (diverges from the real driver, for now)

On real hardware, `~/command.velocity[0]` is raw motor-shaft **rad/s** and SPEED mode is a
pure velocity damper (`force = kd_speed*(v_des−v_actual)`, no position term). That law can't
reject a steady disturbance — here, the ~3.14 N gravity load on the default payload would need
an absurdly large `kd_speed` to produce any visible response. So in this emulator,
`~/command.velocity[0]` is instead **m/s directly** (no radius scaling), and SPEED mode runs a
**PD controller against a position setpoint integrated from that velocity command, plus an
explicit `gravity_feedforward_n` term** that cancels the constant gravity load directly (mirrors
`cable_torque_ctrl_node`'s own `tau_p=-m*g*r` feedforward). Splitting disturbance rejection
(feedforward) from tracking (PD) matters: an earlier version relied on `kp_speed` alone to also
fight gravity (`kp_speed=2000`), which made the loop stiff enough (~79 rad/s natural frequency)
to destabilize the physics solver through the ROS2 + Isaac Sim (GUI-rendering, sub-100Hz)
control loop — visible as the payload suddenly moving very fast / the sim glitching. Current
defaults target a much gentler ~5.6 rad/s. See the file header comment in
`src/ak_motor_cable_control_node.cpp` for the full derivation. **The plan is to update the real
`AK40-10-ROS2-Bridge` driver to this same convention later** — until then, this emulator's
SPEED mode does not behave identically to the real driver's.

**No separate watchdog fallback.** The real driver falls back to a weak `kd_watchdog` damper
when `~/command` goes stale (e.g. the GUI's Stop button publishes one command and then stops
republishing) — safe on real hardware because the winch's own mechanical friction keeps it from
free-falling. This sim's cable has no such friction, so a stale command instead freezes the PD
setpoint at the current position and the same PD+feedforward law holds it there — confirmed
necessary after an earlier version (matching the real driver's weak-damper fallback) let the
payload visibly drop to the ground ~500ms after pressing Stop.

## Sign conventions

- `~/cable_state` (length, velocity) and external-mode torque/force are
  already sign-aligned between the real hardware and the sim (positive =
  retract in both) — passed through with only a unit scale (`effective_radius`).
- `~/command.velocity[0]` (SPEED mode, m/s) is positive = retract by this
  emulator's convention (see above), but the sim's `extension_vel` is
  positive = **extend**. The emulator flips this internally before running
  the PD control law — see the file header comment in
  `src/ak_motor_cable_control_node.cpp` for the full derivation.

## PD+L1 chatter A/B test node (`cable_torque_ctrl_pd_l1_node`)

A diagnostic-only second controller node (not part of the emulator itself), built to test
whether `cable_torque_ctrl_node`'s residual EXTERNAL-mode chatter comes from its BLSMC
sliding-mode switching term. Same public interface as `cable_torque_ctrl_node` (drop-in swap),
same L1 adaptive core, but the switching term is replaced with a plain PD term (`kp_pd`/`kd_pd`).

```bash
ros2 launch ak_motor_cable_control_emulator cable_torque_ctrl_pd_l1.launch.py

# drive it with the real (unmodified) square-wave reference generator — note cable_square_ref
# .launch.py hardcodes its remap to /cable_torque_ctrl_node/reference, so run the executable
# directly instead and remap by hand (quote the remap - unquoted, bash tilde-expands "~" into
# your home directory before ros2 ever sees it, silently breaking the remap):
ros2 run ak_motor_driver cable_square_ref_node --ros-args \
  --params-file install/ak_motor_driver/share/ak_motor_driver/config/cable_square_ref_params.yaml \
  -r '~/reference:=/cable_torque_ctrl_pd_l1_node/reference'
```

### Arming

Same authority split as `cable_torque_ctrl_node`: the ground station owns the outer
`off ↔ standby` gate on the emulator's `ak_motor_cable_control_node`, and this node's own
`~/arm` service owns the inner `standby ↔ running` gate. `~/ext_torque_enable=true` has no
effect while the emulator is still `"off"`, so external mode must be opened first.

```bash
# 1. open external mode on the emulator (or click "Enable External Mode" in the ground station GUI)
ros2 service call /ak_motor_cable_control_node/enable_external_mode std_srvs/srv/Trigger {}
ros2 topic echo /ak_motor_cable_control_node/ext_mode_state --once   # expect "standby"

# 2. arm this node — captures the current position as the gravity-hold setpoint, resets L1,
#    and publishes ext_torque_enable=true (standby -> running)
ros2 service call /cable_torque_ctrl_pd_l1_node/arm std_srvs/srv/Trigger {}
ros2 topic echo /ak_motor_cable_control_node/ext_mode_state --once   # expect "running"
```

Disarm with `ros2 service call /cable_torque_ctrl_pd_l1_node/disarm std_srvs/srv/Trigger {}`.

**Friction feedforward is viscous-only here** (no Coulomb term, unlike `cable_torque_ctrl_node`)
— this sim's shaft model has no Coulomb/static friction at all, and carrying the real Coulomb
value over produced a sustained ~23cm limit-cycle oscillation even while just holding position
(a relay force in phase with velocity, injecting energy every cycle with no real friction to
cancel). **Confirmed live (2026-07-11)**: removing it made the controller smoother with no
chattering — this reframes the earlier chatter investigation to some degree; see this package's
own `CLAUDE.md` for the full writeup.

## `effective_radius`

The sim applies a direct linear force to the cable (no drum). `effective_radius`
(default `0.036` m, matching the real hardware's deployed `drum_radius`) is
the torque(N·m)/force(N) scale factor used for **EXTERNAL mode** and
`~/joint_state` synthesis, so that `torque_limit_upper`/`torque_limit_lower`
keep the same calibrated, real-world meaning as on hardware. It is **not**
used by SPEED mode, which has its own `kp_speed`/`kd_speed` gains tuned
directly in the sim's native m/s/N units (see above).
