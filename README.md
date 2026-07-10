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
the cable — the poll loop damps toward zero velocity instead, same as the
real driver's command-watchdog fallback.

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

## Sign conventions

- `~/cable_state` (length, velocity) and external-mode torque/force are
  already sign-aligned between the real hardware and the sim (positive =
  retract in both) — passed through with only a unit scale (`effective_radius`).
- `~/command.velocity[0]` (SPEED mode, rad/s) is positive = retract in the
  real hardware's raw motor-shaft convention, but the sim's `extension_vel`
  is positive = **extend**. The emulator flips this internally before running
  the `kd_speed * (v_des − v_actual)` control law — see the file header
  comment in `src/ak_motor_cable_control_node.cpp` for the full derivation.

## `effective_radius`

The sim applies a direct linear force to the cable (no drum). `effective_radius`
(default `0.036` m, matching the real hardware's deployed `drum_radius`) is
the torque(N·m)/force(N) and rad/s/m/s scale factor used throughout, so that
`kd_speed`, `torque_limit_upper`, `torque_limit_lower` keep the same
calibrated, real-world meaning as on hardware.
