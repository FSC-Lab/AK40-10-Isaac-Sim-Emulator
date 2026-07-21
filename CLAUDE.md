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
- **EXTERNAL mode: confirmed working, including requirement #5** (user-tested, live, drone
  hovering). Two real bugs were found and fixed getting here — both written up in detail under
  "Payload mass must match..." and the velocity-filtering note in "Shaft dynamics" above:
  (1) `~/cable_state`/`~/joint_state` were publishing raw, unfiltered Isaac Sim velocity, which
  made `cable_torque_ctrl_node`'s sliding-mode/L1-adaptive law chatter even while just holding
  position with no reference; (2) the sim's payload mass (0.32 kg) didn't match
  `cable_torque_ctrl_node`'s deployed `mass=0.565` kg, causing systematically wrong gravity
  feedforward. With both fixed: `ros2 launch ak_motor_driver cable_torque_ctrl.launch.py` +
  `cable_square_ref_node` (both **unmodified**) correctly drive the simulated cable to track a
  commanded reference — **requirement #5 validated**.
  - **A small residual oscillation (chatter) persists even after both fixes** — bounded, not
    growing, effort staying safely within `torque_limit_upper/lower`. Traced (not fixed, and not
    considered a bug): cable velocity swings landed right at the edge of `smc_phi=0.15` m/s (the
    controller's own sliding-mode boundary layer width, in `cable_torque_ctrl_params.yaml`) —
    classic boundary-layer chatter behavior for this controller type, running against this sim's
    particular noise/discretization profile. Ruled out as sim-side bugs before accepting this:
    poll-rate mismatch (both sides are 100 Hz already), thrust/lift insufficiency (Iris body is
    1.5 kg, ~41 N max thrust vs ~20.5 N loaded weight, ~2:1 margin), and PX4 attitude-control
    gain mismatch (SPEED mode, which bypasses `cable_torque_ctrl_node` entirely, stayed clean
    with the same heavier payload). Since `cable_torque_ctrl_node` must run unmodified, further
    reducing this would mean tuning its own SMC/L1 parameters — out of scope here.
  - **Not yet verified**: the `off→standby→running` state transitions and authority hierarchy in
    isolation (implicitly exercised during the above, but not explicitly checked step-by-step),
    the `RUNNING→STANDBY` fallback, and requirement #6 (GUI's Stop/Emergency-Stop/Exit-External
    buttons correctly overriding an active external controller).

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

## PD+L1 chatter A/B variant (`cable_torque_ctrl_pd_l1_node`)

A second controller node, `src/cable_torque_ctrl_pd_l1_node.cpp`, built alongside this
emulator (same package, `ak_motor_cable_control_emulator`) purely as a diagnostic — **not**
part of the emulator itself, and not something the real hardware stack knows about. Purpose:
test the "Verification status" conclusion above that the residual EXTERNAL-mode chatter is
caused by `cable_torque_ctrl_node`'s own BLSMC sliding-mode switching term (`smc_phi` boundary
layer), not by remaining sim/timing bugs, by swapping out *only* that term for a standard PD
term and re-running the exact same live test.

**Design**: a near-verbatim copy of `AK40-10-ROS2-Bridge/src/cable_torque_ctrl_node.cpp`, same
node role and same public interface (`~/cable_state` sub, `~/reference` sub, `~/ext_torque_cmd`
pub, `~/ext_torque_enable` pub, `~/arm`/`~/disarm` services, `~/debug` pub, `~/ude_disturbance`
pub) so it's a drop-in swap for `cable_torque_ctrl_node` in `cable_torque_ctrl.launch.py` (own
launch file provided: `launch/cable_torque_ctrl_pd_l1.launch.py`, same remapping pattern). Kept
byte-for-byte identical: the L1 adaptive state predictor/adaptation law and the UDE monitor.
Replaced: `s = e_v + kp_c*e_p` / `tau_sw = -r*M_eff_hat*smc_eta*sat(s/smc_phi)` / the
`-kp_c*e_v` term folded into BLSMC's `tau_eq`, all removed in favor of a plain `tau_pd =
r*M_eff_hat*(-kp_pd*e_p - kd_pd*e_v)` added to the same `tau_ff` feedforward (dynamics +
gravity + friction, L1-adapted). This substitution is valid specifically *because* the L1
predictor/adaptation law only reads back `tau_applied_prev_` and `tau_f_prev_` (see the control
law comment at the top of the `.cpp`) — it has no dependency on *how* that prior torque was
computed, so it runs unchanged whether BLSMC or PD produced it.

New params (`config/cable_torque_ctrl_pd_l1_params.yaml`): `kp_pd` (default 10.0 N·m/m),
`kd_pd` (default 4.0 N·m·s/m) — replace `kp_c`/`smc_eta`/`smc_phi`. Plant, L1, UDE, timing
copied unchanged from `cable_torque_ctrl_params.yaml`.

**Friction feedforward diverges from `cable_torque_ctrl_node` on purpose**: viscous drag only,
no Coulomb term (`coulomb_friction` isn't even a declared parameter here — removed, not
zeroed). First live test (2026-07-11) carried the real node's Coulomb value over unchanged and
found it produced a **sustained ~23cm limit-cycle oscillation** while just holding position
with zero reference — diagnosed via `~/debug`: `e_p` amplitude was 0.228m in the first quarter
of a 12s sample window and 0.227m in the last quarter (not decaying, not growing — a true limit
cycle), and `tau_ff` swung by ~0.12 N·m in lockstep with each velocity-sign flip, matching
`2×coulomb_friction`. Root cause: at the velocities involved, `tanh(omega/deadband)` saturates
to essentially `sign(velocity)` — a relay force always in phase with velocity, which does net
positive work on the system every cycle. On real hardware that's fine because it's cancelling
*real* Coulomb friction of about that magnitude; in this sim, the shaft-dynamics model has no
Coulomb friction at all (see "Shaft dynamics" above — viscous drag only, by explicit scope
decision), so the term has nothing to cancel and instead pumps energy in continuously. `kd_pd`
alone couldn't dissipate that continuous injection, so the loop never converged to a fixed
point. **Removed the term entirely** (declare_parameter for `coulomb_friction` and
`friction_velocity_deadband` deleted, `tau_f` is now just `viscous_drag_ * omega`) rather than
leaving it as a zeroable parameter, since there's no physical friction here for it to ever
compensate.

This is itself a relevant data point for the A/B test's actual question: it suggests the real
BLSMC controller's switching term may have been partially *absorbing/masking* this same
friction-overcompensation effect (as small bounded chatter) rather than *causing* the chatter —
removing SMC and using raw PD exposed a much larger instability, not a cleaner one, until the
mismatched friction feedforward was also removed. Worth keeping in mind when interpreting the
eventual comparison result.

**Debug topic layout differs** from the real node's (no `s`/`tau_sw`/`tau_eq` concepts here):
`~/debug` is `[tau, e_v, e_p, v_c, p_c, acc_ref, tau_d_hat, tau_ff, tau_pd, tau_f, theta_f_1,
theta_f_2]` (12 elements, vs the real node's 13). `tau_f` is now viscous-only.

**Status — confirmed working (2026-07-11)**: after removing the Coulomb term, user confirmed
live that the controller runs smoother with no chattering, holding position with no reference.

**Second bug found getting the live square-wave reference test running**:
`cable_square_ref.launch.py` (in `AK40-10-ROS2-Bridge`, must stay unmodified) hardcodes its
remap to `/cable_torque_ctrl_node/reference` — not parameterized by a launch arg the way
`cable_torque_ctrl.launch.py` is by `cable_ctrl_node`. Since this test runs
`cable_torque_ctrl_pd_l1_node` instead, the reference never reached it. Fix: run the same
unmodified `cable_square_ref_node` executable directly (`ros2 run`, not the launch file) with a
manual remap override — see README.md's PD+L1 section for the exact command. **Third,
self-inflicted bug on the first attempt at that fix**: an unquoted `-r ~/reference:=...` gets
tilde-expanded by bash itself (`~` → `/home/<user>`) before `ros2` ever sees it, so the remap's
"from" side silently stops matching the node's actual private topic name and the remap does
nothing with no error. Fix: quote the whole remap argument (`-r '~/reference:=...'`) so bash
leaves the `~` for ROS2 to resolve.

With both fixed, the live A/B test (real square-wave reference driving
`cable_torque_ctrl_pd_l1_node`) is running. This means the Coulomb-friction-feedforward
mismatch — not `smc_phi` boundary-layer chatter alone — is at least a major contributor to the
chatter originally observed with the real `cable_torque_ctrl_node` (which has the same
mismatched Coulomb feedforward against this frictionless-in-Coulomb-terms sim plant); the SMC
switching term may have been masking/absorbing this effect as small chatter rather than causing
it outright. Not yet isolated exactly how much of the original chatter was `smc_phi` vs. this
friction mismatch — would need re-running the original `cable_torque_ctrl_node` test with
`coulomb_friction` overridden to `0` there too (diagnostic-only override, not a permanent change
to that must-stay-unmodified file's deployed config) to fully separate the two effects.

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
`~5.54 N` (`(rod_b_mass+payload_mass)*g`, with `payload_mass` set to match
`cable_torque_ctrl_node`'s deployed `mass=0.565` — see the "Payload mass must match" note below)
load on the cable, which would require an absurdly large `kd_speed` before any commanded
velocity produced a visible response (verified empirically
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
`kp_speed` large enough to reject the ~5.54 N gravity load with small error (`kp_speed≈2000`,
first attempted) implies a natural frequency of `sqrt(kp_speed/mass) ≈ 79 rad/s`. That's far too
stiff for a control loop that has to round-trip through ROS2 *and* Isaac Sim's own
GUI-rendering, sub-100Hz physics/render loop — the mismatch between the spring's bandwidth and
the loop's effective update rate destabilized the physics solver (**symptom: the payload
suddenly moving very fast / the sim visibly glitching**, confirmed empirically). Adding
`gravity_feedforward_n` (a constant, matching `(rod_b_mass+payload_mass)*g` for the default
scenario) cancels the disturbance directly, so `kp_speed`/`kd_speed` only have to provide gentle
tracking dynamics — current defaults (`kp_speed=10`, `kd_speed=4`) target a much safer
`sqrt(10/0.565) ≈ 4.2 rad/s`.

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

### Payload mass must match `cable_torque_ctrl_node`'s deployed "mass" param

Found on real test 2026-07-11: `cable_torque_ctrl_node`'s gravity feedforward (`tau_p =
-mass*drum_radius*g`) and its L1 adaptive law's nominal initialization (`M_eff_nom =
ude_inertia/drum_radius² + mass`, computed once in its constructor) both depend directly on its
`mass` parameter — deployed as `0.565` kg in
`AK40-10-ROS2-Bridge/config/cable_torque_ctrl_params.yaml` (the real hardware's actual payload
weight), loaded unconditionally by the unmodified `cable_torque_ctrl.launch.py` (no launch
argument to override it, and no live-parameter-update path in the node — `mass_` and the L1
nominal values are both fixed at startup). If the sim's actual hanging load
(`rod_b_mass+payload_mass`) doesn't match that `0.565` kg, the controller's gravity compensation
is systematically wrong by the difference, forcing the adaptive estimator to continuously fight
a bad nominal model — with the default scenario masses at the time (`payload_mass=0.3`,
`rod_b_mass=0.02` → `0.32` kg, **nearly half** what the controller assumes), this produced
growing oscillation and repeated torque-limit clamping even with the controller just holding
position (no external reference active).

**Fix**: rather than override the controller's parameters (which would mean not running
`cable_torque_ctrl.launch.py` truly unmodified, and risks drifting the deployed config away from
its real-hardware calibration), the Isaac Sim scenario's own default masses were changed instead
— `application/slungload/02_px4_single_drone_payload_variable_length_cable.py`'s
`payload_mass` default is now `0.545` kg (`rod_b_mass=0.02` unchanged →
`rod_b_mass+payload_mass=0.565` kg exactly), and this emulator's own `gravity_feedforward_n`
default was updated to match (`0.565*9.81≈5.54` N) so SPEED mode stays consistent too. **If
either side's masses are ever changed independently, they will silently drift out of sync
again** — check both `cable_torque_ctrl_params.yaml`'s `mass` and the Isaac Sim scenario's
`rod_b_mass+payload_mass` together, not just one.

### `effective_radius` — the torque/force bridge for EXTERNAL mode only

Isaac Sim's rigid bodies (rod_a/rod_b) apply a **direct linear force** to the cable — there is no
drum or rotating shaft in the physics scene itself. The real hardware's `~/ext_torque_cmd` (N·m)
is in motor-shaft units, scaled by the real drum's `drum_radius`. `effective_radius` (default
`0.036` m — the real hardware's **deployed** value from
`AK40-10-ROS2-Bridge/config/cable_control_params.yaml`, not the `0.0175` fallback hardcoded in
that package's `.cpp`) is the conversion factor used for **EXTERNAL mode**
(`force_N = tau_to_cable_Nm / effective_radius` — see "Shaft dynamics" below for
`tau_to_cable`) and for synthesizing `~/joint_state`, so `torque_limit_upper/lower` keep their
real, calibrated meaning. **SPEED mode does not use this parameter at all** — see above.

### Shaft dynamics — EXTERNAL mode only, simulates the motor's own inertia/drag

Isaac Sim's rigid bodies model the mechanical load (cable rods + payload) but have no concept of
the motor's own rotating shaft — from Isaac Sim's point of view, `ext_torque_cmd` (converted
through `effective_radius`) would otherwise arrive at the cable as a force with zero actuator
dynamics, as if the motor itself were massless and frictionless. Real hardware isn't: some of
`ext_torque_cmd` goes into accelerating the shaft's own rotor inertia and overcoming its viscous
drag, and only the remainder actually pulls on the cable. `cable_torque_ctrl_node`'s own BLSMC
control law explicitly assumes this (`M_eff = J/r² + m`), so a plant with no shaft dynamics at
all isn't a faithful stand-in for validating that controller.

Since the drum has no slip, the shaft and the cable share **one rigid DOF** — shaft angular
velocity is just the measured cable velocity through `effective_radius`
(`omega_retract = -last_extension_vel_ / effective_radius`, retract-positive), so there's no
separate integrated shaft state to drift out of sync with Isaac Sim's own physics. Each poll
tick, `compute_and_publish_force()` just reads the already-computed `omega_filtered_retract_`/
`omega_dot_retract_` (see below) and:

```
tau_shaft_absorbed = shaft_inertia_kg_m2 * omega_dot_retract_ + viscous_drag_nms_per_rad * omega_filtered_retract_
tau_to_cable        = ext_torque_cmd_ - tau_shaft_absorbed
force_retract        = tau_to_cable / effective_radius
```

**Bug found on first real test (2026-07-11) and fixed**: the initial version differentiated
`omega` once per *poll tick*, dividing by the fixed `poll_period_s_` (100 Hz). But
`last_extension_vel_` only actually changes when Isaac Sim's `state/cable` message arrives, which
runs on Isaac Sim's own physics-step cadence — not necessarily locked to this node's poll timer.
Whenever the real inter-sample time didn't match the assumed fixed period (message bursts,
render/physics stalls, anything), the finite difference was silently scaled by however wrong
that assumption was — the sim diverged violently (payload shooting off at high speed) the instant
`RUNNING` engaged, i.e. the instant this code path first executed for real. **Fix**: the
derivative is now computed inside `on_state_cable()` — the callback where a genuinely fresh
sample actually arrives — using the *actual measured* wall-clock time since the last real sample
(`last_state_time_`), not an assumed period. An EMA low-pass filter (`omega_filter_alpha`,
default 0.2) is applied to `omega` before differentiating as a second, complementary layer
against plain sample-to-sample sensor noise (worth keeping even with the timing fix, since
differentiation still amplifies whatever noise remains). Both `omega_filtered_retract_` and
`omega_dot_retract_` are updated unconditionally on every real sample (not just while
`RUNNING`), so entering `RUNNING` always sees an up-to-date value and never produces a spurious
spike from stale state on the first tick after entry.

**Lesson for anyone extending this**: never differentiate a value against an assumed/fixed
timer period if the value's actual update rate isn't guaranteed to match that timer — use the
real measured time between updates instead. This is also why `p_des_retract_`'s SPEED-mode
*integration* (`p_des_retract_ += v_des_retract * poll_period_s_`) was fine despite using the
same fixed `poll_period_s_`: integration against a slightly-wrong assumed dt just accumulates a
small, bounded error; differentiation against one can spike unboundedly for a single mistimed
sample.

**Deliberately no Coulomb/static-friction term** — viscous drag only, per explicit project scope
decision ("easy simulation"). `cable_torque_ctrl_node`'s own `coulomb_friction_` feedforward
term becomes a harmless near-no-op against a plant that has none.

**Defaults are the exact values `cable_torque_ctrl_node` is calibrated against** —
`shaft_inertia_kg_m2=0.000995` (its `ude_inertia`) and `viscous_drag_nms_per_rad=0.00038` (its
**deployed** `viscous_drag` in `cable_torque_ctrl_params.yaml`, not the `.cpp` fallback of
`0.00140` — same "deployed yaml wins over .cpp fallback" pattern as `effective_radius` above).
Using the real calibrated numbers (rather than arbitrary placeholders) is what makes this a
faithful proxy for validating that specific controller, not just "some plant with some inertia."

**Deliberately NOT applied to SPEED mode** — SPEED mode keeps its existing simplified
position-hold PD+feedforward law exactly as before (see SPEED mode section above). SPEED mode
exists for convenient bench/setup testing (e.g. getting the drone hovering before switching to
EXTERNAL mode), not for controller validation, and the project scope decision was to keep it
simple there rather than route it through the same shaft-dynamics filter.

**Noise consideration**: `omega_dot_retract` is a finite difference of an already-measured
velocity (i.e. a second derivative of position) — at these calibrated `shaft_inertia`/
`viscous_drag` magnitudes (both tiny relative to the ~0.11 N·m gravity torque already in play)
this hasn't shown up as a practical issue, but if either parameter is increased significantly on
a retune, watch for injected force noise/jitter and consider low-pass-filtering `omega_retract`
before differentiating if it becomes one.

**Second bug found on the next real test (2026-07-11) and fixed**: `cable_torque_ctrl_node`
armed with no `~/reference` (so just its own default gravity-hold-at-arm-position law — nothing
external commanding it) still produced growing oscillation and repeated torque-limit clamping.
Root cause: `~/cable_state`'s velocity field, and `~/joint_state`'s, were still publishing raw
`last_extension_vel_` straight from Isaac Sim — `omega_filtered_retract_` (built for the shaft-
dynamics computation above) was never applied to what actually gets sent to downstream
consumers. `cable_torque_ctrl_node`'s sliding-mode/L1-adaptive control law is specifically
designed to react sharply to velocity and tracking-error (that's the point of sliding-mode
control), and is tuned against real hardware's comparatively clean encoder-over-CAN feedback —
feeding it Isaac Sim's noisier raw rigid-body velocity was enough to make it chatter, even while
just trying to hold still with zero external reference. **Fix**: both `~/cable_state` and
`~/joint_state` now publish the same `omega_filtered_retract_` used internally (converted back
to the right sign/units for each topic), so there's one filtered "sensor" the whole node agrees
on — not two independently-noisy views of the same physical quantity. Position is deliberately
left unfiltered (real encoders are typically far cleaner on position than on differentiated
velocity — filtering only the field that's actually noisy mirrors what a real sensor chain would
look like, rather than over-filtering everything defensively).

**Lesson**: a filter built for one internal computation doesn't automatically protect everything
downstream — check every place a noisy raw value still escapes unfiltered, especially anything
published to an external, unmodifiable, aggressively-tuned consumer like `cable_torque_ctrl_node`.

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
| `gravity_feedforward_n` | 5.54 (N) | Cancels the constant `(rod_b_mass+payload_mass)*g` gravity load directly so `kp_speed`/`kd_speed` don't have to. Re-read on `mode_cmd="speed"`. Retune if scenario masses change (must match `cable_torque_ctrl_node`'s deployed `mass` — see "Payload mass must match" above) |
| `kp_speed` | 10.0 (N/m) | SPEED-mode PD position (tracking, not disturbance-rejection) gain. Re-read from param server on `mode_cmd="speed"`. Targets `sqrt(kp_speed/mass)≈4.2 rad/s` — a `kp_speed=2000` first attempt (relying on `kp` alone to also fight gravity) was stiff enough to destabilize the physics solver through the ROS2+Isaac-Sim loop; keep this soft and raise gradually if retuning |
| `kd_speed` | 4.0 (N·s/m) | SPEED-mode PD velocity gain. Close to critically damped for `kp_speed=10` and this scenario's ~0.565 kg cable+payload mass |
| `command_timeout_ms` | 500.0 | On timeout, holds position via PD+feedforward (no separate `kd_watchdog` param — see Watchdogs above) |
| `heartbeat_timeout_ms` | 1500.0 | |
| `torque_limit_upper` / `torque_limit_lower` | ±1.5 (N·m) | Clamped on `~/ext_torque_cmd` receipt, throttled `[WARN]` on clamp |
| `shaft_inertia_kg_m2` | 0.000995 (kg·m²) | EXTERNAL-mode only — see "Shaft dynamics" above. Matches `cable_torque_ctrl_node`'s `ude_inertia` exactly |
| `viscous_drag_nms_per_rad` | 0.00038 (N·m·s/rad) | EXTERNAL-mode only, viscous only (no Coulomb term) — matches `cable_torque_ctrl_node`'s **deployed** `viscous_drag` (not its `.cpp` fallback of 0.00140) |
| `omega_filter_alpha` | 0.2 | EMA coefficient (0-1) on shaft speed before differentiating for the inertia term. 1.0 = unfiltered. See "Shaft dynamics" above for the divergence bug this (plus the on_state_cable() timing fix) resolved |

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
