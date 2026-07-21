// MIT License
// Copyright (c) 2026 FSC Lab
//
// Isaac Sim-backed drop-in emulator for AK40-10-ROS2-Bridge's
// ak_motor_cable_control_node (see that repo's src/ak_motor_cable_control_node.cpp,
// which this file's structure intentionally mirrors so the two stay easy to diff).
//
// Real hardware:  GUI/controller <-> [ak_motor_cable_control_node] <-> CAN <-> AK40-10 motor
// This emulator:  GUI/controller <-> [ak_motor_cable_control_node] <-> ROS2 <-> Isaac Sim
//                                                                      (ROS2CableWinchBackend)
//
// Only SPEED mode and EXTERNAL mode are actuated (drive real forces into the sim).
// POSITION/TORQUE mode_cmd switches are accepted so the GUI never errors, but commands
// received in those modes are not applied to the cable — the poll loop just holds the cable
// at its current position instead (see compute_and_publish_force()).
//
// EXTERNAL mode simulates shaft inertia + viscous drag (no static/Coulomb friction) between
// ext_torque_cmd and the force actually delivered to the cable — see the "Shaft dynamics"
// section in compute_and_publish_force() for the derivation. SPEED mode is NOT given this
// treatment; it keeps its existing simplified position-hold PD+feedforward law exactly as-is
// (a deliberate scope decision — SPEED mode exists for convenient bench/setup testing, not for
// validating cable_torque_ctrl_node, which is what EXTERNAL mode is for).
//
// DELIBERATE DIVERGENCE FROM THE REAL DRIVER (temporary): on real hardware, SPEED mode's
// ~/command.velocity[0] is raw motor-shaft rad/s and the control law is a pure damper
// (force = kd_speed*(v_des-v_actual), no position term, and no gravity feedforward) — see
// AK40-10-ROS2-Bridge's ak_motor_cable_control_node.cpp. That's fine on real hardware, where
// the winch's own mechanical friction/self-locking keeps a stale/disabled command from letting
// the load free-fall. This sim's cable is frictionless: a pure damper (or its watchdog
// fallback) would let the ~5.54 N gravity load on the default payload free-fall to the ground
// whenever there's no fresh command driving it (confirmed empirically — e.g. the GUI's Stop
// button publishes one command then stops, so anything relying on continuous commands to hold
// position drops the payload ~500ms later). So here, ~/command.velocity[0] is instead
// interpreted directly in m/s (retract-positive, no radius scaling), and SPEED mode runs PD +
// an explicit gravity_feedforward_n term against a position setpoint integrated from that
// velocity command. Whenever there's no fresh SPEED command to track, the same PD+feedforward
// law holds the setpoint at the current actual position instead of falling back to a weak
// damper — see compute_and_publish_force() for the full derivation. The real
// AK40-10-ROS2-Bridge driver is planned to be updated to this same convention later; until
// then, this emulator's SPEED mode does NOT behave identically to the real driver's.

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ak_motor_cable_control_emulator {

class AkMotorCableControlNode : public rclcpp::Node {
 public:
  enum class ControlMode { SPEED, TORQUE, POSITION };

  // OFF: normal operation. STANDBY: cable zeroed, armed, previous mode active.
  // RUNNING: external torque command drives the cable.
  enum class ExternalModeState { OFF, STANDBY, RUNNING };

  AkMotorCableControlNode() : Node("ak_motor_cable_control_node") {
    declare_parameter("isaac_winch_prefix", "cable_winch_0/");
    declare_parameter("poll_rate_hz", 100.0);
    declare_parameter("effective_radius", 0.036);
    // SPEED-mode PD + feedforward, m/s command convention (see file header) — NOT the same
    // units as the real driver's kd_speed (N.m.s/rad). gravity_feedforward_n cancels the
    // constant gravity load directly (mirrors cable_torque_ctrl_node's own tau_p=-m*g*r
    // feedforward), so kp_speed/kd_speed only have to provide gentle tracking dynamics, not
    // disturbance rejection — keeping them soft enough to stay stable through the ROS2 +
    // Isaac Sim (GUI-rendering, sub-100Hz) control loop. An earlier attempt at kp_speed=2000
    // (implying a ~79 rad/s natural frequency) made kp alone fight gravity and was stiff
    // enough to destabilize the physics solver. Defaults below target ~4.2 rad/s, near-critical
    // damping, for this scenario's default ~0.565 kg cable+payload mass (matches
    // cable_torque_ctrl_node's deployed "mass" param, see CLAUDE.md) — retune (along with
    // gravity_feedforward_n) if scenario masses change.
    declare_parameter("gravity_feedforward_n", 5.54);  // N — (rod_b_mass+payload_mass)*g
    declare_parameter("kp_speed", 10.0);   // N/m
    declare_parameter("kd_speed", 4.0);    // N.s/m
    declare_parameter("command_timeout_ms", 500.0);
    declare_parameter("heartbeat_timeout_ms", 1500.0);
    declare_parameter("torque_limit_upper",  1.5);   // N.m
    declare_parameter("torque_limit_lower", -1.5);   // N.m
    // EXTERNAL-mode shaft dynamics (see "Shaft dynamics" note in compute_and_publish_force()).
    // Defaults match cable_torque_ctrl_node's own deployed plant model exactly
    // (AK40-10-ROS2-Bridge/config/cable_torque_ctrl_params.yaml: ude_inertia, viscous_drag —
    // note the deployed viscous_drag there is 0.00038, not the .cpp fallback of 0.00140) so the
    // simulated plant matches what that controller was actually tuned/calibrated against.
    declare_parameter("shaft_inertia_kg_m2", 0.000995);       // kg.m^2
    declare_parameter("viscous_drag_nms_per_rad", 0.00038);   // N.m.s/rad
    // Low-pass filter applied to the measured shaft speed BEFORE it's differentiated for the
    // inertia term (and before it's used in the viscous term) — see "Noise consideration" in
    // CLAUDE.md. EMA coefficient, 0-1: 1.0 = no filtering (raw, noisy), smaller = more smoothing
    // / more lag. 0.2 at the default 100 Hz poll rate gives a time constant of ~40 ms.
    declare_parameter("omega_filter_alpha", 0.2);

    const std::string isaac_prefix = get_parameter("isaac_winch_prefix").as_string();
    const double rate_hz  = get_parameter("poll_rate_hz").as_double();
    effective_radius_     = get_parameter("effective_radius").as_double();
    shaft_inertia_kg_m2_        = get_parameter("shaft_inertia_kg_m2").as_double();
    viscous_drag_nms_per_rad_   = get_parameter("viscous_drag_nms_per_rad").as_double();
    omega_filter_alpha_         = get_parameter("omega_filter_alpha").as_double();
    command_timeout_ms_   = get_parameter("command_timeout_ms").as_double();
    heartbeat_timeout_ms_ = get_parameter("heartbeat_timeout_ms").as_double();
    gravity_feedforward_n_ = get_parameter("gravity_feedforward_n").as_double();
    kp_speed_             = get_parameter("kp_speed").as_double();
    kd_speed_             = get_parameter("kd_speed").as_double();
    torque_limit_upper_   = get_parameter("torque_limit_upper").as_double();
    torque_limit_lower_   = get_parameter("torque_limit_lower").as_double();
    poll_period_s_        = 1.0 / rate_hz;

    // --- Publishers (driver-facing interface, matches ak_motor_cable_control_node exactly) ---
    joint_state_pub_    = create_publisher<sensor_msgs::msg::JointState>("~/joint_state", 10);
    motor_mode_pub_     = create_publisher<std_msgs::msg::Int8>("~/mode", 10);
    error_pub_          = create_publisher<std_msgs::msg::UInt8>("~/error_flags", 10);
    temp_pub_           = create_publisher<std_msgs::msg::Float32>("~/temperature", 10);
    control_mode_pub_   = create_publisher<std_msgs::msg::String>("~/control_mode", 10);
    enabled_pub_        = create_publisher<std_msgs::msg::Bool>("~/enabled", 10);
    node_heartbeat_pub_ = create_publisher<std_msgs::msg::Empty>("~/node_heartbeat", 10);
    ext_mode_state_pub_ = create_publisher<std_msgs::msg::String>("~/ext_mode_state", 10);
    cable_state_pub_    = create_publisher<std_msgs::msg::Float32MultiArray>("~/cable_state", 10);

    // --- Subscriptions (driver-facing interface) ---
    cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "~/command", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_command(msg); });

    mode_cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "~/mode_cmd", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_mode_cmd(msg); });

    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
        "~/heartbeat", 10,
        [this](const std_msgs::msg::Empty::SharedPtr) {
          if (!has_heartbeat_) {
            RCLCPP_INFO(get_logger(), "Primary heartbeat source connected");
          }
          last_heartbeat_time_ = now();
          has_heartbeat_ = true;
        });

    heartbeat_ext_sub_ = create_subscription<std_msgs::msg::Empty>(
        "~/heartbeat_external", 10,
        [this](const std_msgs::msg::Empty::SharedPtr) {
          if (!has_heartbeat_ext_) {
            RCLCPP_INFO(get_logger(), "External heartbeat source connected");
          }
          last_heartbeat_ext_time_ = now();
          has_heartbeat_ext_ = true;
        });

    // External torque command — only applied when ext_mode_state_ == RUNNING.
    // Clamped to [torque_limit_lower_, torque_limit_upper_] on receipt, same as real driver.
    ext_torque_cmd_sub_ = create_subscription<std_msgs::msg::Float32>(
        "~/ext_torque_cmd", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          if (ext_mode_state_ != ExternalModeState::RUNNING) { return; }
          const double raw = static_cast<double>(msg->data);
          if (raw > torque_limit_upper_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "External torque %.3f N.m exceeds upper limit %.3f N.m — clamped",
                raw, torque_limit_upper_);
          } else if (raw < torque_limit_lower_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "External torque %.3f N.m exceeds lower limit %.3f N.m — clamped",
                raw, torque_limit_lower_);
          }
          ext_torque_cmd_ = std::clamp(raw, torque_limit_lower_, torque_limit_upper_);
        });

    // True: STANDBY -> RUNNING.  False: RUNNING -> STANDBY (holds last torque at 0).
    ext_torque_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        "~/ext_torque_enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (ext_mode_state_ == ExternalModeState::STANDBY && msg->data) {
            ext_mode_state_ = ExternalModeState::RUNNING;
            RCLCPP_INFO(get_logger(), "External mode: STANDBY -> RUNNING");
          } else if (ext_mode_state_ == ExternalModeState::RUNNING && !msg->data) {
            ext_torque_cmd_        = 0.0;
            ext_mode_state_        = ExternalModeState::STANDBY;
            control_mode_          = ControlMode::SPEED;
            current_cmd_velocity_  = 0.0;
            p_des_retract_         = p_actual_retract();
            has_command_           = true;
            last_cmd_time_         = now();
            RCLCPP_INFO(get_logger(), "External mode: RUNNING -> STANDBY, holding zero velocity");
          }
        });

    // "off" turns off external mode and restores the previous control mode.
    ext_mode_cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "~/ext_mode_cmd", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          if (msg->data == "off") {
            disable_external_mode();
          } else {
            RCLCPP_WARN(get_logger(), "Unknown ext_mode_cmd '%s' (valid: off)",
                        msg->data.c_str());
          }
        });

    // --- Isaac Sim side: ROS2CableWinchBackend (cable_winch_backend_utils.py) ---
    // Sim publishes/subscribes with qos_profile_sensor_data (BEST_EFFORT) — match it here.
    state_cable_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        isaac_prefix + "state/cable", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_state_cable(msg); });

    command_force_pub_ = create_publisher<std_msgs::msg::Float32>(
        isaac_prefix + "command/force", rclcpp::SensorDataQoS());

    // --- Services ---
    enable_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/enable",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          enabled_ = true;
          // Re-sync the SPEED setpoint so enabling never produces a snap force from wherever
          // the cable drifted to while disabled.
          p_des_retract_ = p_actual_retract();
          resp->success = true;
          resp->message = "Motor enabled";
          RCLCPP_INFO(get_logger(), "Motor enabled");
        });

    disable_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/disable",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          enabled_ = false;
          resp->success = true;
          resp->message = "Motor disabled";
          RCLCPP_INFO(get_logger(), "Motor disabled");
        });

    zero_pos_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/zero_position",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          if (enabled_) {
            resp->success = false;
            resp->message = "Disable motor before zeroing position";
            return;
          }
          extension_offset_ = last_extension_;
          resp->success = true;
          resp->message = "Zero position set";
          RCLCPP_INFO(get_logger(), "Zero position set");
        });

    // Entering external mode saves the current control mode and zeros the cable-length
    // reference (mirrors the real driver's encoder-zero-on-entry, so subsequent position
    // feedback is usable for cable length tracking from this point).
    enable_ext_mode_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/enable_external_mode",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          if (ext_mode_state_ != ExternalModeState::OFF) {
            resp->success = false;
            resp->message = "External mode already active (standby or running)";
            return;
          }
          prev_control_mode_ = control_mode_;
          extension_offset_  = last_extension_;
          ext_torque_cmd_    = 0.0;
          ext_mode_state_    = ExternalModeState::STANDBY;
          resp->success = true;
          resp->message = "External mode enabled (STANDBY), cable length zeroed";
          RCLCPP_INFO(get_logger(),
                      "External mode enabled — cable length zeroed, awaiting ext_torque_enable");
        });

    using namespace std::chrono_literals;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    poll_timer_ = create_wall_timer(period, [this]() { poll_tick(); });

    RCLCPP_INFO(get_logger(),
                "Cable control emulator ready (Isaac Sim prefix '%s', effective_radius=%.4f m, "
                "gravity_feedforward_n=%.2f N, kp_speed=%.1f N/m, kd_speed=%.1f N.s/m, "
                "shaft_inertia=%.6f kg.m^2, viscous_drag=%.5f N.m.s/rad [EXTERNAL mode only]), "
                "default mode: SPEED",
                isaac_prefix.c_str(), effective_radius_, gravity_feedforward_n_, kp_speed_,
                kd_speed_, shaft_inertia_kg_m2_, viscous_drag_nms_per_rad_);
  }

 private:
  void on_command(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // External torque is driving — ignore normal commands while RUNNING.
    if (ext_mode_state_ == ExternalModeState::RUNNING) { return; }

    const double pos = msg->position.empty() ? 0.0 : msg->position[0];
    const double vel = msg->velocity.empty() ? 0.0 : msg->velocity[0];
    const double eff = msg->effort.empty()   ? 0.0 : msg->effort[0];

    switch (control_mode_) {
      case ControlMode::SPEED:
        if (pos != 0.0 || eff != 0.0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "Mode mismatch: SPEED mode only uses velocity field "
              "(got position=%.3f effort=%.3f — ignoring)", pos, eff);
        }
        // m/s, positive = retract (emulator-specific convention — see file header; NOT the
        // real hardware's raw-shaft rad/s convention).
        current_cmd_velocity_ = vel;
        break;

      case ControlMode::TORQUE:
        if (pos != 0.0 || vel != 0.0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "Mode mismatch: TORQUE mode only uses effort field "
              "(got position=%.3f velocity=%.3f — ignoring)", pos, vel);
        }
        // Accepted for interface parity; not actuated this pass (see file header).
        current_cmd_velocity_ = 0.0;
        break;

      case ControlMode::POSITION:
        if (eff != 0.0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "Mode mismatch: POSITION mode only uses position/velocity fields "
              "(got effort=%.3f — ignoring)", eff);
        }
        // Accepted for interface parity; not actuated this pass (see file header).
        current_cmd_velocity_ = 0.0;
        break;
    }
    last_cmd_time_ = now();
    has_command_   = true;
  }

  void on_mode_cmd(const std_msgs::msg::String::SharedPtr msg) {
    const std::string& m = msg->data;
    if (m == "speed") {
      control_mode_ = ControlMode::SPEED;
      gravity_feedforward_n_ = get_parameter("gravity_feedforward_n").as_double();
      kp_speed_ = get_parameter("kp_speed").as_double();
      kd_speed_ = get_parameter("kd_speed").as_double();
      // Re-sync the integrated position setpoint to the current actual position so switching
      // into SPEED mode never produces a snap force from stale/zero setpoint history.
      p_des_retract_ = p_actual_retract();
      RCLCPP_INFO(get_logger(), "Control mode -> SPEED (kp=%.3f kd=%.3f)", kp_speed_, kd_speed_);
    } else if (m == "torque") {
      control_mode_ = ControlMode::TORQUE;
      RCLCPP_INFO(get_logger(), "Control mode -> TORQUE (not actuated by this emulator)");
    } else if (m == "pos") {
      control_mode_ = ControlMode::POSITION;
      RCLCPP_INFO(get_logger(), "Control mode -> POSITION (not actuated by this emulator)");
    } else {
      RCLCPP_WARN(get_logger(), "Unknown mode '%s' (valid: speed, torque, pos)", m.c_str());
    }
  }

  // Called whenever Isaac Sim publishes new cable state — mirrors the real driver publishing
  // ~/joint_state and ~/cable_state once per decoded CAN feedback frame.
  void on_state_cable(const sensor_msgs::msg::JointState::SharedPtr msg) {
    last_extension_     = msg->position.empty() ? last_extension_     : msg->position[0];
    last_extension_vel_ = msg->velocity.empty() ? last_extension_vel_ : msg->velocity[0];

    // Shaft speed/acceleration (EXTERNAL mode only — see "Shaft dynamics" in CLAUDE.md), updated
    // HERE rather than in the poll_tick()-driven compute_and_publish_force(), because this is
    // where a genuinely fresh sample actually arrives. Isaac Sim publishes state/cable on its own
    // physics-step cadence, not necessarily locked to this node's 100 Hz poll timer — computing
    // the derivative against an assumed-fixed poll period diverged badly the moment EXTERNAL
    // RUNNING engaged (first real test, 2026-07-11): whenever the real inter-sample time didn't
    // match the assumed period, the finite difference was silently scaled by however wrong that
    // assumption was, and any msg backlog/burst amplified it further. Using the actual measured
    // wall-clock dt between real samples (plus a low-pass filter on omega itself, per the "Noise
    // consideration" note in CLAUDE.md) fixes both the timing mismatch and plain sensor noise.
    const double omega_new_retract = -last_extension_vel_ / effective_radius_;
    const double omega_prev_retract = omega_filtered_retract_;
    omega_filtered_retract_ = omega_filter_alpha_ * omega_new_retract
                             + (1.0 - omega_filter_alpha_) * omega_prev_retract;
    const rclcpp::Time t_now = now();
    if (has_state_) {
      const double dt = (t_now - last_state_time_).seconds();
      if (dt > 1e-4) {  // guard against duplicate/near-simultaneous timestamps
        omega_dot_retract_ = (omega_filtered_retract_ - omega_prev_retract) / dt;
      }
    }
    last_state_time_ = t_now;
    has_state_ = true;

    const double cable_length = last_extension_ - extension_offset_;

    // Filtered velocity for anything PUBLISHED to downstream consumers (cable_torque_ctrl_node,
    // the GUI) — reuses omega_filtered_retract_ (already computed above) rather than a second,
    // independent filter, so there's one source of truth for "the filtered cable speed." This
    // matters a lot more than it might look: cable_torque_ctrl_node's sliding-mode/L1-adaptive
    // law is specifically DESIGNED to react sharply to velocity/tracking-error, tuned against
    // real hardware's clean encoder feedback over CAN. Feeding it Isaac Sim's raw (noisier)
    // rigid-body velocity caused it to chatter — visible as growing oscillation and repeated
    // torque-limit spikes even with cable_torque_ctrl_node just holding position (no external
    // reference), first observed 2026-07-11. Position is left unfiltered (real encoders are
    // typically far cleaner on position than on differentiated velocity, so this mirrors what a
    // real sensor chain would actually look like, rather than over-filtering everything blindly).
    const double filtered_extension_vel = -omega_filtered_retract_ * effective_radius_;

    // ~/cable_state: length decreases / velocity negative on retract — same sign convention
    // as the sim's extension/extension_vel (both extend-positive), so no flip needed here.
    std_msgs::msg::Float32MultiArray cable_state_msg;
    cable_state_msg.data = {static_cast<float>(cable_length),
                             static_cast<float>(filtered_extension_vel)};
    cable_state_pub_->publish(cable_state_msg);

    // ~/joint_state: synthesized raw-shaft rad/rad-s, positive = retract (opposite sign from
    // the sim's extend-positive convention — see sign-convention table in the design plan).
    // velocity here is exactly omega_filtered_retract_ already (no conversion needed - it's
    // already retract-positive rad/s).
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name.push_back("ak40_10_sim");
    js.position.push_back(-cable_length / effective_radius_);
    js.velocity.push_back(omega_filtered_retract_);
    js.effort.push_back(last_force_retract_ * effective_radius_);
    joint_state_pub_->publish(js);
  }

  // Returns true if at least one heartbeat source is still alive.
  // If both sources go stale, disables the motor (once) and warns.
  void check_heartbeat() {
    if (!has_heartbeat_ && !has_heartbeat_ext_) {
      return;  // No heartbeat ever received — bench mode, watchdog inactive.
    }

    const bool fresh_main = has_heartbeat_ &&
        (now() - last_heartbeat_time_).nanoseconds() / 1e6 <= heartbeat_timeout_ms_;
    const bool fresh_ext = has_heartbeat_ext_ &&
        (now() - last_heartbeat_ext_time_).nanoseconds() / 1e6 <= heartbeat_timeout_ms_;

    if (!fresh_main && !fresh_ext) {
      if (!heartbeat_lost_) {
        heartbeat_lost_ = true;
        RCLCPP_WARN(get_logger(), "All heartbeat sources lost — disabling motor");
        enabled_ = false;
        if (ext_mode_state_ != ExternalModeState::OFF) {
          ext_mode_state_        = ExternalModeState::OFF;
          ext_torque_cmd_        = 0.0;
          control_mode_          = ControlMode::SPEED;
          current_cmd_velocity_  = 0.0;
          p_des_retract_         = p_actual_retract();
          has_command_           = true;
          last_cmd_time_         = now();
          RCLCPP_WARN(get_logger(),
              "Heartbeat lost — external mode cleared, fallback to zero-velocity SPEED");
        }
      }
    } else {
      if (heartbeat_lost_) {
        heartbeat_lost_ = false;
        RCLCPP_INFO(get_logger(), "Heartbeat regained (re-enable motor manually)");
      }
    }
  }

  void disable_external_mode() {
    if (ext_mode_state_ == ExternalModeState::OFF) { return; }
    ext_mode_state_ = ExternalModeState::OFF;
    ext_torque_cmd_ = 0.0;
    control_mode_   = prev_control_mode_;
    RCLCPP_INFO(get_logger(), "External mode disabled, control mode restored");
  }

  // Current cable position in the same retract-positive frame as v_actual_retract, zeroed by
  // extension_offset_ (zero_position / enable_external_mode). Extend-positive sim `extension`
  // flips sign here, same as the joint_state synthesis in on_state_cable().
  double p_actual_retract() const {
    return -(last_extension_ - extension_offset_);
  }

  const char* ext_mode_state_str() const {
    switch (ext_mode_state_) {
      case ExternalModeState::OFF:     return "off";
      case ExternalModeState::STANDBY: return "standby";
      case ExternalModeState::RUNNING: return "running";
    }
    return "off";
  }

  // Computes the force to send to Isaac Sim and publishes it. Working "frame" is
  // retract-positive throughout (the sim's command/force and the real hardware's
  // ext_torque_cmd already share that sign; only the SPEED-mode velocity comparison needs
  // an explicit flip because the sim's extension_vel is extend-positive).
  //
  // SPEED mode runs PD + gravity feedforward against a position setpoint that is integrated
  // from the commanded velocity each tick (p_des_retract_ += v_des_retract * dt) whenever
  // there's a fresh SPEED command to actively track. Whenever there ISN'T — command gone
  // stale (e.g. the GUI's Stop button publishes one zero-velocity message and then stops
  // republishing, so the "fresh" window is only command_timeout_ms long), no command received
  // yet, or POSITION/TORQUE mode (not actuated) — the setpoint freezes at the current actual
  // position and the SAME PD+feedforward law holds it there. Unlike the real driver's
  // kd_watchdog fallback (a weak pure damper, safe on real hardware because the winch's own
  // mechanical friction/self-locking keeps it from free-falling), this sim's cable is
  // frictionless: without feedforward, the fallback branch would let the payload's ~5.54 N
  // weight free-fall to the ground. So there is deliberately no separate weak "watchdog" law
  // here — going stale means "hold position," not "go passive."
  // Shaft dynamics (EXTERNAL mode only): the motor shaft and the cable share one rigid DOF (the
  // drum has no slip), so shaft angular velocity is just the measured cable velocity converted
  // through effective_radius_ — no separate integrated shaft state is needed, and it can't drift
  // out of sync with what Isaac Sim is actually doing. omega_filtered_retract_/omega_dot_retract_
  // are computed in on_state_cable() (using the ACTUAL elapsed time between real Isaac Sim
  // samples, not this function's poll period — see that function's comment for why that
  // distinction matters) and just read here. The torque the shaft's own inertia and viscous drag
  // "absorb" (J*omega_dot + b*omega) is subtracted from ext_torque_cmd_ before converting the
  // remainder to a force — i.e. some of the commanded torque goes into accelerating/dragging the
  // (fictitious, not present in Isaac Sim's own rigid-body masses) shaft itself, only the rest
  // reaches the cable.
  void compute_and_publish_force() {
    const double v_actual_retract = -last_extension_vel_;
    const double p_actual = p_actual_retract();
    double force_retract = 0.0;

    if (!enabled_) {
      force_retract = 0.0;  // freewheel — stands in for the real exit_mit_mode CAN frame
      p_des_retract_ = p_actual;
    } else if (ext_mode_state_ == ExternalModeState::RUNNING) {
      const double tau_shaft_absorbed = shaft_inertia_kg_m2_ * omega_dot_retract_
                                       + viscous_drag_nms_per_rad_ * omega_filtered_retract_;
      const double tau_to_cable = ext_torque_cmd_ - tau_shaft_absorbed;
      force_retract = tau_to_cable / effective_radius_;
      p_des_retract_ = p_actual;
      RCLCPP_DEBUG(get_logger(),
          "shaft: omega=%.4f omega_dot=%.4f tau_absorbed=%.5f ext_torque_cmd=%.5f "
          "tau_to_cable=%.5f force=%.4f",
          omega_filtered_retract_, omega_dot_retract_, tau_shaft_absorbed, ext_torque_cmd_,
          tau_to_cable, force_retract);
    } else {
      const double elapsed_ms = (now() - last_cmd_time_).nanoseconds() / 1e6;
      const bool command_fresh = has_command_ && elapsed_ms <= command_timeout_ms_;
      const bool actively_tracking = command_fresh && control_mode_ == ControlMode::SPEED;

      double v_des_retract = 0.0;
      if (actively_tracking) {
        v_des_retract = current_cmd_velocity_;  // m/s, already retract-positive
        p_des_retract_ += v_des_retract * poll_period_s_;
      } else {
        p_des_retract_ = p_actual;  // hold wherever we are — see function comment
        if (has_command_ && control_mode_ == ControlMode::SPEED && !command_fresh) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                               "Command timeout (%.0f ms), holding position", elapsed_ms);
        }
      }

      const double e_p = p_des_retract_ - p_actual;
      const double e_v = v_des_retract - v_actual_retract;
      force_retract = gravity_feedforward_n_ + kp_speed_ * e_p + kd_speed_ * e_v;
    }

    last_force_retract_ = force_retract;
    std_msgs::msg::Float32 force_msg;
    force_msg.data = static_cast<float>(force_retract);
    command_force_pub_->publish(force_msg);
  }

  void poll_tick() {
    check_heartbeat();

    compute_and_publish_force();

    std_msgs::msg::String mode_str;
    if (ext_mode_state_ != ExternalModeState::OFF) {
      mode_str.data = "external";
    } else {
      switch (control_mode_) {
        case ControlMode::SPEED:    mode_str.data = "speed";  break;
        case ControlMode::TORQUE:   mode_str.data = "torque"; break;
        case ControlMode::POSITION: mode_str.data = "pos";    break;
      }
    }
    control_mode_pub_->publish(mode_str);

    std_msgs::msg::String ext_state_msg;
    ext_state_msg.data = ext_mode_state_str();
    ext_mode_state_pub_->publish(ext_state_msg);

    std_msgs::msg::Bool enabled_msg;
    enabled_msg.data = enabled_;
    enabled_pub_->publish(enabled_msg);

    node_heartbeat_pub_->publish(std_msgs::msg::Empty{});

    // No physical motor to report on — constant stubs, published for interface parity only.
    std_msgs::msg::Int8 motor_mode_msg;
    motor_mode_msg.data = 0;
    motor_mode_pub_->publish(motor_mode_msg);

    std_msgs::msg::UInt8 err_msg;
    err_msg.data = 0;
    error_pub_->publish(err_msg);

    std_msgs::msg::Float32 temp_msg;
    temp_msg.data = 25.0f;
    temp_pub_->publish(temp_msg);
  }

  // --- Isaac Sim I/O ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_cable_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           command_force_pub_;

  // --- Publishers (driver-facing interface) ---
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr          motor_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr         error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr       temp_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        control_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr         node_heartbeat_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        ext_mode_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cable_state_pub_;

  // --- Subscriptions (driver-facing interface) ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        mode_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr         heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr         heartbeat_ext_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr       ext_torque_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          ext_torque_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        ext_mode_cmd_sub_;

  // --- Services ---
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_pos_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_ext_mode_srv_;

  rclcpp::TimerBase::SharedPtr poll_timer_;

  ControlMode  control_mode_{ControlMode::SPEED};
  ControlMode  prev_control_mode_{ControlMode::SPEED};
  double       current_cmd_velocity_{0.0};  // m/s, positive = retract
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_heartbeat_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_heartbeat_ext_time_{0, 0, RCL_ROS_TIME};
  bool         has_command_{false};
  bool         has_heartbeat_{false};
  bool         has_heartbeat_ext_{false};
  bool         enabled_{false};
  bool         heartbeat_lost_{false};
  double       command_timeout_ms_{500.0};
  double       heartbeat_timeout_ms_{1500.0};
  double       gravity_feedforward_n_{5.54};  // N — cancels constant gravity load in SPEED mode
  double       kp_speed_{10.0};              // N/m — SPEED-mode PD position gain
  double       kd_speed_{4.0};               // N.s/m — SPEED-mode PD velocity gain
  double       p_des_retract_{0.0};         // m — SPEED-mode integrated position setpoint
  double       poll_period_s_{0.01};        // s — fixed dt for the setpoint integrator

  // External mode
  ExternalModeState ext_mode_state_{ExternalModeState::OFF};
  double            ext_torque_cmd_{0.0};
  double            torque_limit_upper_{1.5};    // N.m
  double            torque_limit_lower_{-1.5};   // N.m

  // External-mode shaft dynamics (see "Shaft dynamics" note in compute_and_publish_force() and
  // on_state_cable()). omega_filtered_retract_/omega_dot_retract_ are computed in
  // on_state_cable() — using the actual measured wall-clock time between real Isaac Sim samples
  // (last_state_time_), not this node's poll period — and just read in compute_and_publish_force().
  double shaft_inertia_kg_m2_{0.000995};        // kg.m^2 — matches cable_torque_ctrl_node's ude_inertia
  double viscous_drag_nms_per_rad_{0.00038};    // N.m.s/rad — matches its deployed viscous_drag
  double omega_filter_alpha_{0.2};              // EMA coefficient applied to shaft speed before differentiating
  double omega_filtered_retract_{0.0};          // rad/s — filtered shaft speed
  double omega_dot_retract_{0.0};               // rad/s^2 — filtered shaft acceleration
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};  // timestamp of the last on_state_cable() call

  // Isaac Sim state (raw, extend-positive — as received from ROS2CableWinchBackend)
  double effective_radius_{0.036};    // m — torque(N.m)/force(N), rad-s/m-s scale factor
  double last_extension_{0.0};        // m
  double last_extension_vel_{0.0};    // m/s
  double extension_offset_{0.0};      // m — zero reference (zero_position / enable_external_mode)
  double last_force_retract_{0.0};    // N — for ~/joint_state effort field
  bool   has_state_{false};
};

}  // namespace ak_motor_cable_control_emulator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak_motor_cable_control_emulator::AkMotorCableControlNode>());
  rclcpp::shutdown();
  return 0;
}
