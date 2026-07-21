// MIT License
// Copyright (c) 2026 FSC Lab

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ak_motor_cable_control_emulator {

static constexpr double kGravity = 9.81;  // m/s^2

// Cable torque controller — PD + Friction feedforward + L1 adaptive control + UDE monitor.
//
// A/B comparison against AK40-10-ROS2-Bridge's cable_torque_ctrl_node (BLSMC+L1): same node
// role/interface (drop-in replacement, same ~/cable_state, ~/reference, ~/ext_torque_cmd,
// ~/ext_torque_enable, ~/arm, ~/disarm), same L1 adaptive core UNCHANGED (the L1
// predictor/adaptation only look at tau_applied_prev_/tau_f_prev_ and the resulting velocity
// error, so they are entirely agnostic to whether PD or SMC produced that prior torque) — the
// ONLY thing replaced is the discontinuous SMC switching term (tau_sw =
// -r*M_eff_hat*eta*sat(s/phi)) with a smooth PD feedback term on position/velocity error.
// Purpose: isolate whether the chatter observed with the real BLSMC controller against this
// sim is caused by the SMC switching term specifically (see fsc_PegasusSimulator's CLAUDE.md,
// "why does the SMC controller chatter in sim" investigation) — if this PD+L1 variant
// holds/tracks cleanly where BLSMC+L1 chatters, that confirms the SMC switching term (not sim
// timing, not the L1/plant model) as the dominant cause.
//
// Friction feedforward is viscous-only here, UNLIKE cable_torque_ctrl_node (which also has a
// Coulomb term, calibrated against real hardware's actual static friction). This sim's shaft-
// dynamics model has no Coulomb/static friction at all (see the emulator's own CLAUDE.md,
// "Shaft dynamics") — carrying the real Coulomb value over here first was tried and found to
// inject a sustained ~23cm limit-cycle oscillation while just holding position with no
// reference: tanh(omega/deadband) saturates to essentially sign(velocity) at these speeds, a
// relay force always in phase with velocity that does net positive work every cycle since
// there's no matching real friction for it to cancel — a textbook friction-overcompensation
// limit cycle. Removed entirely (not just zeroed) since there's no physical friction here for
// it to ever compensate.
//
// Control law (retract-positive convention, M_eff = J/r^2 + m, L1-adapted r*M_eff_hat/m_hat*g*r):
//   a_dyn  = acc_ref - g
//   tau_ff = r*M_eff_hat*a_dyn + m_hat*g*r + tau_f          (feedforward: dynamics + gravity + friction)
//   tau_pd = r*M_eff_hat*(-kp_pd*e_p - kd_pd*e_v)           (smooth PD feedback, replaces tau_sw)
//   tau    = sat(tau_ff + tau_pd, sat_lower, sat_upper)
//
// L1 adaptive law and UDE monitor: copied verbatim from cable_torque_ctrl_node (see that file's
// header comment for the full derivation) - unchanged by the PD/SMC swap above.
//
// Reference topic ~/reference (Float64MultiArray, 3 elements): same as cable_torque_ctrl_node -
//   [0] acc_ref (m/s^2), [1] v_c_star (m/s), [2] p_c_star (m), cable_state convention.
// Default when no reference received: acc_ref=g, v_c_star=0, e_p=p_c-hold_pos_star.
//
// Debug topic ~/debug (Float64MultiArray, 12 elements):
//   [0]  tau          (N.m)   total post-saturation command
//   [1]  e_v          (m/s)   cable velocity error
//   [2]  e_p          (m)     cable position error
//   [3]  v_c          (m/s)   actual cable velocity
//   [4]  p_c          (m)     actual cable position
//   [5]  acc_ref      (m/s^2) active reference acceleration
//   [6]  tau_d_hat    (N.m)   UDE estimate (monitor only - NOT in control)
//   [7]  tau_ff       (N.m)   feedforward component (gravity+inertia+friction, L1 adapted)
//   [8]  tau_pd       (N.m)   PD feedback component (replaces SMC's tau_sw)
//   [9]  tau_f        (N.m)   friction feedforward component inside tau_ff
//   [10] theta_f_1            L1 filtered theta_1 ~= 1/(r*M_eff_hat)
//   [11] theta_f_2   (m/s^2)  L1 filtered theta_2 ~= -mg/M_eff_hat

class CableTorqueCtrlPdL1Node : public rclcpp::Node {
 public:
  CableTorqueCtrlPdL1Node() : Node("cable_torque_ctrl_pd_l1_node") {
    // Plant
    declare_parameter("drum_radius",   0.036);
    declare_parameter("mass",          0.565);
    // PD (replaces BLSMC's kp_c/smc_eta/smc_phi)
    declare_parameter("kp_pd",         10.0);      // N.m per m of r*M_eff_hat-scaled position error
    declare_parameter("kd_pd",          4.0);      // N.m per m/s of r*M_eff_hat-scaled velocity error
    declare_parameter("sat_upper",      1.5);      // N.m
    declare_parameter("sat_lower",     -1.5);      // N.m
    declare_parameter("motor_direction", 1);       // 1 or -1
    declare_parameter("poll_rate_hz",        100.0);
    declare_parameter("reference_timeout_ms", 500.0);
    // Friction feedforward — viscous only. No Coulomb term: unlike cable_torque_ctrl_node
    // (calibrated against real hardware, which does have static/Coulomb friction), this sim's
    // shaft-dynamics model deliberately has no Coulomb/static friction term (see this package's
    // CLAUDE.md, "Shaft dynamics"). Carrying the real Coulomb feedforward value over here was
    // tried first and found to inject a sustained ~23cm limit-cycle oscillation while holding
    // position with no reference: coulomb_friction*tanh(omega/deadband) saturates to essentially
    // sign(velocity) at these speeds, i.e. a relay force always in phase with velocity, doing net
    // positive work every cycle since there is no matching real friction for it to cancel — kd_pd
    // alone couldn't dissipate that continuous energy injection. Removed entirely rather than
    // just zeroed, since there's no physical friction here for it to ever compensate.
    declare_parameter("viscous_drag", 0.00038);
    // L1 adaptive (same as cable_torque_ctrl_node's deployed values)
    declare_parameter("l1_as",          50.0);
    declare_parameter("l1_gamma_1",      5.0);
    declare_parameter("l1_gamma_2",      5.0);
    declare_parameter("l1_omega_f",      5.0);
    declare_parameter("l1_theta_1_min",  2.0);
    // UDE (monitor)
    declare_parameter("ude_lambda",         10.0);
    declare_parameter("ude_inertia",         0.000995);
    declare_parameter("ude_integral_limit",  0.06);

    drum_radius_                = get_parameter("drum_radius").as_double();
    mass_                       = get_parameter("mass").as_double();
    kp_pd_                      = get_parameter("kp_pd").as_double();
    kd_pd_                      = get_parameter("kd_pd").as_double();
    sat_upper_                  = get_parameter("sat_upper").as_double();
    sat_lower_                  = get_parameter("sat_lower").as_double();
    motor_direction_            = get_parameter("motor_direction").as_int();
    ref_timeout_ms_             = get_parameter("reference_timeout_ms").as_double();
    viscous_drag_               = get_parameter("viscous_drag").as_double();
    l1_as_                      = get_parameter("l1_as").as_double();
    l1_gamma_1_                 = get_parameter("l1_gamma_1").as_double();
    l1_gamma_2_                 = get_parameter("l1_gamma_2").as_double();
    l1_omega_f_                 = get_parameter("l1_omega_f").as_double();
    l1_theta_1_min_             = get_parameter("l1_theta_1_min").as_double();
    ude_lambda_                 = get_parameter("ude_lambda").as_double();
    ude_inertia_                = get_parameter("ude_inertia").as_double();
    ude_integral_limit_         = get_parameter("ude_integral_limit").as_double();
    const double rate_hz = get_parameter("poll_rate_hz").as_double();
    dt_ = 1.0 / rate_hz;

    if (motor_direction_ != 1 && motor_direction_ != -1) {
      RCLCPP_WARN(get_logger(), "motor_direction must be 1 or -1, got %d — defaulting to 1",
                  motor_direction_);
      motor_direction_ = 1;
    }

    const double M_eff_nom = ude_inertia_ / (drum_radius_ * drum_radius_) + mass_;
    l1_theta_1_nom_        = 1.0 / (drum_radius_ * M_eff_nom);
    l1_theta_2_nom_        = -mass_ * kGravity / M_eff_nom;
    reset_l1();

    ext_torque_cmd_pub_    = create_publisher<std_msgs::msg::Float32>("~/ext_torque_cmd", 10);
    ext_torque_enable_pub_ = create_publisher<std_msgs::msg::Bool>("~/ext_torque_enable", 10);
    debug_pub_             = create_publisher<std_msgs::msg::Float64MultiArray>("~/debug", 10);
    ude_pub_               = create_publisher<std_msgs::msg::Float64>("~/ude_disturbance", 10);

    cable_state_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "~/cable_state", 10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
          if (msg->data.size() < 2) { return; }
          p_c_ = -motor_direction_ * static_cast<double>(msg->data[0]);
          v_c_ = -motor_direction_ * static_cast<double>(msg->data[1]);
        });

    reference_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/reference", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          if (msg->data.size() < 3) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Reference must have 3 elements [acc_ref, v_c_star, p_c_star], got %zu",
                msg->data.size());
            return;
          }
          acc_ref_       = msg->data[0];
          v_c_star_      = -motor_direction_ * msg->data[1];
          p_c_star_      = -motor_direction_ * msg->data[2];
          has_ref_       = true;
          last_ref_time_ = now();
        });

    arm_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/arm",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          if (armed_) {
            resp->success = false;
            resp->message = "Already armed";
            return;
          }
          hold_pos_star_ = p_c_;
          reset_l1();
          l1_v_hat_ = v_c_;
          std_msgs::msg::Bool en_msg;
          en_msg.data = true;
          ext_torque_enable_pub_->publish(en_msg);
          armed_ = true;
          RCLCPP_INFO(get_logger(),
                      "Armed — holding position %.4f m, L1 reset (theta1=%.3f, theta2=%.3f)",
                      hold_pos_star_, l1_theta_f_1_, l1_theta_f_2_);
          resp->success = true;
          resp->message = "Armed";
        });

    disarm_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/disarm",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          do_disarm();
          resp->success = true;
          resp->message = "Disarmed";
        });

    using namespace std::chrono_literals;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    poll_timer_ = create_wall_timer(period, [this]() { poll(); });

    RCLCPP_INFO(get_logger(),
                "Cable torque ctrl (PD+L1, chatter A/B variant) ready\n"
                "  Plant: mass=%.3f kg, r=%.4f m, J=%.4f kg·m², M_eff=%.3f kg\n"
                "  PD: kp_pd=%.2f N.m/m, kd_pd=%.2f N.m.s/m\n"
                "  Friction: Fv=%.5f N.m.s/rad (viscous only, no Coulomb term)\n"
                "  L1: a_s=%.1f, gamma=[%.2f,%.2f], omega_f=%.2f rad/s, "
                "theta1_nom=%.3f, theta1_min=%.2f\n"
                "  UDE: lambda=%.1f rad/s, limit=%.3f N.m",
                mass_, drum_radius_, ude_inertia_, M_eff_nom,
                kp_pd_, kd_pd_,
                viscous_drag_,
                l1_as_, l1_gamma_1_, l1_gamma_2_, l1_omega_f_,
                l1_theta_1_nom_, l1_theta_1_min_,
                ude_lambda_, ude_integral_limit_);
  }

 private:
  void do_disarm() {
    armed_ = false;
    reset_ude();
    reset_l1();
    std_msgs::msg::Bool en_msg;
    en_msg.data = false;
    ext_torque_enable_pub_->publish(en_msg);
    RCLCPP_INFO(get_logger(), "Disarmed — UDE and L1 reset");
  }

  void reset_ude() {
    ude_integral_term_ = 0.0;
    ude_tau_d_hat_     = 0.0;
    tau_applied_prev_  = 0.0;
    tau_f_prev_        = 0.0;
  }

  void reset_l1() {
    l1_v_hat_       = 0.0;
    l1_theta_hat_1_ = l1_theta_1_nom_;
    l1_theta_hat_2_ = l1_theta_2_nom_;
    l1_theta_f_1_   = l1_theta_1_nom_;
    l1_theta_f_2_   = l1_theta_2_nom_;
  }

  void poll() {
    if (!armed_) { return; }

    // Default: gravity hold at arm position. Identical to cable_torque_ctrl_node.
    double acc_ref  = kGravity;
    double v_c_star = 0.0;
    double e_p      = p_c_ - hold_pos_star_;

    if (has_ref_) {
      const double elapsed_ms = (now() - last_ref_time_).nanoseconds() / 1e6;
      if (elapsed_ms <= ref_timeout_ms_) {
        acc_ref  = acc_ref_;
        v_c_star = v_c_star_;
        e_p      = p_c_ - p_c_star_;
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Reference timeout (%.0f ms) — gravity hold active", elapsed_ms);
      }
    }

    const double e_v   = v_c_ - v_c_star;
    const double omega = v_c_ / drum_radius_;
    const double a_dyn = acc_ref - kGravity;

    // ── Friction feedforward — viscous only (see the no-Coulomb-term note above) ────────
    const double tau_f = viscous_drag_ * omega;

    // ── L1 adaptive — identical to cable_torque_ctrl_node. Note this only depends on
    // tau_applied_prev_/tau_f_prev_ and the velocity-error epsilon, never on how that prior
    // torque was computed - so it works unchanged whether PD or SMC produced it. ────────
    const double tau_minus_f_prev = tau_applied_prev_ - tau_f_prev_;
    const double epsilon          = l1_v_hat_ - v_c_;
    l1_v_hat_ += dt_ * (-l1_as_ * epsilon
                        + l1_theta_hat_1_ * tau_minus_f_prev
                        + l1_theta_hat_2_);

    l1_theta_hat_1_ -= dt_ * l1_gamma_1_ * tau_minus_f_prev * epsilon;
    l1_theta_hat_1_  = std::max(l1_theta_hat_1_, l1_theta_1_min_);
    l1_theta_hat_2_ -= dt_ * l1_gamma_2_ * epsilon;

    const double alpha = 1.0 - std::exp(-l1_omega_f_ * dt_);
    l1_theta_f_1_ += alpha * (l1_theta_hat_1_ - l1_theta_f_1_);
    l1_theta_f_2_ += alpha * (l1_theta_hat_2_ - l1_theta_f_2_);

    const double r_M_eff_hat = 1.0 / l1_theta_f_1_;
    const double m_g_r_hat   = -l1_theta_f_2_ * r_M_eff_hat;

    // ── Control: feedforward (dynamics+gravity+friction) + smooth PD feedback ──────────
    // Replaces BLSMC's sliding-surface reaching law + discontinuous switching term with a
    // plain PD law on position/velocity error, scaled by the same L1-adapted r*M_eff_hat so
    // the closed-loop bandwidth stays consistent as the plant estimate adapts.
    const double tau_ff = r_M_eff_hat * a_dyn + m_g_r_hat + tau_f;
    const double tau_pd = r_M_eff_hat * (-kp_pd_ * e_p - kd_pd_ * e_v);

    // ── UDE — monitor only, identical to cable_torque_ctrl_node ────────────
    const double tau_p         = -mass_ * drum_radius_ * kGravity;
    const double ude_integrand = ude_tau_d_hat_ + tau_p + tau_applied_prev_;
    const double new_integral  = ude_integral_term_ + ude_integrand * dt_;
    if (std::abs(ude_lambda_ * new_integral) <= ude_integral_limit_) {
      ude_integral_term_ = new_integral;
    }
    ude_tau_d_hat_ = ude_lambda_ * ude_inertia_ * omega - ude_lambda_ * ude_integral_term_;

    // ── Final command ──────────────────────────────────────────────────────
    const double tau_raw = tau_ff + tau_pd;
    const double tau     = std::clamp(tau_raw, sat_lower_, sat_upper_);

    if (tau_raw > sat_upper_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Torque saturated upper: raw=%.3f N.m", tau_raw);
    } else if (tau_raw < sat_lower_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Torque saturated lower: raw=%.3f N.m", tau_raw);
    }

    tau_applied_prev_ = tau;
    tau_f_prev_       = tau_f;

    std_msgs::msg::Float32 torque_msg;
    torque_msg.data = static_cast<float>(motor_direction_ * tau);
    ext_torque_cmd_pub_->publish(torque_msg);

    std_msgs::msg::Float64 ude_msg;
    ude_msg.data = ude_tau_d_hat_;
    ude_pub_->publish(ude_msg);

    // debug: [tau, e_v, e_p, v_c, p_c, acc_ref, tau_d_hat, tau_ff, tau_pd, tau_f, theta_f_1, theta_f_2]
    std_msgs::msg::Float64MultiArray dbg;
    dbg.data = {tau,  e_v,  e_p,  v_c_,  p_c_,  acc_ref,  ude_tau_d_hat_,
                tau_ff,  tau_pd,  tau_f,  l1_theta_f_1_,  l1_theta_f_2_};
    debug_pub_->publish(dbg);
  }

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           ext_torque_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              ext_torque_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           ude_pub_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cable_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr reference_sub_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_srv_;

  rclcpp::TimerBase::SharedPtr poll_timer_;

  // Live state
  double p_c_{0.0};
  double v_c_{0.0};

  // Reference
  double       acc_ref_{kGravity};
  double       v_c_star_{0.0};
  double       p_c_star_{0.0};
  bool         has_ref_{false};
  rclcpp::Time last_ref_time_{0, 0, RCL_ROS_TIME};

  // Control state
  bool   armed_{false};
  double hold_pos_star_{0.0};

  // Plant parameters
  double drum_radius_{0.036};
  double mass_{0.565};
  // PD parameters (replace BLSMC's kp_c/smc_eta/smc_phi)
  double kp_pd_{10.0};
  double kd_pd_{4.0};
  double sat_upper_{1.5};
  double sat_lower_{-1.5};
  int    motor_direction_{1};
  double ref_timeout_ms_{500.0};
  double dt_{0.01};
  // Friction feedforward parameter (viscous only, no Coulomb term — see note above)
  double viscous_drag_{0.00038};
  // L1 parameters
  double l1_as_{50.0};
  double l1_gamma_1_{5.0};
  double l1_gamma_2_{5.0};
  double l1_omega_f_{5.0};
  double l1_theta_1_min_{2.0};
  double l1_theta_1_nom_{0.0};
  double l1_theta_2_nom_{0.0};
  // L1 state
  double l1_v_hat_{0.0};
  double l1_theta_hat_1_{0.0};
  double l1_theta_hat_2_{0.0};
  double l1_theta_f_1_{0.0};
  double l1_theta_f_2_{0.0};
  // UDE parameters
  double ude_lambda_{10.0};
  double ude_inertia_{0.000995};
  double ude_integral_limit_{0.06};
  // Shared state (used by UDE integrand and L1 predictor)
  double tau_applied_prev_{0.0};
  double tau_f_prev_{0.0};
  // UDE state
  double ude_integral_term_{0.0};
  double ude_tau_d_hat_{0.0};
};

}  // namespace ak_motor_cable_control_emulator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak_motor_cable_control_emulator::CableTorqueCtrlPdL1Node>());
  rclcpp::shutdown();
  return 0;
}
