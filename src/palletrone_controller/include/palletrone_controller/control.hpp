#pragma once
#include <palletrone_interfaces/config.hpp>

namespace palletrone {
struct Pid {
  Vec3 kp, ki, kd, integral_limit, output_limit, integral = Vec3::Zero(), derivative = Vec3::Zero();
  double cutoff;
  Pid(const Config& c, const std::string& p)
      : kp(c.vector(p + ".kp", true)),
        ki(c.vector(p + ".ki", true)),
        kd(c.vector(p + ".kd", true)),
        integral_limit(c.vector(p + ".integral_limit", true)),
        output_limit(c.vector(p + ".output_limit", true)),
        cutoff(c.scalar(p + ".derivative_cutoff_hz", 0, true)) {
    if (output_limit.minCoeff() <= 0)
      throw std::runtime_error("PID output limits must be positive");
  }
  void reset() {
    integral.setZero();
    derivative.setZero();
  }
  Vec3 update(const Vec3& error, const Vec3& measurement_rate, double dt,
              const Vec3& feedforward = Vec3::Zero(), bool integrate = true) {
    derivative += (1 - std::exp(-2 * M_PI * cutoff * dt)) * (measurement_rate - derivative);
    Vec3 candidate = integral;
    if (integrate) candidate = bound(integral + ki.cwiseProduct(error) * dt, integral_limit);
    const Vec3 pd = kp.cwiseProduct(error) - kd.cwiseProduct(derivative) + feedforward;
    for (int i = 0; i < 3; ++i) {
      // Conditional integration: allow unwinding, reject growth into saturation.
      if (std::abs(pd[i] + candidate[i]) <= output_limit[i] ||
          error[i] * (pd[i] + candidate[i]) <= 0)
        integral[i] = candidate[i];
    }
    return bound(pd + integral, output_limit);
  }
};

// Algorithm reference: SEOSUK/PX4-Modifying @ 36300ac08efa6c11ef76679142dff7d12722ccfd
// src/modules/mc_rate_control/{torque_disturbance_observer,dob_based_com_estimator}.cpp
// Same second-order Q and MOCE law; exact ZOH discretization replaces Euler.
class QFilter {
 public:
  explicit QFilter(double cutoff) : w_(cutoff) {}
  void reset(const Vec3& input = Vec3::Zero()) {
    x_.row(0).setZero();
    x_.row(1) = input.transpose() / (w_ * w_);
  }
  void update(const Vec3& input, double dt) {
    const double a = w_ / std::sqrt(2.0), s = std::sin(a * dt), c = std::cos(a * dt),
                 e = std::exp(-a * dt);
    Eigen::Matrix2d transition;
    transition << c - s, -w_ * w_ * s / a, s / a, c + s;
    Eigen::Matrix<double, 2, 3> equilibrium = Eigen::Matrix<double, 2, 3>::Zero();
    equilibrium.row(1) = input.transpose() / (w_ * w_);
    x_ = equilibrium + e * transition * (x_ - equilibrium);
  }
  Vec3 value() const { return w_ * w_ * x_.row(1).transpose(); }
  Vec3 derivative() const { return w_ * w_ * x_.row(0).transpose(); }

 private:
  double w_;
  Eigen::Matrix<double, 2, 3> x_ = Eigen::Matrix<double, 2, 3>::Zero();
};

class TorqueDob {
 public:
  TorqueDob(const Config& c, const ModelConfig& m)
      : rate_(c.scalar("dob.cutoff_rad_s", 0, true)),
        torque_(c.scalar("dob.cutoff_rad_s", 0, true)),
        inertia_(m.inertia),
        limit_(c.vector("dob.estimate_limit", true)) {}
  void reset(const Vec3& omega = Vec3::Zero()) {
    rate_.reset(omega);
    torque_.reset();
    estimate_.setZero();
  }
  Vec3 update(const Vec3& omega, const Vec3& effective_torque, double dt) {
    rate_.update(omega, dt);
    torque_.update(effective_torque, dt);
    estimate_ = bound(inertia_.cwiseProduct(rate_.derivative()) - torque_.value(), limit_);
    return estimate_;
  }
  Vec3 estimate() const { return estimate_; }

 private:
  QFilter rate_, torque_;
  Vec3 inertia_, limit_, estimate_ = Vec3::Zero();
};

class Moce {
 public:
  Moce(const Config& c, const ModelConfig& m)
      : force_(c.scalar("moce.force_cutoff_rad_s", 0, true)),
        gamma_(c.vector("moce.gamma", true)),
        limit_(c.vector("moce.offset_limit", true)),
        rate_limit_(c.vector("moce.rate_limit", true)),
        inertia_(m.inertia),
        initial_(m.initial_com),
        tau_(c.scalar("moce.output_tau_s")),
        min_force_(c.scalar("moce.min_force")),
        min_horizontal_(c.scalar("moce.min_horizontal_force")),
        z_enabled_(c.get<bool>("moce.estimate_z")) {
    if ((initial_.cwiseAbs().array() > limit_.array()).any())
      throw std::runtime_error("Initial CoM exceeds MOCE bounds");
    reset();
  }
  void reset() {
    force_.reset();
    estimate_ = initial_;
    filtered_ = initial_;
    rate_.setZero();
  }
  void update(const Vec3& force, const Vec3& disturbance, double dt, bool active) {
    force_.update(force, dt);
    const Vec3 f = force_.value();
    rate_.setZero();
    if (active && f.norm() >= min_force_) {
      // FLU: residual torque = [F]x * (c_true - c_est).
      // Reference law: c_dot = Gamma * (J^-1 [QF]x)^T * d_hat (Nm).
      const Mat3 a = inertia_.cwiseInverse().asDiagonal() * skew(f);
      rate_ = bound(gamma_.cwiseProduct(a.transpose() * disturbance), rate_limit_);
      if (!z_enabled_ || f.head<2>().norm() < min_horizontal_) rate_.z() = 0;
      const Vec3 next = bound(estimate_ + rate_ * dt, limit_);
      rate_ = (next - estimate_) / dt;
      estimate_ = next;
    }
    filtered_ += (tau_ > 0 ? 1 - std::exp(-dt / tau_) : 1) * (estimate_ - filtered_);
  }
  Vec3 estimate() const { return estimate_; }
  Vec3 filtered() const { return filtered_; }
  Vec3 rate() const { return rate_; }
  void setZEnabled(bool enabled) { z_enabled_ = enabled; }

 private:
  QFilter force_;
  Vec3 gamma_, limit_, rate_limit_, inertia_, initial_, estimate_, filtered_, rate_;
  double tau_, min_force_, min_horizontal_;
  bool z_enabled_;
};

struct State {
  double time = 0;
  Vec3 position = Vec3::Zero(), velocity = Vec3::Zero(), rpy = Vec3::Zero(), omega = Vec3::Zero();
  Vec4 servo = Vec4::Zero();
  bool finite() const {
    return std::isfinite(time) && position.allFinite() && velocity.allFinite() && rpy.allFinite() &&
           omega.allFinite() && servo.allFinite();
  }
};
struct Reference {
  Vec3 position = Vec3::Zero(), velocity = Vec3::Zero(), acceleration = Vec3::Zero(),
       rpy = Vec3::Zero(), rate = Vec3::Zero();
  bool finite() const {
    return position.allFinite() && velocity.allFinite() && acceleration.allFinite() &&
           rpy.allFinite() && rate.allFinite();
  }
};
struct ControlOutput {
  Vec3 force = Vec3::Zero(), torque = Vec3::Zero(), velocity_sp = Vec3::Zero(),
       rate_sp = Vec3::Zero();
  Vec3 pid_torque = Vec3::Zero(), disturbance = Vec3::Zero(), com = Vec3::Zero(),
       com_filtered = Vec3::Zero(), com_rate = Vec3::Zero();
  bool adapting = false;
};

class CascadeController {
 public:
  CascadeController(const Config& c, const ModelConfig& m)
      : m_(m),
        position_(c, "controller.position"),
        velocity_(c, "controller.velocity"),
        attitude_(c, "controller.attitude"),
        rate_(c, "controller.rate"),
        dob_(c, m),
        moce_(c, m),
        force_limit_(c.vector("controller.force_limit", true)),
        torque_limit_(c.vector("controller.torque_limit", true)),
        dob_enabled_(c.get<bool>("dob.enabled")),
        compensate_(c.get<bool>("dob.compensate")),
        moce_enabled_(c.get<bool>("moce.enabled")),
        gyro_ff_(c.get<bool>("controller.gyroscopic_feedforward")),
        min_height_(c.scalar("controller.adaptation_min_height")),
        max_gap_(c.scalar("controller.max_sample_gap_s", 0, true)) {}
  void reset(const State& s) {
    position_.reset();
    velocity_.reset();
    attitude_.reset();
    rate_.reset();
    dob_.reset(s.omega);
    moce_.reset();
    previous_ = s;
    initialized_ = true;
  }
  void setMoceEnabled(bool enabled) { moce_enabled_ = enabled; }
  void setMoceZEnabled(bool enabled) { moce_.setZEnabled(enabled); }
  bool moceEnabled() const { return moce_enabled_; }
  ControlOutput update(const State& s, const Reference& ref) {
    if (!s.finite() || !ref.finite()) throw std::runtime_error("Nonfinite controller input");
    if (!initialized_ || s.time <= previous_.time || s.time - previous_.time > max_gap_) reset(s);
    double dt = s.time - previous_.time;
    if (dt <= 0) dt = 1 / m_.state_hz;
    const bool airborne = s.position.z() > min_height_;
    ControlOutput out;
    out.velocity_sp = position_.update(ref.position - s.position, s.velocity, dt, ref.velocity);
    const Vec3 acc = velocity_.update(out.velocity_sp - s.velocity,
                                      (s.velocity - previous_.velocity) / dt, dt, ref.acceleration);
    const Mat3 r = rotation(s.rpy), rd = rotation(ref.rpy);
    Vec3 f_world = m_.mass * (acc + Vec3(0, 0, m_.gravity));
    f_world.z() = std::max(0.0, f_world.z());
    out.force = bound(r.transpose() * f_world, force_limit_);
    Eigen::Quaterniond error(r.transpose() * rd);
    if (error.w() < 0) error.coeffs() *= -1;
    const Eigen::AngleAxisd angle(error);
    out.rate_sp = attitude_.update(angle.angle() * angle.axis(), s.omega, dt,
                                   r.transpose() * rd * ref.rate, airborne);
    out.pid_torque = rate_.update(out.rate_sp - s.omega, (s.omega - previous_.omega) / dt, dt,
                                  Vec3::Zero(), airborne);
    const Vec3 gyro = s.omega.cross(m_.inertia.cwiseProduct(s.omega));
    Vec3 nominal = out.pid_torque;
    if (gyro_ff_) nominal += gyro;
    const bool compensation = dob_enabled_ && compensate_ && airborne;
    Vec3 observer_input = nominal;
    if (compensation) observer_input -= dob_.estimate();
    observer_input = bound(observer_input, torque_limit_);
    if (airborne && (dob_enabled_ || moce_enabled_))
      out.disturbance = dob_.update(s.omega, observer_input - gyro, dt);
    else
      dob_.reset(s.omega);
    out.torque = nominal;
    if (compensation) out.torque -= out.disturbance;
    out.torque = bound(out.torque, torque_limit_);
    out.adapting = airborne && moce_enabled_;
    moce_.update(out.force, out.disturbance, dt, out.adapting);
    out.com = moce_.estimate();
    out.com_filtered = moce_.filtered();
    out.com_rate = moce_.rate();
    previous_ = s;
    return out;
  }

 private:
  ModelConfig m_;
  Pid position_, velocity_, attitude_, rate_;
  TorqueDob dob_;
  Moce moce_;
  Vec3 force_limit_, torque_limit_;
  bool dob_enabled_, compensate_, moce_enabled_, gyro_ff_, initialized_ = false;
  double min_height_, max_gap_;
  State previous_;
};

struct Allocation {
  Vec4 thrust = Vec4::Zero(), angle = Vec4::Zero(), speed = Vec4::Zero();
};
class Allocator {
 public:
  Allocator(const Config& c, const ModelConfig& m)
      : m_(m),
        cutoff_(c.scalar("allocator.yaw_split_cutoff_hz", 0, true)),
        yaw_limit_(c.scalar("allocator.yaw_reaction_limit")),
        regularization_(c.scalar("allocator.regularization", 0, true)) {}
  Allocation update(const Vec3& force, const Vec3& torque, const Vec4& measured, const Vec3& com,
                    double dt) {
    if (!force.allFinite() || !torque.allFinite() || !measured.allFinite() || !com.allFinite() ||
        !std::isfinite(dt) || dt <= 0)
      throw std::runtime_error("Invalid allocator input");
    yaw_lpf_ += (1 - std::exp(-2 * M_PI * cutoff_ * dt)) * (torque.z() - yaw_lpf_);
    const double reaction = std::clamp(torque.z() - yaw_lpf_, -yaw_limit_, yaw_limit_);
    Eigen::Matrix4d a1, a2;
    for (int i = 0; i < 4; ++i) {
      const double spin = (i % 2 == 0 ? 1 : -1);
      const Vec3 direction =
          m_.tangent(i) * std::sin(measured[i]) + Vec3::UnitZ() * std::cos(measured[i]);
      const Vec3 moment =
          (m_.rotorPosition(i) - com).cross(direction) + spin * m_.reaction_ratio * direction;
      a1.col(i) << moment.x(), moment.y(), spin * m_.reaction_ratio * direction.z(), direction.z();
    }
    Allocation out;
    out.thrust = solve(a1, Vec4(torque.x(), torque.y(), reaction, force.z()))
                     .cwiseMax(0)
                     .cwiseMin(m_.max_thrust);
    for (int i = 0; i < 4; ++i) {
      const Vec3 tangent = m_.tangent(i) * out.thrust[i];
      a2.col(i) << tangent.x(), tangent.y(), (m_.rotorPosition(i) - com).cross(tangent).z(),
          (i % 2 == 0 ? 1 : -1) * out.thrust[i];
    }
    // A2 solves sin(theta), not theta: retain accuracy up to +/-0.7 rad.
    const Vec4 sine = solve(a2, Vec4(force.x(), force.y(), torque.z() - reaction, 0));
    for (int i = 0; i < 4; ++i) {
      out.angle[i] =
          std::asin(std::clamp(sine[i], -std::sin(m_.servo_limit), std::sin(m_.servo_limit)));
      out.speed[i] = std::sqrt(out.thrust[i] / m_.thrust_coefficient);
    }
    return out;
  }

 private:
  Vec4 solve(const Eigen::Matrix4d& a, const Vec4& b) const {
    Eigen::FullPivLU<Eigen::Matrix4d> lu(a);
    if (lu.isInvertible()) return lu.solve(b);
    return (a.transpose() * a + regularization_ * Eigen::Matrix4d::Identity())
        .ldlt()
        .solve(a.transpose() * b);
  }
  ModelConfig m_;
  double cutoff_, yaw_limit_, regularization_, yaw_lpf_ = 0;
};
}  // namespace palletrone
