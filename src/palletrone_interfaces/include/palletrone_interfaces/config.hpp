#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace palletrone {
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat3 = Eigen::Matrix3d;

// Required keys, with no hidden gain/model defaults in the executables.
class Config {
 public:
  explicit Config(const std::string& path) : root_(YAML::LoadFile(path)), path_(path) {}
  YAML::Node at(const std::string& key) const {
    YAML::Node n = YAML::Clone(root_);
    std::size_t start = 0;
    while (start < key.size()) {
      const auto end = key.find('.', start);
      const auto part = key.substr(start, end - start);
      const YAML::Node child = static_cast<const YAML::Node&>(n)[part];
      if (!child) throw std::runtime_error(path_ + ": missing " + key);
      n.reset(child);
      if (end == std::string::npos) break;
      start = end + 1;
    }
    return n;
  }
  template <typename T>
  T get(const std::string& key) const {
    return at(key).as<T>();
  }
  double scalar(const std::string& key, double minimum = 0.0, bool strict = false) const {
    const double v = get<double>(key);
    if (!std::isfinite(v) || (strict ? v <= minimum : v < minimum))
      throw std::runtime_error(path_ + ": invalid " + key);
    return v;
  }
  Vec3 vector(const std::string& key, bool nonnegative = false) const {
    const auto n = at(key);
    if (!n.IsSequence() || n.size() != 3)
      throw std::runtime_error(path_ + ": expected 3 values at " + key);
    Vec3 v(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
    if (!v.allFinite() || (nonnegative && v.minCoeff() < 0))
      throw std::runtime_error(path_ + ": invalid vector " + key);
    return v;
  }
  Vec4 vector4(const std::string& key, bool nonnegative = false) const {
    const auto n = at(key);
    if (!n.IsSequence() || n.size() != 4)
      throw std::runtime_error(path_ + ": expected 4 values at " + key);
    Vec4 v(n[0].as<double>(), n[1].as<double>(), n[2].as<double>(), n[3].as<double>());
    if (!v.allFinite() || (nonnegative && v.minCoeff() < 0))
      throw std::runtime_error(path_ + ": invalid vector " + key);
    return v;
  }

 private:
  YAML::Node root_;
  std::string path_;
};

inline Vec3 bound(const Vec3& x, const Vec3& limit) { return x.cwiseMax(-limit).cwiseMin(limit); }
inline Mat3 skew(const Vec3& v) {
  Mat3 m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}
inline Mat3 rotation(const Vec3& rpy) {
  return (Eigen::AngleAxisd(rpy.z(), Vec3::UnitZ()) * Eigen::AngleAxisd(rpy.y(), Vec3::UnitY()) *
          Eigen::AngleAxisd(rpy.x(), Vec3::UnitX()))
      .toRotationMatrix();
}
inline Vec3 rpyFromRotation(const Mat3& r) {
  return Vec3(std::atan2(r(2, 1), r(2, 2)), std::asin(std::clamp(-r(2, 0), -1.0, 1.0)),
              std::atan2(r(1, 0), r(0, 0)));
}

struct ModelConfig {
  double mass, vehicle_mass, payload_mass, gravity, rotor_mass, arm, rotor_z, pivot;
  double thrust_coefficient, reaction_ratio, max_thrust, motor_delay, motor_tau;
  double effectiveness_initial, effectiveness_slope, effectiveness_min, effectiveness_max;
  double servo_limit, servo_gain, servo_damping, servo_armature, servo_max_torque, servo_delay;
  double sensor_delay, servo_noise, physics_hz, state_hz, realtime_factor, viewer_hz, timeout;
  double disturbance_start, disturbance_end;
  int seed, residual_seed, residual_force_seed;
  bool effectiveness_enabled, residual_enabled, residual_force_enabled;
  Vec3 base_inertia, inertia, com, payload_position, initial_com, initial_position, initial_rpy;
  Vec3 rotor_inertia, rotor_com, pos_noise, vel_noise, att_noise, gyro_noise;
  Vec3 disturbance_force, disturbance_torque;
  Vec3 residual_std, residual_tau;
  Vec3 residual_force_std, residual_force_tau;
  explicit ModelConfig(const Config& c) {
    vehicle_mass = c.scalar("model.vehicle_mass", 0, true);
    payload_mass = c.scalar("model.payload.mass");
    payload_position = c.vector("model.payload.position");
    mass = vehicle_mass + payload_mass;
    gravity = c.scalar("model.gravity", 0, true);
    base_inertia = c.vector("model.base_inertia", true);
    inertia = c.vector("model.nominal_inertia", true);
    com = payload_mass * payload_position / mass;
    initial_com = c.vector("model.initial_com_estimate");
    initial_position = c.vector("model.initial_position");
    initial_rpy = c.vector("model.initial_rpy");
    rotor_mass = c.scalar("model.rotor_mass", 0, true);
    rotor_inertia = c.vector("model.rotor_inertia", true);
    rotor_com = c.vector("model.rotor_com");
    arm = c.scalar("model.arm_xy", 0, true);
    rotor_z = c.scalar("model.rotor_z", -10);
    pivot = c.scalar("model.servo_pivot_offset");
    thrust_coefficient = c.scalar("model.motor.thrust_coefficient", 0, true);
    reaction_ratio = c.scalar("model.motor.reaction_ratio", 0, true);
    max_thrust = c.scalar("model.motor.max_thrust", 0, true);
    motor_delay = c.scalar("model.motor.delay_s");
    motor_tau = c.scalar("model.motor.time_constant_s");
    effectiveness_enabled = c.get<bool>("model.motor.effectiveness.enabled");
    effectiveness_initial = c.scalar("model.motor.effectiveness.initial");
    effectiveness_slope = c.get<double>("model.motor.effectiveness.slope_per_s");
    effectiveness_min = c.scalar("model.motor.effectiveness.min");
    effectiveness_max = c.scalar("model.motor.effectiveness.max");
    servo_limit = c.scalar("model.servo.limit_rad", 0, true);
    servo_gain = c.scalar("model.servo.dc_gain", 0, true);
    servo_damping = c.scalar("model.servo.joint_damping");
    servo_armature = c.scalar("model.servo.armature");
    servo_max_torque = c.scalar("model.servo.max_torque", 0, true);
    servo_delay = c.scalar("model.servo.delay_s");
    sensor_delay = c.scalar("model.sensors.delay_s");
    servo_noise = c.scalar("model.sensors.servo_std");
    pos_noise = c.vector("model.sensors.position_std", true);
    vel_noise = c.vector("model.sensors.velocity_std", true);
    att_noise = c.vector("model.sensors.attitude_std", true);
    gyro_noise = c.vector("model.sensors.gyro_std", true);
    seed = c.get<int>("model.sensors.seed");
    disturbance_force = c.vector("model.disturbance.force_world");
    disturbance_torque = c.vector("model.disturbance.torque_body");
    disturbance_start = c.scalar("model.disturbance.start_s");
    disturbance_end = c.scalar("model.disturbance.end_s");
    residual_enabled = c.get<bool>("model.residual_torque.enabled");
    if (c.get<std::string>("model.residual_torque.frame") != "body")
      throw std::runtime_error("model.residual_torque.frame must be body");
    residual_std = c.vector("model.residual_torque.std_nm", true);
    residual_tau = c.vector("model.residual_torque.correlation_time_s", true);
    residual_seed = c.get<int>("model.residual_torque.seed");
    residual_force_enabled = c.get<bool>("model.residual_force.enabled");
    if (c.get<std::string>("model.residual_force.frame") != "body")
      throw std::runtime_error("model.residual_force.frame must be body");
    residual_force_std = c.vector("model.residual_force.std_n", true);
    residual_force_tau = c.vector("model.residual_force.correlation_time_s", true);
    residual_force_seed = c.get<int>("model.residual_force.seed");
    physics_hz = c.scalar("simulation.physics_hz", 0, true);
    state_hz = c.scalar("simulation.state_hz", 0, true);
    realtime_factor = c.scalar("simulation.realtime_factor", 0, true);
    viewer_hz = c.scalar("simulation.viewer_hz", 0, true);
    timeout = c.scalar("simulation.command_timeout_s", 0, true);
    if (vehicle_mass <= 4 * rotor_mass || base_inertia.minCoeff() <= 0 || inertia.minCoeff() <= 0 ||
        rotor_inertia.minCoeff() <= 0 || 2 * base_inertia.maxCoeff() > base_inertia.sum() ||
        2 * rotor_inertia.maxCoeff() > rotor_inertia.sum() || servo_limit >= 1.5 ||
        servo_gain > 1.0 || !std::isfinite(effectiveness_slope) ||
        effectiveness_min > effectiveness_max || effectiveness_initial < effectiveness_min ||
        effectiveness_initial > effectiveness_max ||
        state_hz > physics_hz || disturbance_end < disturbance_start ||
        (residual_enabled && residual_tau.minCoeff() <= 0) ||
        (residual_force_enabled && residual_force_tau.minCoeff() <= 0))
      throw std::runtime_error(
          "Invalid model mass/inertia/servo limit/sample rate/disturbance interval");
  }
  Vec3 rotorPosition(int i) const {
    const int sx[4] = {1, -1, -1, 1}, sy[4] = {1, 1, -1, -1};
    return Vec3(sx[i] * arm, sy[i] * arm, rotor_z);
  }
  Vec3 tangent(int i) const {
    const int sx[4] = {1, 1, -1, -1}, sy[4] = {-1, 1, 1, -1};
    return Vec3(sx[i] / std::sqrt(2.0), sy[i] / std::sqrt(2.0), 0);
  }
};
}  // namespace palletrone
