#include <plant/model.hpp>

namespace palletrone {
PlantModel::PlantModel(const std::string& scene, const Config& control, const ModelConfig& config)
    : config_(config),
      thrust_scale_(control.vector4("motor.thrust_scale", true)),
      rng_(config.seed),
      residual_rng_(config.residual_seed),
      residual_force_rng_(config.residual_force_seed) {
  if (thrust_scale_.maxCoeff() > 1)
    throw std::runtime_error("motor.thrust_scale values must be within [0,1]");
  char error[2048]{};
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
      mj_parseXML(scene.c_str(), nullptr, error, sizeof(error)), mj_deleteSpec);
  if (!spec) throw std::runtime_error(std::string("MuJoCo XML parse: ") + error);
  auto body = [&](const std::string& name) {
    auto* b = mjs_findBody(spec.get(), name.c_str());
    if (!b) throw std::runtime_error("Missing body: " + name);
    return b;
  };
  auto element = [&](mjtObj type, const std::string& name) {
    auto* e = mjs_findElement(spec.get(), type, name.c_str());
    if (!e) throw std::runtime_error("Missing model element: " + name);
    return e;
  };
  spec->option.timestep = 1 / config.physics_hz;
  spec->option.gravity[2] = -config.gravity;
  spec->option.integrator = mjINT_IMPLICITFAST;
  // Apply inertial/geometry changes BEFORE compilation. Editing body_ipos after
  // compilation leaves MuJoCo's sameframe/simple-body optimizations stale.
  auto* base = body("base");
  const double unloaded_base_mass = config.vehicle_mass - 4 * config.rotor_mass;
  base->mass = unloaded_base_mass + config.payload_mass;
  base->explicitinertial = 1;
  for (int k = 0; k < 4; ++k) base->iquat[k] = (k == 0 ? 1 : 0);
  Eigen::Quaterniond orientation(rotation(config.initial_rpy));
  base->quat[0] = orientation.w();
  base->quat[1] = orientation.x();
  base->quat[2] = orientation.y();
  base->quat[3] = orientation.z();
  for (int k = 0; k < 3; ++k) {
    base->pos[k] = config.initial_position[k];
  }
  Vec3 rotor_moment = Vec3::Zero();
  const double kp = control.scalar("servo.kp", 0, true), kd = control.scalar("servo.kd");
  for (int i = 0; i < 4; ++i) {
    const std::string n = std::to_string(i + 1);
    auto *mount = body("servo" + n), *prop = body("prop" + n + "_body");
    auto* joint = mjs_asJoint(element(mjOBJ_JOINT, "j_servo" + n));
    auto* motor = mjs_asActuator(element(mjOBJ_ACTUATOR, "BLDC" + n));
    auto* servo = mjs_asActuator(element(mjOBJ_ACTUATOR, "servo" + n + "_pos"));
    const Vec3 p = config.rotorPosition(i);
    for (int k = 0; k < 3; ++k) {
      mount->pos[k] = p[k] + (k == 2 ? config.pivot : 0);
      prop->pos[k] = (k == 2 ? -config.pivot : 0);
      joint->pos[k] = prop->pos[k];
      prop->ipos[k] = config.rotor_com[k];
      prop->inertia[k] = config.rotor_inertia[k];
    }
    prop->mass = config.rotor_mass;
    prop->explicitinertial = 1;
    for (int k = 0; k < 4; ++k) prop->iquat[k] = (k == 0 ? 1 : 0);
    const auto* q = mount->quat;
    rotor_moment +=
        config.rotor_mass *
        (p + Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized() * config.rotor_com);
    joint->limited = 1;
    joint->range[0] = -config.servo_limit;
    joint->range[1] = config.servo_limit;
    joint->damping = config.servo_damping;
    joint->armature = config.servo_armature;
    servo->ctrllimited = 1;
    servo->ctrlrange[0] = -config.servo_limit;
    servo->ctrlrange[1] = config.servo_limit;
    servo->forcelimited = 1;
    servo->forcerange[0] = -config.servo_max_torque;
    servo->forcerange[1] = config.servo_max_torque;
    servo->gainprm[0] = kp;
    servo->biasprm[1] = -kp;
    servo->biasprm[2] = -kd;
    motor->ctrllimited = 1;
    motor->ctrlrange[0] = 0;
    motor->ctrlrange[1] = config.max_thrust;
    motor->gear[5] = (i % 2 == 0 ? 1 : -1) * config.reaction_ratio;
  }
  // Merge the rigid payload into the base inertial body. Its location and mass
  // determine total mass, CoM, and the full parallel-axis inertia tensor.
  const Vec3 base_com = (config.mass * config.com - rotor_moment) / base->mass;
  const Vec3 unloaded_base_com = -rotor_moment / unloaded_base_mass;
  auto parallel_axis = [](double mass, const Vec3& offset) -> Mat3 {
    return mass * (offset.squaredNorm() * Mat3::Identity() - offset * offset.transpose());
  };
  const Mat3 base_inertia_tensor =
      config.base_inertia.asDiagonal().toDenseMatrix() +
      parallel_axis(unloaded_base_mass, unloaded_base_com - base_com) +
      parallel_axis(config.payload_mass, config.payload_position - base_com);
  Eigen::SelfAdjointEigenSolver<Mat3> eigensolver(base_inertia_tensor);
  Vec3 base_inertia = eigensolver.eigenvalues();
  Mat3 principal_axes = eigensolver.eigenvectors();
  // Eigenvectors may form a reflection; a quaternion must represent a proper rotation.
  if (principal_axes.determinant() < 0) principal_axes.col(0) *= -1;
  if (eigensolver.info() != Eigen::Success || !base_inertia.allFinite() ||
      2 * base_inertia.maxCoeff() > base_inertia.sum()) {
    throw std::runtime_error("Derived base inertia violates the principal-moment triangle rule: " +
                             std::to_string(base_inertia.x()) + ", " +
                             std::to_string(base_inertia.y()) + ", " +
                             std::to_string(base_inertia.z()));
  }
  const Eigen::Quaterniond inertia_orientation(principal_axes);
  base->iquat[0] = inertia_orientation.w();
  base->iquat[1] = inertia_orientation.x();
  base->iquat[2] = inertia_orientation.y();
  base->iquat[3] = inertia_orientation.z();
  for (int k = 0; k < 3; ++k) {
    base->ipos[k] = base_com[k];
    base->inertia[k] = base_inertia[k];
  }
  auto* payload_marker = mjs_asSite(element(mjOBJ_SITE, "payload_marker"));
  for (int k = 0; k < 3; ++k) payload_marker->pos[k] = config.payload_position[k];
  model_.reset(mj_compile(spec.get(), nullptr));
  if (!model_) throw std::runtime_error(std::string("MuJoCo compile: ") + mjs_getError(spec.get()));
  auto id = [this](mjtObj type, const std::string& name) {
    const int result = mj_name2id(model_.get(), type, name.c_str());
    if (result < 0) throw std::runtime_error("Missing compiled object: " + name);
    return result;
  };
  base_ = id(mjOBJ_BODY, "base");
  pos_sensor_ = id(mjOBJ_SENSOR, "base_pos");
  vel_sensor_ = id(mjOBJ_SENSOR, "base_linvel");
  gyro_sensor_ = id(mjOBJ_SENSOR, "body_gyro");
  for (int i = 0; i < 4; ++i) {
    const auto n = std::to_string(i + 1);
    joint_[i] = id(mjOBJ_JOINT, "j_servo" + n);
    motor_[i] = id(mjOBJ_ACTUATOR, "BLDC" + n);
    servo_[i] = id(mjOBJ_ACTUATOR, "servo" + n + "_pos");
  }
  data_.reset(mj_makeData(model_.get()));
  if (!data_) throw std::runtime_error("MuJoCo data allocation failed");
  mj_forward(model_.get(), data_.get());
}

void PlantModel::setInput(const Vec4& speed, const Vec4& angle) {
  if (!speed.allFinite() || !angle.allFinite())
    throw std::runtime_error("Nonfinite actuator command");
  const Vec4 nominal_thrust = (config_.thrust_coefficient * speed.cwiseMax(0).array().square())
                                  .matrix()
                                  .cwiseMin(config_.max_thrust);
  if (!effectiveness_started_ && nominal_thrust.maxCoeff() > 1e-9) {
    effectiveness_start_time_ = data_->time;
    effectiveness_started_ = true;
  }
  if (config_.residual_enabled && !residual_started_ && nominal_thrust.maxCoeff() > 1e-9) {
    for (int k = 0; k < 3; ++k)
      ou_torque_body_[k] = config_.residual_std[k] * residual_noise_(residual_rng_);
    residual_started_ = true;
  }
  if (config_.residual_force_enabled && !residual_force_started_ &&
      nominal_thrust.maxCoeff() > 1e-9) {
    for (int k = 0; k < 3; ++k)
      ou_force_body_[k] =
          config_.residual_force_std[k] * residual_force_noise_(residual_force_rng_);
    residual_force_started_ = true;
  }
  // T_actual_i = eta_common(t_effective) * eta_relative_i * T_nominal_i.
  // Scaling precedes the existing transport delay and first-order motor lag.
  const Vec4 thrust = actualThrustScale().cwiseProduct(nominal_thrust);
  motor_delay_.push(data_->time, thrust);
  servo_delay_.push(data_->time,
                    (config_.servo_gain * angle)
                        .cwiseMax(-config_.servo_limit)
                        .cwiseMin(config_.servo_limit));
  last_input_ = data_->time;
}
double PlantModel::effectivenessForElapsed(double elapsed) const {
  if (!config_.effectiveness_enabled) return 1.0;
  return std::clamp(config_.effectiveness_initial + config_.effectiveness_slope * elapsed,
                    config_.effectiveness_min, config_.effectiveness_max);
}
double PlantModel::commonEffectiveness() const {
  const double elapsed = effectiveness_started_ ? data_->time - effectiveness_start_time_ : 0.0;
  return effectivenessForElapsed(elapsed);
}
Vec3 PlantModel::sensor3(int id) const {
  const auto* v = data_->sensordata + model_->sensor_adr[id];
  return Vec3(v[0], v[1], v[2]);
}
Vec3 PlantModel::noisy(const Vec3& x, const Vec3& sigma) {
  Vec3 result = x;
  for (int k = 0; k < 3; ++k) result[k] += sigma[k] * noise_(rng_);
  return result;
}
PlantSample PlantModel::truth() const {
  PlantSample s;
  s.time = data_->time;
  s.position = sensor3(pos_sensor_);
  s.velocity = sensor3(vel_sensor_);
  s.omega = sensor3(gyro_sensor_);
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> r(data_->xmat + 9 * base_);
  s.rpy = rpyFromRotation(r);
  s.acceleration = Eigen::Map<const Vec3>(data_->qacc);
  s.angular_acceleration = Eigen::Map<const Vec3>(data_->qacc + 3);
  s.truth_acceleration = s.acceleration;
  s.truth_angular_acceleration = s.angular_acceleration;
  for (int i = 0; i < 4; ++i) s.servo[i] = data_->qpos[model_->jnt_qposadr[joint_[i]]];
  s.common_effectiveness = commonEffectiveness();
  s.actual_thrust_scale = actualThrustScale();
  s.ou_torque_body = ou_torque_body_;
  s.total_external_torque_body = total_external_torque_body_;
  s.ou_force_body = ou_force_body_;
  s.total_external_force_body = total_external_force_body_;
  s.motor_force = motor_force_;
  return s;
}
std::optional<PlantSample> PlantModel::step() {
  auto* m = model_.get();
  auto* d = data_.get();
  if (d->time - last_input_ > config_.timeout) {
    motor_delay_.clear(Vec4::Zero());
    servo_delay_.clear(Vec4::Zero());
  }
  const Vec4 target = motor_delay_.sample(d->time - config_.motor_delay);
  const Vec4 angle = servo_delay_.sample(d->time - config_.servo_delay);
  motor_force_ += (config_.motor_tau > 0 ? 1 - std::exp(-m->opt.timestep / config_.motor_tau) : 1) *
                  (target - motor_force_);
  for (int i = 0; i < 4; ++i) {
    d->ctrl[motor_[i]] = motor_force_[i];
    d->ctrl[servo_[i]] = angle[i];
  }
  mju_zero(d->qfrc_applied, m->nv);
  if (residual_started_) {
    for (int k = 0; k < 3; ++k) {
      const double a = std::exp(-m->opt.timestep / config_.residual_tau[k]);
      ou_torque_body_[k] =
          a * ou_torque_body_[k] +
          config_.residual_std[k] * std::sqrt(1 - a * a) * residual_noise_(residual_rng_);
    }
  }
  if (residual_force_started_) {
    for (int k = 0; k < 3; ++k) {
      const double a = std::exp(-m->opt.timestep / config_.residual_force_tau[k]);
      ou_force_body_[k] =
          a * ou_force_body_[k] + config_.residual_force_std[k] * std::sqrt(1 - a * a) *
                                      residual_force_noise_(residual_force_rng_);
    }
  }
  const bool constant_active =
      d->time >= config_.disturbance_start && d->time < config_.disturbance_end;
  total_external_torque_body_ =
      ou_torque_body_ + (constant_active ? config_.disturbance_torque : Vec3::Zero());
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> r(d->xmat + 9 * base_);
  applied_external_torque_world_ = r * total_external_torque_body_;
  const Vec3 constant_force_world =
      constant_active ? config_.disturbance_force : Vec3::Zero();
  applied_external_force_world_ = constant_force_world + r * ou_force_body_;
  total_external_force_body_ = ou_force_body_ + r.transpose() * constant_force_world;
  applied_force_point_world_ = Eigen::Map<const Vec3>(d->subtree_com + 3 * base_);
  if (!applied_external_force_world_.isZero() || !applied_external_torque_world_.isZero())
    mj_applyFT(m, d, applied_external_force_world_.data(), applied_external_torque_world_.data(),
               applied_force_point_world_.data(), base_, d->qfrc_applied);
  mj_step(m, d);
  // Refresh poses/sensors at the integrated time (mj_step sensors otherwise lag qpos).
  mj_forward(m, d);
  for (int i = 0; i < m->nq; ++i)
    if (!std::isfinite(d->qpos[i])) throw std::runtime_error("Nonfinite MuJoCo state");
  if (d->time + 1e-10 >= next_sample_) {
    PlantSample s = truth();
    s.position = noisy(s.position, config_.pos_noise);
    s.velocity = noisy(s.velocity, config_.vel_noise);
    s.rpy = noisy(s.rpy, config_.att_noise);
    s.omega = noisy(s.omega, config_.gyro_noise);
    for (int i = 0; i < 4; ++i) s.servo[i] += config_.servo_noise * noise_(rng_);
    if (have_noisy_) {
      const double dt = s.time - previous_noisy_.time;
      s.acceleration = (s.velocity - previous_noisy_.velocity) / dt;
      s.angular_acceleration = (s.omega - previous_noisy_.omega) / dt;
    }
    previous_noisy_ = s;
    have_noisy_ = true;
    sensor_delay_.push_back(s);
    do {
      next_sample_ += 1 / config_.state_hz;
    } while (next_sample_ <= d->time + 1e-10);
  }
  if (!sensor_delay_.empty() &&
      sensor_delay_.front().time + config_.sensor_delay <= d->time + 1e-10) {
    auto sample = sensor_delay_.front();
    sensor_delay_.pop_front();
    return sample;
  }
  return std::nullopt;
}
}  // namespace palletrone
