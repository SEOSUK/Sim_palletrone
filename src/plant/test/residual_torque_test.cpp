#include <deque>
#include <iostream>
#include <plant/model.hpp>

using namespace palletrone;

void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
  try {
    if (argc != 5)
      throw std::runtime_error("residual_torque_test CONTROL MODEL_V4 MODEL_V3 SCENE_XML");
    Config control(argv[1]);
    ModelConfig v4{Config(argv[2])}, v3{Config(argv[3])};
    const std::string scene = argv[4];

    ModelConfig disabled = v4;
    disabled.residual_enabled = false;
    PlantModel disabled_plant(scene, control, disabled), v3_plant(scene, control, v3);
    const Vec4 speed = Vec4::Constant(std::sqrt(12.0 / v4.thrust_coefficient));
    for (int i = 0; i < 1000; ++i) {
      disabled_plant.setInput(speed, Vec4::Zero());
      v3_plant.setInput(speed, Vec4::Zero());
      const auto a = disabled_plant.step(), b = v3_plant.step();
      check(a && b, "Disabled comparison samples available");
      check((a->position - b->position).norm() == 0 && (a->velocity - b->velocity).norm() == 0 &&
                (a->rpy - b->rpy).norm() == 0 && (a->omega - b->omega).norm() == 0,
            "Disabled residual exactly reproduces v3");
    }

    ModelConfig stats_config = v4;
    stats_config.gravity = 0;
    stats_config.disturbance_force.setZero();
    stats_config.disturbance_torque.setZero();
    PlantModel stats(scene, control, stats_config);
    stats.setInput(Vec4::Ones(), Vec4::Zero());
    constexpr int samples = 200000;
    std::vector<Vec3> history;
    history.reserve(samples);
    for (int i = 0; i < samples; ++i) {
      stats.setInput(Vec4::Ones(), Vec4::Zero());
      stats.step();
      history.push_back(stats.ouTorqueBody());
    }
    Vec3 mean = Vec3::Zero(), variance = Vec3::Zero(), acf_at_tau = Vec3::Zero();
    for (const auto& value : history) mean += value;
    mean /= samples;
    for (const auto& value : history) variance += (value - mean).cwiseAbs2();
    variance /= samples;
    for (int axis = 0; axis < 3; ++axis) {
      const int lag = static_cast<int>(std::llround(stats_config.residual_tau[axis] *
                                                    stats_config.physics_hz));
      double covariance = 0;
      for (int i = lag; i < samples; ++i)
        covariance += (history[i][axis] - mean[axis]) * (history[i - lag][axis] - mean[axis]);
      acf_at_tau[axis] = covariance / ((samples - lag) * variance[axis]);
    }
    const Vec3 empirical_std = variance.cwiseSqrt();
    check(((empirical_std - stats_config.residual_std).cwiseAbs().array() <
           0.06 * stats_config.residual_std.array()).all(),
          "OU stationary STD and axis mapping");
    check(((acf_at_tau.array() - std::exp(-1.0)).abs() < 0.06).all(),
          "OU empirical 1/e correlation time");

    PlantModel same(scene, control, stats_config);
    ModelConfig other_config = stats_config;
    ++other_config.residual_seed;
    PlantModel other(scene, control, other_config);
    same.setInput(Vec4::Ones(), Vec4::Zero());
    other.setInput(Vec4::Ones(), Vec4::Zero());
    check((same.ouTorqueBody() - stats.ouTorqueBody()).norm() > 0,
          "OU history advances from stationary initialization");
    check((same.ouTorqueBody() - other.ouTorqueBody()).norm() > 0,
          "Different residual seeds produce different histories");
    PlantModel same_again(scene, control, stats_config);
    same_again.setInput(Vec4::Ones(), Vec4::Zero());
    check((same.ouTorqueBody() - same_again.ouTorqueBody()).norm() == 0,
          "Fixed residual seed is exactly reproducible");

    ModelConfig transform_config = v4;
    transform_config.initial_rpy = Vec3(0.3, -0.2, 0.4);
    transform_config.disturbance_torque = Vec3(0.03, -0.02, 0.01);
    transform_config.disturbance_start = 0;
    transform_config.disturbance_end = 10;
    PlantModel transformed(scene, control, transform_config);
    transformed.setInput(Vec4::Ones(), Vec4::Zero());
    const int base = mj_name2id(transformed.model(), mjOBJ_BODY, "base");
    const Eigen::Matrix3d rotation_before =
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            transformed.data()->xmat + 9 * base);
    transformed.step();
    check((transformed.totalExternalTorqueBody() - transformed.ouTorqueBody() -
           transform_config.disturbance_torque).norm() < 1e-12,
          "Constant and OU body torque are exactly additive");
    check((transformed.appliedExternalTorqueWorld() -
           rotation_before * transformed.totalExternalTorqueBody()).norm() < 1e-12,
          "Body torque transforms correctly to world frame");

    ModelConfig zero_constant = transform_config;
    zero_constant.disturbance_torque.setZero();
    PlantModel zero(scene, control, zero_constant);
    zero.setInput(Vec4::Ones(), Vec4::Zero());
    zero.step();
    check((zero.totalExternalTorqueBody() - zero.ouTorqueBody()).norm() < 1e-12,
          "Zero constant disturbance leaves only OU torque");

    std::cout << "OU empirical_std=" << empirical_std.transpose()
              << " acf_at_configured_tau=" << acf_at_tau.transpose() << '\n';
    std::cout << "PASS: v3 compatibility, OU statistics/ACF/seeds, frame transform, and additive "
                 "physical torque\n";
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
