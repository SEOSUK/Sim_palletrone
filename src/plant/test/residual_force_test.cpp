#include <iostream>
#include <plant/model.hpp>
#include <vector>

using namespace palletrone;

void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
  try {
    if (argc != 5)
      throw std::runtime_error("residual_force_test CONTROL MODEL_V5 MODEL_V4 SCENE_XML");
    Config control(argv[1]);
    ModelConfig v5{Config(argv[2])}, v4{Config(argv[3])};
    const std::string scene = argv[4];
    const Vec4 speed = Vec4::Constant(std::sqrt(12.0 / v5.thrust_coefficient));

    ModelConfig disabled = v5;
    disabled.residual_force_enabled = false;
    PlantModel disabled_plant(scene, control, disabled), v4_plant(scene, control, v4);
    for (int i = 0; i < 1000; ++i) {
      disabled_plant.setInput(speed, Vec4::Zero());
      v4_plant.setInput(speed, Vec4::Zero());
      const auto a = disabled_plant.step(), b = v4_plant.step();
      check(a && b, "Disabled comparison samples available");
      check((a->position - b->position).norm() == 0 && (a->velocity - b->velocity).norm() == 0 &&
                (a->rpy - b->rpy).norm() == 0 && (a->omega - b->omega).norm() == 0 &&
                (a->ou_torque_body - b->ou_torque_body).norm() == 0,
            "Disabled force residual exactly reproduces frozen v4");
    }

    ModelConfig stats_config = v5;
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
      history.push_back(stats.ouForceBody());
    }
    Vec3 mean = Vec3::Zero(), variance = Vec3::Zero(), acf_at_tau = Vec3::Zero();
    for (const auto& value : history) mean += value;
    mean /= samples;
    for (const auto& value : history) variance += (value - mean).cwiseAbs2();
    variance /= samples;
    for (int axis = 0; axis < 3; ++axis) {
      const int lag = static_cast<int>(
          std::llround(stats_config.residual_force_tau[axis] * stats_config.physics_hz));
      double covariance = 0;
      for (int i = lag; i < samples; ++i)
        covariance += (history[i][axis] - mean[axis]) *
                      (history[i - lag][axis] - mean[axis]);
      acf_at_tau[axis] = covariance / ((samples - lag) * variance[axis]);
    }
    const Vec3 empirical_std = variance.cwiseSqrt();
    check(((empirical_std - stats_config.residual_force_std).cwiseAbs().array() <
           0.06 * stats_config.residual_force_std.array()).all(),
          "Force OU stationary STD and axis mapping");
    check(((acf_at_tau.array() - std::exp(-1.0)).abs() < 0.06).all(),
          "Force OU empirical 1/e correlation time");

    PlantModel same(scene, control, stats_config);
    ModelConfig other_config = stats_config;
    ++other_config.residual_force_seed;
    PlantModel other(scene, control, other_config);
    same.setInput(Vec4::Ones(), Vec4::Zero());
    other.setInput(Vec4::Ones(), Vec4::Zero());
    check((same.ouForceBody() - other.ouForceBody()).norm() > 0,
          "Different force seeds produce different histories");
    PlantModel same_again(scene, control, stats_config);
    same_again.setInput(Vec4::Ones(), Vec4::Zero());
    check((same.ouForceBody() - same_again.ouForceBody()).norm() == 0,
          "Fixed force seed is exactly reproducible");

    ModelConfig transform_config = v5;
    transform_config.initial_rpy = Vec3(0.3, -0.2, 0.4);
    transform_config.disturbance_force = Vec3(0.3, -0.2, 0.1);  // legacy world frame
    transform_config.disturbance_start = 0;
    transform_config.disturbance_end = 10;
    PlantModel transformed(scene, control, transform_config);
    transformed.setInput(Vec4::Ones(), Vec4::Zero());
    const int base = mj_name2id(transformed.model(), mjOBJ_BODY, "base");
    const Eigen::Matrix3d rotation_before =
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            transformed.data()->xmat + 9 * base);
    const Vec3 com_before = Eigen::Map<const Vec3>(transformed.data()->subtree_com + 3 * base);
    transformed.step();
    check((transformed.appliedExternalForceWorld() -
           (transform_config.disturbance_force + rotation_before * transformed.ouForceBody()))
              .norm() < 1e-12,
          "Body OU and legacy world force are exactly additive");
    check((transformed.totalExternalForceBody() - transformed.ouForceBody() -
           rotation_before.transpose() * transform_config.disturbance_force).norm() < 1e-12,
          "Total force body diagnostic uses the correct frame transform");
    check((transformed.appliedForcePointWorld() - com_before).norm() < 1e-12,
          "External force is applied at the total-system subtree CoM");
    check((transformed.appliedForcePointWorld() - com_before)
                  .cross(transformed.appliedExternalForceWorld()).norm() < 1e-12,
          "Force-only CoM application creates no direct lever-arm torque");

    ModelConfig no_force = v5;
    no_force.residual_force_enabled = false;
    PlantModel torque_only(scene, control, no_force), force_and_torque(scene, control, v5);
    for (int i = 0; i < 1000; ++i) {
      torque_only.setInput(Vec4::Ones(), Vec4::Zero());
      force_and_torque.setInput(Vec4::Ones(), Vec4::Zero());
      torque_only.step();
      force_and_torque.step();
      check((torque_only.ouTorqueBody() - force_and_torque.ouTorqueBody()).norm() == 0,
            "Independent force RNG leaves frozen v4 torque OU unchanged");
    }

    std::cout << "force OU empirical_std=" << empirical_std.transpose()
              << " acf_at_configured_tau=" << acf_at_tau.transpose() << '\n';
    std::cout << "PASS: v4 compatibility, force OU statistics/seeds/frame/CoM, additive force, "
                 "and independent torque OU\n";
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
