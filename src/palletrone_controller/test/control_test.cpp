#include <iostream>
#include <palletrone_controller/control.hpp>
#include <palletrone_interfaces/command.hpp>

using namespace palletrone;
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
int main(int argc, char** argv) {
  try {
    if (argc != 4) throw std::runtime_error("control_test CONTROL_YAML MODEL_YAML COMMAND_YAML");
    Config c(argv[1]);
    ModelConfig m{Config(argv[2])};
    QFilter filter(2.0);
    for (int i = 0; i < 6000; ++i) filter.update(Vec3(1, 2, 3), 0.0025);
    check((filter.value() - Vec3(1, 2, 3)).norm() < 1e-7, "Q filter DC gain");
    check(filter.derivative().norm() < 1e-7, "Q derivative DC rejection");
    TorqueDob dob(c, m);
    Vec3 omega = Vec3::Zero(), estimate = Vec3::Zero();
    const Vec3 disturbance(0.2, -0.15, 0.1);
    for (int i = 0; i < 10000; ++i) {
      const Vec3 torque = -2 * omega - estimate;
      omega += 0.0025 * m.inertia.cwiseInverse().cwiseProduct(torque + disturbance);
      estimate = dob.update(omega, torque, 0.0025);
    }
    check((estimate - disturbance).norm() < 0.001, "DOB torque sign/magnitude");
    check(omega.norm() < 0.001, "DOB compensation rejects constant torque");
    dob.reset(omega);
    check(dob.estimate().norm() == 0, "DOB reset");
    Moce moce(c, m);
    const Vec3 actual(0.03, -0.02,
                      c.get<bool>("moce.estimate_z") ? 0.04 : m.initial_com.z());
    const Vec3 force(0, 0, m.mass * m.gravity);
    for (int i = 0; i < 30000; ++i)
      moce.update(force, skew(force) * (actual - moce.estimate()), 0.01, true);
    check((moce.estimate().head<2>() - actual.head<2>()).norm() < 0.001,
          "MOCE horizontal convergence/sign");
    check(std::abs(moce.estimate().z() - m.initial_com.z()) < 1e-12,
          "MOCE vertical excitation gate");
    const Vec3 held = moce.estimate();
    for (int i = 0; i < 100; ++i) moce.update(Vec3(5, 5, 40), Vec3(1, 1, 1), 0.01, false);
    check((moce.estimate() - held).norm() == 0, "MOCE disabled/frozen behavior");
    for (int i = 0; i < 40000; ++i) {
      const Vec3 f(8 * std::sin(0.1 * i * 0.01), 8 * std::cos(0.1 * i * 0.01), m.mass * m.gravity);
      moce.update(f, skew(f) * (actual - moce.estimate()), 0.01, true);
    }
    check((moce.estimate().head<2>() - actual.head<2>()).norm() < 0.002,
          "MOCE XY convergence under excitation");
    if (c.get<bool>("moce.estimate_z"))
      check(std::abs(moce.estimate().z() - actual.z()) < 0.002,
            "Enabled MOCE Z convergence under excitation");
    else
      check(std::abs(moce.estimate().z() - m.initial_com.z()) < 1e-12,
            "Disabled MOCE Z remains at its initial estimate");
    Allocator allocator(c, m);
    const auto hover = allocator.update(force, Vec3::Zero(), Vec4::Zero(), Vec3::Zero(), 0.0025);
    check((hover.thrust - Vec4::Constant(force.z() / 4)).norm() < 1e-8, "Hover allocation");
    check(hover.angle.norm() < 1e-8, "Hover servo angle");
    const auto shifted =
        allocator.update(force, Vec3::Zero(), Vec4::Zero(), Vec3(0.03, -0.02, 0), 0.0025);
    Vec3 moment = Vec3::Zero();
    for (int i = 0; i < 4; ++i)
      moment += (m.rotorPosition(i) - Vec3(0.03, -0.02, 0)).cross(Vec3(0, 0, shifted.thrust[i]));
    check(moment.norm() < 1e-8, "CoM estimate changes allocation moment arms");
    const auto saturated = allocator.update(Vec3(1000, -1000, 500), Vec3(100, -100, 100),
                                            Vec4::Zero(), Vec3::Zero(), 0.0025);
    check(saturated.angle.allFinite() &&
              saturated.angle.cwiseAbs().maxCoeff() <= m.servo_limit + 1e-12,
          "Servo command saturation");
    check(saturated.thrust.minCoeff() >= 0 && saturated.thrust.maxCoeff() <= m.max_thrust,
          "Thrust saturation");
    const auto zero =
        allocator.update(Vec3::Zero(), Vec3::Zero(), Vec4::Zero(), Vec3::Zero(), 0.0025);
    check(zero.angle.allFinite() && zero.speed.allFinite(), "Singular zero-thrust allocation");
    CascadeController controller(c, m);
    controller.setMoceEnabled(true);
    check(controller.moceEnabled(), "Runtime MOCE request can enable an initially disabled estimator");
    controller.setMoceEnabled(false);
    check(!controller.moceEnabled(), "Runtime MOCE request can disable an allowed estimator");
    State state;
    state.position = Vec3(0, 0, 2);
    Reference ref;
    ref.position = state.position;
    const auto out = controller.update(state, ref);
    check((out.force - force).norm() < 1e-8 && out.torque.norm() < 1e-8,
          "Cascade equilibrium/gravity compensation");
    CascadeController feedforward_controller(c, m);
    Reference feedforward_ref;
    feedforward_ref.position = state.position;
    feedforward_ref.velocity = Vec3(0.20, -0.15, 0.10);
    const auto feedforward_out = feedforward_controller.update(state, feedforward_ref);
    check((feedforward_out.velocity_sp - feedforward_ref.velocity).norm() < 1e-12,
          "Analytic trajectory velocity is added to the position-loop velocity setpoint");
    state.time = 1;
    const auto reset = controller.update(state, ref);
    check(reset.force.allFinite() && reset.torque.allFinite(), "Sample-gap reset");
    CommandConfig command_config{Config(argv[3])};
    const bool initial_moce_setting = c.get<bool>("moce.enabled");
    CommandGenerator trajectory(command_config, initial_moce_setting);
    check(trajectory.moceEnabled() == initial_moce_setting,
          "Command MOCE state starts from control.yaml");
    const auto manual = trajectory.at(0);
    check((manual.position - command_config.startup).norm() < 1e-12,
          "Configured startup command");
    trajectory.handle(command_config.keys.at("forward"), 0, Vec3::Zero(), Vec3::Zero());
    check(std::abs(trajectory.at(0).position.x() -
                   (command_config.startup.x() + command_config.tick.x())) < 1e-12,
          "Configured keyboard tick");
    trajectory.handle(command_config.keys.at("trajectory_toggle"), 2, Vec3::Zero(), Vec3::Zero());
    const auto start = trajectory.at(2);
    check((start.position - (command_config.center +
                             command_config.amplitude.cwiseProduct(command_config.phase.array().sin().matrix())))
                                 .norm() < 1e-12,
          "Configured Lissajous equation");
    trajectory.handle(command_config.keys.at("moce_toggle"), 2, Vec3::Zero(), Vec3::Zero());
    check(trajectory.moceEnabled() != initial_moce_setting, "MOCE command toggle");
    const double t = 10, h = 1e-4;
    check(((trajectory.at(t + h).position - trajectory.at(t - h).position) / (2 * h) -
           trajectory.at(t).velocity)
                  .norm() < 1e-6,
          "Trajectory velocity feedforward");
    auto locked_config = command_config;
    locked_config.manual_cancels = false;
    locked_config.repeat = true;
    CommandGenerator locked(locked_config);
    locked.handle(locked_config.keys.at("forward"), 0, Vec3::Zero(), Vec3::Zero());
    locked.handle(locked_config.keys.at("yaw_left"), 0, Vec3::Zero(), Vec3::Zero());
    locked.handle(locked_config.keys.at("trajectory_toggle"), 2, Vec3::Zero(), Vec3::Zero());
    CommandGenerator uninterrupted = locked;
    for (const char* name : {"forward", "backward", "left", "right", "up", "down", "yaw_left", "yaw_right"}) {
      locked.handle(locked_config.keys.at(name), 3, Vec3::Zero(), Vec3::Zero());
      const auto actual = locked.at(3), expected = uninterrupted.at(3);
      check(locked.trajectoryActive() && (actual.position - expected.position).norm() < 1e-12 &&
                (actual.rpy - expected.rpy).norm() < 1e-12 &&
                (actual.velocity - expected.velocity).norm() < 1e-12 &&
                (actual.acceleration - expected.acceleration).norm() < 1e-12,
            "Movement/yaw keys must not change an active locked trajectory");
    }
    const auto stop = uninterrupted.at(4);
    locked.handle(locked_config.keys.at("trajectory_toggle"), 4, Vec3::Zero(), Vec3::Zero());
    const auto hold = locked.at(5);
    check(!locked.trajectoryActive() && (hold.position - stop.position).norm() < 1e-12 &&
              (hold.rpy - stop.rpy).norm() < 1e-12 && hold.velocity.isZero() && hold.acceleration.isZero(),
          "Stopping Lissajous holds its current command without feedforward");
    locked.handle(locked_config.keys.at("forward"), 5, Vec3::Zero(), Vec3::Zero());
    const Vec3 next = (stop.position + Vec3(locked_config.tick.x(), 0, 0))
                         .cwiseMax(locked_config.minimum).cwiseMin(locked_config.maximum);
    check((locked.at(5).position - next).norm() < 1e-12,
          "Manual control resumes from the stopped trajectory, not the old manual command");
    auto cancel_config = locked_config;
    cancel_config.manual_cancels = true;
    CommandGenerator cancel(cancel_config);
    cancel.handle(cancel_config.keys.at("trajectory_toggle"), 2, Vec3::Zero(), Vec3::Zero());
    const Vec3 cancel_position = cancel.at(4).position;
    cancel.handle(cancel_config.keys.at("forward"), 4, Vec3::Zero(), Vec3::Zero());
    check(!cancel.trajectoryActive() &&
              (cancel.at(5).position - (cancel_position + Vec3(cancel_config.tick.x(), 0, 0))
                                          .cwiseMax(cancel_config.minimum).cwiseMin(cancel_config.maximum))
                      .norm() < 1e-12,
          "Opt-in manual cancellation starts at the current trajectory command");
    CommandGenerator experiment(command_config, true, false);
    const double experiment_start = 10.0;
    experiment.handle(command_config.keys.at("automated_experiment"), experiment_start,
                      Vec3::Zero(), Vec3::Zero());
    check(experiment.automatedExperimentActive() && !experiment.moceEnabled() &&
              !experiment.moceZEnabled(),
          "M starts the automated experiment with all MOCE axes disabled");
    const auto ascent_start = experiment.at(experiment_start);
    check((ascent_start.position - command_config.startup).norm() < 1e-12 &&
              ascent_start.velocity.isZero() && ascent_start.acceleration.isZero(),
          "Automated ascent starts continuously from the current command");
    const double ascent_end = experiment_start + command_config.experiment_ascent_duration;
    const auto ascent_mid = experiment.at(
        experiment_start + 0.5 * command_config.experiment_ascent_duration);
    check((ascent_mid.position -
           0.5 * (command_config.startup + command_config.experiment_hover_position)).norm() < 1e-12,
          "Automated ascent uses the configured minimum-jerk midpoint");
    const auto hover_start = experiment.at(ascent_end);
    check((hover_start.position - command_config.experiment_hover_position).norm() < 1e-12 &&
              hover_start.velocity.isZero() && !experiment.moceEnabled(),
          "Automated experiment begins hover after ascent");
    experiment.at(ascent_end + command_config.experiment_moce_start - 1e-6);
    check(!experiment.moceEnabled(), "MOCE remains off before the scheduled hover time");
    experiment.at(ascent_end + command_config.experiment_moce_start);
    check(!experiment.moceEnabled() && !experiment.moceZEnabled(),
          "MOCE remains off throughout automated hover");
    const double curve_start = ascent_end + command_config.experiment_hover_duration;
    const auto curve = experiment.at(curve_start);
    const Vec3 expected_curve_start =
        command_config.center +
        command_config.amplitude.cwiseProduct(command_config.phase.array().sin().matrix());
    check(!experiment.moceEnabled() && !experiment.moceZEnabled() &&
              (curve.position - expected_curve_start).norm() < 1e-12 &&
              (curve.velocity -
               command_config.amplitude.cwiseProduct(
                   (2 * M_PI * command_config.frequency.array()).matrix())
                   .cwiseProduct(command_config.phase.array().cos().matrix())).norm() < 1e-12,
          "Automated experiment starts analytical Lissajous with MOCE disabled");
    const double curve_end = curve_start + command_config.experiment_lissajous_duration;
    const auto finish = experiment.at(curve_end);
    check(!experiment.automatedExperimentActive() &&
              (finish.position - command_config.experiment_hover_position).norm() < 1e-12 &&
              finish.velocity.isZero() && finish.acceleration.isZero(),
          "Automated experiment returns to hover after 60 seconds of Lissajous");
    std::cout << "PASS: PID cascade, Q filter, DOB rejection, MOCE XYZ/gating, CoM allocation, "
                 "actuator bounds, keyboard command, Lissajous trajectory\n";
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
