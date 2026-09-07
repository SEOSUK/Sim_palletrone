#include <fstream>
#include <iostream>
#include <limits>
#include <palletrone_controller/control.hpp>
#include <palletrone_interfaces/command.hpp>
#include <plant/model.hpp>

using namespace palletrone;
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
State state(const PlantSample& sample) {
  State s;
  s.time = sample.time;
  s.position = sample.position;
  s.velocity = sample.velocity;
  s.rpy = sample.rpy;
  s.omega = sample.omega;
  s.servo = sample.servo;
  return s;
}
int main(int argc, char** argv) {
  try {
    if (argc < 5)
      throw std::runtime_error(
          "simulation_test CONTROL_YAML MODEL_YAML COMMAND_YAML SCENE_XML [duration] [CSV]");
    Config c(argv[1]);
    ModelConfig m{Config(argv[2])};
    CommandConfig command_config{Config(argv[3])};
    const std::string scene = argv[4];
    const bool com_regression = argc > 7 && std::string(argv[7]) == "com";
    if (com_regression) {
      m.payload_mass = 0.5;
      m.mass = m.vehicle_mass + m.payload_mass;
      m.com = Vec3(0.03, -0.02, c.get<bool>("moce.estimate_z") ? 0.04 : 0.0);
      m.payload_position = m.com * m.mass / m.payload_mass;
    }
    const double duration = argc > 5 ? std::stod(argv[5]) : 46;
    PlantModel plant(scene, c, m);
    check(std::abs(mj_getTotalmass(plant.model()) - m.mass) < 1e-9, "Configured total mass");
    const int base = mj_name2id(plant.model(), mjOBJ_BODY, "base");
    const int payload_site = mj_name2id(plant.model(), mjOBJ_SITE, "payload_marker");
    check(payload_site >= 0, "Payload marker site exists");
    check((Eigen::Map<const Vec3>(plant.model()->site_pos + 3 * payload_site) -
           m.payload_position)
              .norm() < 1e-12,
          "MuJoCo payload marker uses canonical FLU body coordinates");
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> r(plant.data()->xmat + 9 * base);
    Vec3 actual_com = r.transpose() * (Eigen::Map<Vec3>(plant.data()->subtree_com + 3 * base) -
                                       Eigen::Map<Vec3>(plant.data()->xpos + 3 * base));
    check((actual_com - m.com).norm() < 1e-8, "Configured total CoM");
    check(std::abs(plant.model()->jnt_range[3] - m.servo_limit) < 1e-12, "Radian servo range");
    ModelConfig delayed = m;
    delayed.sensor_delay = 0.0125;
    delayed.motor_delay = 0.01;
    delayed.motor_tau = 0;
    delayed.pos_noise.setZero();
    delayed.vel_noise.setZero();
    delayed.gyro_noise.setZero();
    delayed.servo_noise = 0;
    PlantModel delay_test(scene, c, delayed);
    delay_test.setInput(Vec4::Constant(std::sqrt(5 / delayed.thrust_coefficient)), Vec4::Zero());
    const double dt = 1 / delayed.physics_hz;
    while (delay_test.data()->time < 0.03) {
      const double before = delay_test.data()->time;
      const auto sample = delay_test.step();
      const double actual_thrust = 5.0 * c.vector4("motor.thrust_scale", true)[0];
      check(std::abs(delay_test.data()->ctrl[0] -
                     (before + 1e-10 >= delayed.motor_delay ? actual_thrust : 0.0)) < 1e-9,
            "Actuator delay boundary");
      if (sample)
        check(std::abs(delay_test.data()->time - sample->time - delayed.sensor_delay) < dt + 1e-9,
              "Sensor delay preserves acquisition timestamp");
      else
        check(delay_test.data()->time < dt + delayed.sensor_delay + 1e-9,
              "Sensor release deadline");
    }
    // Isolate the servo while retaining its actual articulated mass/inertia.
    PlantModel servo_test(scene, c, m);
    servo_test.model()->opt.gravity[2] = 0;
    const int geom = mj_name2id(servo_test.model(), mjOBJ_GEOM, "servo1_vis");
    const Eigen::Matrix3d before = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        servo_test.data()->geom_xmat + 9 * geom);
    const double target = std::min(0.6, m.servo_limit * 0.9);
    std::vector<double> servo_time, servo_angle;
    for (int step = 0; step < static_cast<int>(3 * m.physics_hz); ++step) {
      mju_copy(servo_test.data()->qpos, servo_test.model()->qpos0, 7);
      mju_zero(servo_test.data()->qvel, 6);
      servo_test.setInput(Vec4::Zero(), Vec4(target, -target, target, -target));
      servo_test.step();
      servo_time.push_back(servo_test.data()->time);
      servo_angle.push_back(servo_test.truth().servo[0]);
    }
    const double servo_gain = servo_angle.back() / target;
    double best_error = std::numeric_limits<double>::infinity(), fitted_delay = 0, fitted_tau = 0;
    for (double delay = 0; delay <= 0.1 + 1e-12; delay += dt) {
      for (double tau = dt; tau <= 0.15 + 1e-12; tau += dt) {
        double error = 0;
        for (size_t i = 0; i < servo_time.size(); ++i) {
          const double predicted = servo_time[i] <= delay
                                       ? 0
                                       : servo_gain * target *
                                             (1 - std::exp(-(servo_time[i] - delay) / tau));
          error += (servo_angle[i] - predicted) * (servo_angle[i] - predicted);
        }
        if (error < best_error) {
          best_error = error;
          fitted_delay = delay;
          fitted_tau = tau;
        }
      }
    }
    std::cout << "servo_fopdt_gain=" << servo_gain << " delay_s=" << fitted_delay
              << " tau_s=" << fitted_tau << '\n';
    check(std::abs(servo_gain - 0.994) < 0.02, "Servo DC gain");
    check(std::abs(fitted_delay - 0.050) < 0.01, "Servo total delay");
    check(std::abs(fitted_tau - 0.052) < 0.02, "Servo total time constant");
    check((before - Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
                        servo_test.data()->geom_xmat + 9 * geom))
                  .norm() > 0.1,
          "Servo mesh rotation");
    CascadeController controller(c, m);
    if (com_regression) controller.setMoceEnabled(true);
    Allocator allocator(c, m);
    CommandGenerator trajectory(command_config);
    trajectory.handle(command_config.keys.at("trajectory_toggle"), 0, Vec3::Zero(), Vec3::Zero());
    auto target_at = [&](double time) {
      auto point = trajectory.at(time);
      // The published experiment equation uses its supplied world-Z origin.
      // Lift it only in this free-flight regression to stay above the test floor.
      point.position.z() += 5.775;
      if (com_regression && time >= 12) {
        const Vec3 origin = trajectory.at(12).position;
        Vec3 flight_origin = origin;
        flight_origin.z() += 5.775;
        const double t = time - 12, decay = std::exp(-t), phase = t - 1 + decay, speed = 1 - decay;
        const Vec3 tangent(std::cos(phase), std::sin(phase), 0);
        point.position = flight_origin + Vec3(std::sin(phase), 1 - std::cos(phase), 0);
        point.velocity = speed * tangent;
        point.acceleration =
            decay * tangent + speed * speed * Vec3(-std::sin(phase), std::cos(phase), 0);
      }
      return point;
    };
    double next_ref = 0;
    Reference ref;
    double position_squared = 0, attitude_squared = 0, max_angle = 0;
    int count = 0;
    ControlOutput out;
    std::ofstream csv;
    if (argc > 6 && std::string(argv[6]) != "-") {
      csv.open(argv[6]);
      csv << "time,x,y,z,ref_x,ref_y,ref_z,roll,pitch,yaw,com_x,com_y,com_z,dob_x,dob_y,dob_z\n";
    }
    while (plant.data()->time < duration) {
      const auto sensed = plant.step();
      if (!sensed) continue;
      const State s = state(*sensed);
      if (s.time >= next_ref) {
        const auto point = target_at(s.time);
        ref.position = point.position;
        ref.velocity = point.velocity;
        ref.acceleration = point.acceleration;
        next_ref = s.time + 1 / command_config.publish_hz;
      }
      out = controller.update(s, ref);
      const auto input = allocator.update(out.force, out.torque, s.servo, out.com, 1 / m.state_hz);
      plant.setInput(input.speed, input.angle);
      const auto truth = plant.truth();
      check(truth.position.allFinite() && truth.omega.allFinite(), "Finite closed-loop state");
      check(truth.position.norm() < 50 && truth.rpy.head<2>().norm() < 1.0,
            "Closed-loop flight stability");
      max_angle = std::max(max_angle, truth.servo.cwiseAbs().maxCoeff());
      if (s.time > 12) {
        position_squared += (truth.position - target_at(truth.time).position).squaredNorm();
        attitude_squared += truth.rpy.squaredNorm();
        ++count;
      }
      if (csv && static_cast<int>(s.time * m.state_hz) % 8 == 0) {
        csv << s.time;
        for (const auto& v : {truth.position, ref.position, truth.rpy, out.com, out.disturbance})
          for (int k = 0; k < 3; ++k) csv << ',' << v[k];
        csv << '\n';
      }
    }
    const double pos_rmse = std::sqrt(position_squared / std::max(count, 1)),
                 att_rmse = std::sqrt(attitude_squared / std::max(count, 1));
    std::cout << "duration=" << duration << " position_rmse_m=" << pos_rmse
              << " attitude_rmse_rad=" << att_rmse << " max_servo_rad=" << max_angle
              << " com_estimate=" << out.com.transpose() << " configured_com=" << m.com.transpose()
              << " dob=" << out.disturbance.transpose() << '\n';
    if (com_regression) {
      check((out.com.head<2>() - m.com.head<2>()).norm() < 0.003,
            "MOCE horizontal CoM convergence in native flight");
      // The plant retains four articulated rotor masses while MOCE uses the
      // configured fixed diagonal rigid-body inertia. Tilt-dependent inertia
      // appears mainly as a bounded effective vertical-CoM bias.
      if (c.get<bool>("moce.estimate_z"))
        check(std::abs(out.com.z() - m.com.z()) < 0.012,
              "MOCE vertical effective-CoM convergence in native flight");
      else
        check(std::abs(out.com.z() - m.initial_com.z()) < 1e-12,
              "Disabled vertical MOCE remains at its initial estimate");
    }
    check(pos_rmse < 0.7, "Closed-loop position RMSE with configured thrust loss");
    check(att_rmse < 0.15, "Closed-loop attitude RMSE");
    check(max_angle < m.servo_limit + 0.05, "Physical servo limits");
    for (int i = 0; i < mjNWARNING; ++i)
      check(plant.data()->warning[i].number == 0, "MuJoCo numerical warning");
    std::cout << "PASS: model parameters, independent delays, servo dynamics/mesh, native "
                 "closed-loop flight\n";
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
