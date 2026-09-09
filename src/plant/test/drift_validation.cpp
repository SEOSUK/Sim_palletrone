#include <fstream>
#include <iomanip>
#include <iostream>
#include <palletrone_controller/control.hpp>
#include <palletrone_interfaces/command.hpp>
#include <plant/model.hpp>

using namespace palletrone;

struct Accumulator {
  double pos2 = 0, att2 = 0, peak_att = 0, angacc2 = 0, dob = 0;
  Vec3 attitude_axis2 = Vec3::Zero(), dob_axis2 = Vec3::Zero();
  int count = 0;
  void add(const Vec3& position_error, const Vec3& attitude_error, const Vec3& torque,
           const Vec3& disturbance, const Vec3& inertia) {
    const double att = attitude_error.norm();
    pos2 += position_error.squaredNorm();
    att2 += att * att;
    attitude_axis2 += attitude_error.cwiseAbs2();
    peak_att = std::max(peak_att, att);
    angacc2 += torque.cwiseQuotient(inertia).squaredNorm();
    dob += disturbance.cwiseQuotient(inertia).norm();
    dob_axis2 += disturbance.cwiseQuotient(inertia).cwiseAbs2();
    ++count;
  }
  void print(const char* name) const {
    std::cout << name << ',' << std::sqrt(pos2 / count) << ',' << std::sqrt(att2 / count) << ','
              << peak_att << ',' << std::sqrt(angacc2 / count) << ',' << dob / count << '\n';
  }
  void printAxes(const char* name) const {
    std::cout << name << ",attitude_axis_rms," <<
        (attitude_axis2 / count).cwiseSqrt().transpose() << ",dob_axis_rms," <<
        (dob_axis2 / count).cwiseSqrt().transpose() << '\n';
  }
};

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
    if (argc < 6)
      throw std::runtime_error(
          "drift_validation CONTROL MODEL COMMAND SCENE LABEL [OUTPUT_CSV] "
          "[MEASUREMENT_SEED [TORQUE_SEED FORCE_SEED]]");
    Config control(argv[1]);
    ModelConfig model{Config(argv[2])};
    if (argc > 7) model.seed = std::stoi(argv[7]);
    // Keep the historical single-seed CLI compatible while allowing Monte Carlo
    // trials to drive the three independent RNG streams separately.
    model.residual_seed = argc > 8 ? std::stoi(argv[8]) : model.seed;
    model.residual_force_seed = argc > 9 ? std::stoi(argv[9]) : model.seed;
    CommandConfig command_config{Config(argv[3])};
    PlantModel plant(argv[4], control, model);
    CascadeController controller(control, model);
    Allocator allocator(control, model);
    CommandGenerator command(command_config);
    const PlantSample initial = plant.truth();
    command.handle(command_config.keys.at("automated_experiment"), initial.time, initial.position,
                   initial.rpy);

    std::ofstream csv;
    if (argc > 6) {
      csv.open(argv[6]);
      csv << "time,layout.data_offset";
      for (int i = 0; i < 124; ++i) csv << ",data[" << i << ']';
      csv << '\n';
      csv << std::setprecision(17);
    }

    Reference reference;
    ControlOutput output;
    double next_command = 0, moce_start = -1;
    bool previous_moce = false;
    Accumulator overall, transient1, transient2, complete;
    while (plant.data()->time < 96.0) {
      const auto sampled = plant.step();
      if (!sampled) continue;
      const State s = state(*sampled);
      if (!s.position.allFinite() || !s.rpy.allFinite() || s.position.norm() >= 50)
        throw std::runtime_error("Nonfinite or divergent automated flight state");
      if (s.time + 1e-10 >= next_command) {
        const auto point = command.at(s.time);
        reference.position = point.position;
        reference.velocity = point.velocity;
        reference.acceleration = point.acceleration;
        reference.rpy = point.rpy;
        reference.rate = point.rate;
        controller.setMoceEnabled(command.moceEnabled());
        controller.setMoceZEnabled(command.moceZEnabled());
        if (!previous_moce && command.moceEnabled()) moce_start = s.time;
        previous_moce = command.moceEnabled();
        next_command += 1.0 / command_config.publish_hz;
      }
      output = controller.update(s, reference);
      const auto input =
          allocator.update(output.force, output.torque, s.servo, output.com, 1.0 / model.state_hz);
      plant.setInput(input.speed, input.angle);

      const double teval = moce_start >= 0 ? s.time - (moce_start - 5.0) : -1.0;
      Vec3 attitude_error = s.rpy - reference.rpy;
      attitude_error.z() = std::atan2(std::sin(attitude_error.z()), std::cos(attitude_error.z()));
      const Vec3 position_error = s.position - reference.position;
      if (moce_start >= 0 && teval >= 0 && teval <= 80)
        overall.add(position_error, attitude_error, output.torque, output.disturbance, model.inertia);
      if (teval >= 5 && teval <= 60)
        transient1.add(position_error, attitude_error, output.torque, output.disturbance,
                       model.inertia);
      if (teval >= 20 && teval <= 60)
        transient2.add(position_error, attitude_error, output.torque, output.disturbance,
                       model.inertia);
      if (teval >= 60 && teval <= 80)
        complete.add(position_error, attitude_error, output.torque, output.disturbance,
                     model.inertia);
      if (csv) {
        std::array<double, 124> data{};
        for (int i = 0; i < 3; ++i) {
          data[i] = s.position[i];
          data[i + 3] = reference.position[i];
          data[i + 6] = s.velocity[i];
          data[i + 9] = reference.velocity[i];
          data[i + 12] = s.rpy[i];
          data[i + 15] = reference.rpy[i];
          data[i + 18] = s.omega[i];
          data[i + 21] = output.rate_sp[i];
          data[i + 24] = output.force[i];
          data[i + 27] = output.torque[i];
          data[i + 31] = output.disturbance[i];
          data[i + 42] = sampled->acceleration[i];
          data[i + 45] = reference.acceleration[i];
          data[i + 48] = sampled->angular_acceleration[i];
          data[i + 51] = output.pid_torque[i];
          data[i + 54] = output.com[i];
          data[i + 57] = output.com_filtered[i];
          data[i + 60] = output.com_rate[i];
        }
        for (int i = 0; i < 4; ++i) {
          data[i + 34] = input.speed[i];
          data[i + 38] = input.angle[i];
        }
        data[63] = output.adapting;
        data[64] = command.moceEnabled();
        data[65] = command.moceZEnabled();
        data[93] = sampled->common_effectiveness;
        for (int i = 0; i < 4; ++i) data[94 + i] = sampled->actual_thrust_scale[i];
        for (int i = 0; i < 3; ++i) {
          data[98 + i] = sampled->truth_acceleration[i];
          data[101 + i] = sampled->truth_angular_acceleration[i];
          data[104 + i] = sampled->ou_torque_body[i];
          data[107 + i] = sampled->total_external_torque_body[i];
          data[110 + i] = sampled->ou_force_body[i];
          data[113 + i] = sampled->total_external_force_body[i];
        }
        for (int i = 0; i < 4; ++i) {
          data[116 + i] = sampled->motor_force[i];
          data[120 + i] = sampled->servo[i];
        }
        csv << static_cast<int64_t>(std::llround(s.time * 1e9)) << ",0";
        for (double value : data) csv << ',' << value;
        csv << '\n';
      }
    }
    if (!overall.count || !complete.count) throw std::runtime_error("Incomplete metric windows");
    if (!std::isfinite(overall.pos2) || !std::isfinite(overall.att2) ||
        !output.com.allFinite())
      throw std::runtime_error("Nonfinite validation metrics");
    std::cout << "label,window,position_rms,attitude_rms,peak_attitude,angacc_cmd_rms,mean_dob\n";
    std::cout << argv[5] << ',';
    overall.print("overall_0_80");
    std::cout << argv[5] << ',';
    transient1.print("transient1_5_60");
    std::cout << argv[5] << ',';
    transient2.print("transient2_20_60");
    std::cout << argv[5] << ',';
    complete.print("complete_60_80");
    complete.printAxes("complete_60_80");
    std::cout << "final_com," << output.com.transpose() << ",true_com," << model.com.transpose()
              << ",final_com_error," << (output.com - model.com).norm() << ",final_eta,"
              << plant.commonEffectiveness() << '\n';
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
#include <array>
