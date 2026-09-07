#include <fstream>
#include <iomanip>
#include <iostream>
#include <palletrone_controller/control.hpp>
#include <palletrone_interfaces/command.hpp>
#include <plant/model.hpp>

using namespace palletrone;

struct Accumulator {
  double pos2 = 0, att2 = 0, peak_att = 0, angacc2 = 0, dob = 0;
  int count = 0;
  void add(const Vec3& position_error, const Vec3& attitude_error, const Vec3& torque,
           const Vec3& disturbance, const Vec3& inertia) {
    const double att = attitude_error.norm();
    pos2 += position_error.squaredNorm();
    att2 += att * att;
    peak_att = std::max(peak_att, att);
    angacc2 += torque.cwiseQuotient(inertia).squaredNorm();
    dob += disturbance.cwiseQuotient(inertia).norm();
    ++count;
  }
  void print(const char* name) const {
    std::cout << name << ',' << std::sqrt(pos2 / count) << ',' << std::sqrt(att2 / count) << ','
              << peak_att << ',' << std::sqrt(angacc2 / count) << ',' << dob / count << '\n';
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
          "drift_validation CONTROL MODEL COMMAND SCENE LABEL [OUTPUT_CSV]");
    Config control(argv[1]);
    ModelConfig model{Config(argv[2])};
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
      csv << "time,teval,pos_error_norm,att_error_norm,angacc_cmd_norm,dob_accel_norm,com_x,"
             "com_y,com_z,eta_common,scale_1,scale_2,scale_3,scale_4,moce_enabled,moce_z_enabled\n";
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

      if (moce_start < 0) continue;
      const double teval = s.time - (moce_start - 5.0);
      Vec3 attitude_error = s.rpy - reference.rpy;
      attitude_error.z() = std::atan2(std::sin(attitude_error.z()), std::cos(attitude_error.z()));
      const Vec3 position_error = s.position - reference.position;
      if (teval >= 0 && teval <= 80)
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
        csv << s.time << ',' << teval << ',' << position_error.norm() << ','
            << attitude_error.norm() << ',' << output.torque.cwiseQuotient(model.inertia).norm()
            << ',' << output.disturbance.cwiseQuotient(model.inertia).norm() << ','
            << output.com.x() << ',' << output.com.y() << ',' << output.com.z() << ','
            << sampled->common_effectiveness;
        for (int i = 0; i < 4; ++i) csv << ',' << sampled->actual_thrust_scale[i];
        csv << ',' << command.moceEnabled() << ',' << command.moceZEnabled() << '\n';
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
    std::cout << "final_com," << output.com.transpose() << " final_eta,"
              << plant.commonEffectiveness() << '\n';
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
