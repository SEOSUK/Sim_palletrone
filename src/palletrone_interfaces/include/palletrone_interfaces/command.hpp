#pragma once

#include <palletrone_interfaces/config.hpp>

#include <array>
#include <cctype>
#include <map>

namespace palletrone {
struct CommandReference {
  Vec3 position = Vec3::Zero();
  Vec3 velocity = Vec3::Zero();
  Vec3 acceleration = Vec3::Zero();
  Vec3 rpy = Vec3::Zero();
  Vec3 rate = Vec3::Zero();
};

struct CommandConfig {
  double publish_hz, keyboard_poll_hz, yaw_tick, duration;
  double experiment_ascent_duration, experiment_hover_duration, experiment_moce_start,
      experiment_lissajous_duration;
  Vec3 startup, startup_rpy, tick, minimum, maximum, center, amplitude, frequency, phase,
      experiment_hover_position;
  bool manual_cancels, repeat;
  std::string finish_behavior;
  std::map<std::string, char> keys;

  explicit CommandConfig(const Config& c) {
    publish_hz = c.scalar("command.publish_hz", 0, true);
    keyboard_poll_hz = c.scalar("command.keyboard_poll_hz", 0, true);
    startup = c.vector("command.startup_position");
    startup_rpy = c.vector("command.startup_rpy");
    tick = c.vector("command.position_tick", true);
    yaw_tick = c.scalar("command.yaw_tick_rad", 0, true);
    minimum = c.vector("command.position_min");
    maximum = c.vector("command.position_max");
    manual_cancels = c.get<bool>("command.manual_cancels_trajectory");
    center = c.vector("command.lissajous.center");
    amplitude = c.vector("command.lissajous.amplitude", true);
    frequency = c.vector("command.lissajous.frequency_hz", true);
    phase = c.vector("command.lissajous.phase_rad");
    repeat = c.get<bool>("command.lissajous.repeat");
    duration = c.scalar("command.lissajous.duration_s", 0, true);
    finish_behavior = c.get<std::string>("command.lissajous.finish_behavior");
    experiment_hover_position = c.vector("command.automated_experiment.hover_position");
    experiment_ascent_duration =
        c.scalar("command.automated_experiment.ascent_duration_s", 0, true);
    experiment_hover_duration =
        c.scalar("command.automated_experiment.hover_duration_s", 0, true);
    experiment_moce_start = c.scalar("command.automated_experiment.moce_start_s");
    experiment_lissajous_duration =
        c.scalar("command.automated_experiment.lissajous_duration_s", 0, true);
    if ((minimum.array() >= maximum.array()).any() ||
        (startup.array() < minimum.array()).any() || (startup.array() > maximum.array()).any()) {
      throw std::runtime_error("Invalid command position bounds/startup position");
    }
    if (finish_behavior != "hold" && finish_behavior != "reset") {
      throw std::runtime_error("command.lissajous.finish_behavior must be hold or reset");
    }
    if ((experiment_hover_position.array() < minimum.array()).any() ||
        (experiment_hover_position.array() > maximum.array()).any() ||
        experiment_moce_start > experiment_hover_duration) {
      throw std::runtime_error("Invalid automated experiment hover position/timing");
    }
    static const std::array<const char*, 14> names = {
        "forward", "backward", "left", "right", "up", "down", "yaw_left", "yaw_right",
        "trajectory_toggle", "automated_experiment", "moce_toggle", "hold_current_pose", "reset",
        "quit"};
    std::array<bool, 256> used{};
    for (const char* name : names) {
      const auto value = c.get<std::string>(std::string("command.keys.") + name);
      if (value.size() != 1) throw std::runtime_error(std::string("Command key must be one character: ") + name);
      const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(value[0])));
      if (used[static_cast<unsigned char>(key)]) throw std::runtime_error("Duplicate command key");
      used[static_cast<unsigned char>(key)] = true;
      keys[name] = key;
    }
  }
};

class CommandGenerator {
 public:
  explicit CommandGenerator(CommandConfig config, bool initial_moce = false,
                            bool initial_moce_z = false)
      : config_(std::move(config)), initial_moce_(initial_moce), initial_moce_z_(initial_moce_z) {
    reset();
  }

  void reset() {
    manual_position_ = config_.startup;
    manual_rpy_ = config_.startup_rpy;
    trajectory_ = false;
    automated_experiment_ = false;
    moce_enabled_ = initial_moce_;
    moce_z_enabled_ = initial_moce_z_;
  }
  bool trajectoryActive() const { return trajectory_; }
  bool automatedExperimentActive() const { return automated_experiment_; }
  bool quitRequested() const { return quit_; }
  bool moceEnabled() const { return moce_enabled_; }
  bool moceZEnabled() const { return moce_z_enabled_; }
  const std::string& status() const { return status_; }

  bool handle(char input, double time, const Vec3& measured_position, const Vec3& measured_rpy) {
    const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(input)));
    if (key == config_.keys.at("quit")) {
      quit_ = true;
      status_ = "quit requested";
      return true;
    }
    if (key == config_.keys.at("trajectory_toggle")) {
      if (automated_experiment_) {
        status_ = "G ignored during automated experiment";
        return true;
      }
      if (trajectory_) {
        const auto current = at(time);
        manual_position_ = current.position;
        manual_rpy_ = current.rpy;
        trajectory_ = false;
        status_ = "Lissajous stopped; holding last command";
      } else {
        trajectory_start_ = time;
        trajectory_ = true;
        status_ = "Lissajous started";
      }
      return true;
    }
    if (key == config_.keys.at("automated_experiment")) {
      experiment_start_position_ =
          measured_position.cwiseMax(config_.minimum).cwiseMin(config_.maximum);
      experiment_start_rpy_ = measured_rpy;
      experiment_start_ = time;
      trajectory_ = false;
      automated_experiment_ = true;
      moce_enabled_ = false;
      moce_z_enabled_ = false;
      status_ = "automated experiment started: ascending to hover";
      return true;
    }
    if (key == config_.keys.at("moce_toggle")) {
      moce_enabled_ = !moce_enabled_;
      status_ = moce_enabled_ ? "MOCE enabled" : "MOCE disabled";
      return true;
    }
    if (key == config_.keys.at("hold_current_pose")) {
      trajectory_ = false;
      automated_experiment_ = false;
      manual_position_ = measured_position.cwiseMax(config_.minimum).cwiseMin(config_.maximum);
      manual_rpy_ = measured_rpy;
      status_ = "holding measured pose";
      return true;
    }
    if (key == config_.keys.at("reset")) {
      reset();
      status_ = "command reset";
      return true;
    }
    Vec3 delta = Vec3::Zero();
    double yaw = 0;
    if (key == config_.keys.at("forward")) delta.x() = config_.tick.x();
    else if (key == config_.keys.at("backward")) delta.x() = -config_.tick.x();
    else if (key == config_.keys.at("left")) delta.y() = config_.tick.y();
    else if (key == config_.keys.at("right")) delta.y() = -config_.tick.y();
    else if (key == config_.keys.at("up")) delta.z() = config_.tick.z();
    else if (key == config_.keys.at("down")) delta.z() = -config_.tick.z();
    else if (key == config_.keys.at("yaw_left")) yaw = config_.yaw_tick;
    else if (key == config_.keys.at("yaw_right")) yaw = -config_.yaw_tick;
    else return false;
    if (automated_experiment_) {
      status_ = "manual input ignored during automated experiment; H or X cancels it";
      return true;
    }
    if (trajectory_) {
      const auto current = at(time);
      if (trajectory_ && !config_.manual_cancels) {
        status_ = "manual input ignored during Lissajous; stop the trajectory to move manually";
        return true;
      }
      manual_position_ = current.position;
      manual_rpy_ = current.rpy;
      trajectory_ = false;
    }
    manual_position_ = (manual_position_ + delta).cwiseMax(config_.minimum).cwiseMin(config_.maximum);
    manual_rpy_.z() = std::atan2(std::sin(manual_rpy_.z() + yaw), std::cos(manual_rpy_.z() + yaw));
    status_ = "manual command updated";
    return true;
  }

  CommandReference at(double time) {
    if (automated_experiment_) return automatedExperiment(time);
    CommandReference output;
    output.position = manual_position_;
    output.rpy = manual_rpy_;
    if (!trajectory_) return output;
    double elapsed = std::max(0.0, time - trajectory_start_);
    if (!config_.repeat && elapsed >= config_.duration) {
      const auto finish = lissajous(config_.duration);
      trajectory_ = false;
      manual_position_ = config_.finish_behavior == "reset" ? config_.startup : finish.position;
      output.position = manual_position_;
      status_ = "Lissajous complete";
      return output;
    }
    return lissajous(elapsed);
  }

 private:
  CommandReference automatedExperiment(double time) {
    // The M-key sequence is an MOCE-off baseline experiment.
    moce_enabled_ = false;
    moce_z_enabled_ = false;
    const double elapsed = std::max(0.0, time - experiment_start_);
    if (elapsed < config_.experiment_ascent_duration) {
      const double u = elapsed / config_.experiment_ascent_duration;
      const double blend = u * u * u * (10 + u * (-15 + 6 * u));
      const double blend_d =
          30 * u * u * (1 - 2 * u + u * u) / config_.experiment_ascent_duration;
      const double blend_dd = 60 * u * (1 - 3 * u + 2 * u * u) /
                              (config_.experiment_ascent_duration * config_.experiment_ascent_duration);
      CommandReference output;
      const Vec3 delta = config_.experiment_hover_position - experiment_start_position_;
      output.position = experiment_start_position_ + blend * delta;
      output.velocity = blend_d * delta;
      output.acceleration = blend_dd * delta;
      output.rpy = experiment_start_rpy_;
      return output;
    }
    const double hover_elapsed = elapsed - config_.experiment_ascent_duration;
    if (hover_elapsed < config_.experiment_hover_duration) {
      CommandReference output;
      output.position = config_.experiment_hover_position;
      output.rpy = experiment_start_rpy_;
      return output;
    }
    const double curve_elapsed = hover_elapsed - config_.experiment_hover_duration;
    if (curve_elapsed < config_.experiment_lissajous_duration) {
      return lissajous(curve_elapsed);
    }
    automated_experiment_ = false;
    manual_position_ = config_.experiment_hover_position;
    manual_rpy_ = experiment_start_rpy_;
    status_ = "automated experiment complete; hovering";
    CommandReference output;
    output.position = manual_position_;
    output.rpy = manual_rpy_;
    return output;
  }

  CommandReference lissajous(double elapsed) const {
    CommandReference output;
    output.position = config_.center;
    output.rpy = manual_rpy_;
    for (int i = 0; i < 3; ++i) {
      const double omega = 2.0 * M_PI * config_.frequency[i];
      const double phase = omega * elapsed + config_.phase[i];
      output.position[i] += config_.amplitude[i] * std::sin(phase);
      output.velocity[i] = config_.amplitude[i] * omega * std::cos(phase);
      output.acceleration[i] = -config_.amplitude[i] * omega * omega * std::sin(phase);
    }
    return output;
  }

  CommandConfig config_;
  Vec3 manual_position_ = Vec3::Zero(), manual_rpy_ = Vec3::Zero();
  Vec3 experiment_start_position_ = Vec3::Zero(), experiment_start_rpy_ = Vec3::Zero();
  double trajectory_start_ = 0, experiment_start_ = 0;
  bool initial_moce_ = false, initial_moce_z_ = false;
  bool trajectory_ = false, automated_experiment_ = false, quit_ = false, moce_enabled_ = true,
       moce_z_enabled_ = false;
  std::string status_ = "ready";
};
}  // namespace palletrone
