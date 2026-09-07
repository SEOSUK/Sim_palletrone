#pragma once
#include <mujoco/mujoco.h>

#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <palletrone_interfaces/config.hpp>
#include <random>

namespace palletrone {
template <typename T>
class DelayLine {
 public:
  explicit DelayLine(T initial) : held_(initial) {}
  void push(double time, const T& value) { queue_.emplace_back(time, value); }
  T sample(double time) {
    while (!queue_.empty() && queue_.front().first <= time + 1e-10) {
      held_ = queue_.front().second;
      queue_.pop_front();
    }
    return held_;
  }
  void clear(const T& value) {
    queue_.clear();
    held_ = value;
  }

 private:
  std::deque<std::pair<double, T>> queue_;
  T held_;
};
struct PlantSample {
  double time = 0;
  Vec3 position = Vec3::Zero(), velocity = Vec3::Zero(), acceleration = Vec3::Zero();
  Vec3 rpy = Vec3::Zero(), omega = Vec3::Zero(), angular_acceleration = Vec3::Zero();
  Vec4 servo = Vec4::Zero();
};
class PlantModel {
 public:
  PlantModel(const std::string& scene, const Config& control, const ModelConfig& config);
  void setInput(const Vec4& speed, const Vec4& angle);
  std::optional<PlantSample> step();
  PlantSample truth() const;
  mjModel* model() const { return model_.get(); }
  mjData* data() const { return data_.get(); }
  const ModelConfig& config() const { return config_; }

 private:
  using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
  using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
  ModelConfig config_;
  ModelPtr model_{nullptr, mj_deleteModel};
  DataPtr data_{nullptr, mj_deleteData};
  int base_ = 0, pos_sensor_ = 0, vel_sensor_ = 0, gyro_sensor_ = 0;
  std::array<int, 4> joint_{}, motor_{}, servo_{};
  DelayLine<Vec4> motor_delay_{Vec4::Zero()}, servo_delay_{Vec4::Zero()};
  Vec4 thrust_scale_ = Vec4::Ones(), motor_force_ = Vec4::Zero();
  std::deque<PlantSample> sensor_delay_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_{0, 1};
  double last_input_ = 0, next_sample_ = 0;
  PlantSample previous_noisy_;
  bool have_noisy_ = false;
  Vec3 sensor3(int id) const;
  Vec3 noisy(const Vec3& x, const Vec3& sigma);
};
}  // namespace palletrone
