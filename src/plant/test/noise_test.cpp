#include <iostream>
#include <plant/model.hpp>

using namespace palletrone;

void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

struct Stats {
  int count = 0;
  Vec3 sum = Vec3::Zero(), sum2 = Vec3::Zero();
  void add(const Vec3& value) {
    ++count;
    sum += value;
    sum2 += value.cwiseAbs2();
  }
  Vec3 stddev() const {
    const Vec3 mean = sum / count;
    return (sum2 / count - mean.cwiseAbs2()).cwiseSqrt();
  }
};

int main(int argc, char** argv) {
  try {
    if (argc != 4) throw std::runtime_error("noise_test CONTROL MODEL_V3 SCENE_XML");
    Config control(argv[1]);
    ModelConfig noisy_config{Config(argv[2])};
    const std::string scene = argv[3];

    ModelConfig zero_config = noisy_config;
    zero_config.pos_noise.setZero();
    zero_config.vel_noise.setZero();
    zero_config.att_noise.setZero();
    zero_config.gyro_noise.setZero();
    zero_config.servo_noise = 0;
    PlantModel zero(scene, control, zero_config);
    for (int i = 0; i < 100; ++i) {
      const auto sample = zero.step();
      check(sample.has_value(), "Zero-noise sample available");
      const auto truth = zero.truth();
      check((sample->position - truth.position).norm() == 0 &&
                (sample->velocity - truth.velocity).norm() == 0 &&
                (sample->rpy - truth.rpy).norm() == 0 &&
                (sample->omega - truth.omega).norm() == 0 &&
                (sample->servo - truth.servo).norm() == 0,
            "Zero std preserves deterministic measured state");
    }

    PlantModel same_a(scene, control, noisy_config), same_b(scene, control, noisy_config);
    ModelConfig other_seed = noisy_config;
    ++other_seed.seed;
    PlantModel different(scene, control, other_seed);
    bool seed_difference = false;
    for (int i = 0; i < 100; ++i) {
      const auto a = same_a.step(), b = same_b.step(), c = different.step();
      check(a && b && c, "Seed test samples available");
      check((a->position - b->position).norm() == 0 && (a->velocity - b->velocity).norm() == 0 &&
                (a->rpy - b->rpy).norm() == 0 && (a->omega - b->omega).norm() == 0 &&
                (a->servo - b->servo).norm() == 0,
            "Fixed seed is exactly reproducible");
      seed_difference |= (a->position - c->position).norm() > 0 ||
                         (a->velocity - c->velocity).norm() > 0 ||
                         (a->rpy - c->rpy).norm() > 0 || (a->omega - c->omega).norm() > 0;
    }
    check(seed_difference, "Different seeds produce different samples");

    PlantModel statistics(scene, control, noisy_config);
    Stats position, velocity, attitude, gyro;
    double servo_sum = 0, servo_sum2 = 0;
    constexpr int samples = 50000;
    for (int i = 0; i < samples; ++i) {
      const auto measured = statistics.step();
      check(measured.has_value(), "Noise statistics sample available");
      const auto truth = statistics.truth();
      position.add(measured->position - truth.position);
      velocity.add(measured->velocity - truth.velocity);
      attitude.add(measured->rpy - truth.rpy);
      gyro.add(measured->omega - truth.omega);
      const double servo_noise = measured->servo[0] - truth.servo[0];
      servo_sum += servo_noise;
      servo_sum2 += servo_noise * servo_noise;
    }
    auto matches = [](const Vec3& measured, const Vec3& target) {
      return ((measured - target).cwiseAbs().array() <= 0.03 * target.array()).all();
    };
    check(matches(position.stddev(), noisy_config.pos_noise), "Position noise axis/std mapping");
    check(matches(velocity.stddev(), noisy_config.vel_noise), "Velocity noise axis/std mapping");
    check(matches(attitude.stddev(), noisy_config.att_noise), "Attitude noise axis/std mapping");
    check(matches(gyro.stddev(), noisy_config.gyro_noise), "Gyro noise axis/std mapping");
    const double servo_mean = servo_sum / samples;
    const double servo_std = std::sqrt(servo_sum2 / samples - servo_mean * servo_mean);
    check(std::abs(servo_std - noisy_config.servo_noise) <= 0.04 * noisy_config.servo_noise,
          "Existing servo noise behavior");

    ModelConfig delayed_config = zero_config;
    delayed_config.sensor_delay = 0.0125;
    PlantModel delayed(scene, control, delayed_config);
    bool received = false;
    for (int i = 0; i < 20; ++i) {
      const auto sample = delayed.step();
      if (sample) {
        received = true;
        check(std::abs(delayed.data()->time - sample->time - delayed_config.sensor_delay) <
                  1.0 / delayed_config.physics_hz + 1e-9,
              "Existing sensor delay behavior");
      }
    }
    check(received, "Delayed sensor sample released");
    std::cout << "PASS: zero noise, seed reproducibility/divergence, axis-wise noise std, servo "
                 "noise, and sensor delay\n";
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
