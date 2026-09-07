#include <cmath>
#include <iostream>
#include <plant/model.hpp>

using namespace palletrone;

void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
  try {
    if (argc != 6)
      throw std::runtime_error(
          "effectiveness_test CONTROL_V2 MODEL_V2 CONTROL_V1 MODEL_V1 SCENE_XML");
    const std::string scene = argv[5];
    ModelConfig v2_model{Config(argv[2])};
    PlantModel v2(scene, Config(argv[1]), v2_model);

    check(std::abs(v2.commonEffectiveness() - 0.6966) < 1e-12,
          "Initial common effectiveness");
    for (int i = 0; i < static_cast<int>(2 * v2_model.physics_hz); ++i) v2.step();
    check(std::abs(v2.commonEffectiveness() - 0.6966) < 1e-12,
          "Idle viewer time does not age effectiveness");
    v2.setInput(Vec4::Ones(), Vec4::Zero());
    check(std::abs(v2.commonEffectiveness() - 0.6966) < 1e-12,
          "First thrust command defines the effectiveness time origin");
    check(std::abs(v2.effectivenessForElapsed(60.0) - 0.64596) < 1e-12,
          "Linear effectiveness decay at 60 seconds");
    check(std::abs(v2.effectivenessForElapsed(1000.0) - 0.63) < 1e-12,
          "Minimum effectiveness clamp");

    ModelConfig rising_model = v2_model;
    rising_model.effectiveness_slope = 0.01;
    PlantModel rising(scene, Config(argv[1]), rising_model);
    check(std::abs(rising.effectivenessForElapsed(60.0) - 0.70) < 1e-12,
          "Maximum effectiveness clamp");

    ModelConfig v1_model{Config(argv[4])};
    PlantModel v1(scene, Config(argv[3]), v1_model);
    check(std::abs(v1.commonEffectiveness() - 1.0) < 1e-12,
          "Disabled common effectiveness is unity");
    check((v1.actualThrustScale() - Vec4::Constant(0.688)).norm() < 1e-12,
          "Disabled mode exactly preserves legacy static scaling");

    PlantModel relative(scene, Config(argv[3]), v2_model);
    check((relative.actualThrustScale() - Vec4::Constant(0.6966 * 0.688)).norm() < 1e-12,
          "Common and relative rotor effectiveness multiply exactly once");

    std::cout << "PASS: common thrust-effectiveness drift, time origin, clamps, legacy mode, "
                 "and relative rotor scaling\n";
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
