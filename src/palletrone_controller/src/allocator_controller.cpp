#include <ament_index_cpp/get_package_share_directory.hpp>
#include <palletrone_controller/control.hpp>
#include <palletrone_interfaces/msg/input.hpp>
#include <palletrone_interfaces/msg/palletrone_state.hpp>
#include <palletrone_interfaces/msg/wrench.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace palletrone;
class AllocatorController : public rclcpp::Node {
 public:
  AllocatorController() : Node("allocator_controller") {
    const auto dir =
        ament_index_cpp::get_package_share_directory("palletrone_interfaces") + "/config/";
    Config control(declare_parameter("control_config", dir + "control.yaml"));
    ModelConfig model(Config(declare_parameter("model_config", dir + "model.yaml")));
    allocator_ = std::make_unique<Allocator>(control, model);
    default_dt_ = 1 / model.state_hz;
    pub_ = create_publisher<palletrone_interfaces::msg::Input>("/input", 10);
    state_sub_ = create_subscription<palletrone_interfaces::msg::PalletroneState>(
        "/palletrone_state", 10,
        [this](palletrone_interfaces::msg::PalletroneState::ConstSharedPtr s) {
          Vec4 v;
          for (int i = 0; i < 4; ++i) v[i] = s->servo[i];
          if (v.allFinite()) measured_ = v;
        });
    wrench_sub_ = create_subscription<palletrone_interfaces::msg::Wrench>(
        "/wrench", 10, [this](palletrone_interfaces::msg::Wrench::ConstSharedPtr w) {
          Vec3 f(w->force[0], w->force[1], w->force[2]),
              t(w->moment[0], w->moment[1], w->moment[2]), c(w->com[0], w->com[1], w->com[2]);
          if (!f.allFinite() || !t.allFinite() || !c.allFinite() || !std::isfinite(w->sample_time))
            return;
          const double dt = w->sample_time > last_time_ ? w->sample_time - last_time_ : default_dt_;
          last_time_ = w->sample_time;
          const auto out = allocator_->update(f, t, measured_, c, dt);
          palletrone_interfaces::msg::Input msg;
          for (int i = 0; i < 4; ++i) {
            msg.u[i] = out.speed[i];
            msg.u[i + 4] = out.angle[i];
          }
          pub_->publish(msg);
        });
  }

 private:
  std::unique_ptr<Allocator> allocator_;
  Vec4 measured_ = Vec4::Zero();
  double last_time_ = 0, default_dt_ = 0;
  rclcpp::Publisher<palletrone_interfaces::msg::Input>::SharedPtr pub_;
  rclcpp::Subscription<palletrone_interfaces::msg::Wrench>::SharedPtr wrench_sub_;
  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr state_sub_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AllocatorController>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("allocator_controller"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
