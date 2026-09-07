#include <ament_index_cpp/get_package_share_directory.hpp>
#include <palletrone_controller/control.hpp>
#include <palletrone_interfaces/msg/cmd.hpp>
#include <palletrone_interfaces/msg/control_status.hpp>
#include <palletrone_interfaces/msg/palletrone_state.hpp>
#include <palletrone_interfaces/msg/wrench.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace palletrone;
template <typename A>
Vec3 vec(const A& a) {
  return Vec3(a[0], a[1], a[2]);
}
template <typename A>
void assign(A& a, const Vec3& v) {
  for (int i = 0; i < 3; ++i) a[i] = v[i];
}
class WrenchController : public rclcpp::Node {
 public:
  WrenchController() : Node("wrench_controller") {
    const auto dir =
        ament_index_cpp::get_package_share_directory("palletrone_interfaces") + "/config/";
    Config control(declare_parameter("control_config", dir + "control.yaml"));
    ModelConfig model(Config(declare_parameter("model_config", dir + "model.yaml")));
    controller_ = std::make_unique<CascadeController>(control, model);
    pub_ = create_publisher<palletrone_interfaces::msg::Wrench>("/wrench", 10);
    status_pub_ =
        create_publisher<palletrone_interfaces::msg::ControlStatus>("/control_status", 10);
    cmd_sub_ = create_subscription<palletrone_interfaces::msg::Cmd>(
        "/cmd", 10, [this](palletrone_interfaces::msg::Cmd::ConstSharedPtr msg) {
          Reference r;
          r.position = vec(msg->pos_cmd);
          r.velocity = vec(msg->vel_cmd);
          r.acceleration = vec(msg->acc_cmd);
          r.rpy = vec(msg->rpy_cmd);
          r.rate = vec(msg->rate_cmd);
          if (r.finite()) {
            ref_ = r;
            controller_->setMoceEnabled(msg->moce_enabled);
            controller_->setMoceZEnabled(msg->moce_z_enabled);
            have_cmd_ = true;
          }
        });
    state_sub_ = create_subscription<palletrone_interfaces::msg::PalletroneState>(
        "/palletrone_state", 10,
        [this](palletrone_interfaces::msg::PalletroneState::ConstSharedPtr msg) { onState(*msg); });
    RCLCPP_INFO(get_logger(),
                "Position/velocity + attitude/rate PID cascades, MOCE 2.0 and torque DOB ready");
  }

 private:
  void onState(const palletrone_interfaces::msg::PalletroneState& msg) {
    if (!have_cmd_) return;
    State s;
    s.time = msg.sample_time;
    s.position = vec(msg.pos);
    s.velocity = vec(msg.vel);
    s.rpy = vec(msg.rpy);
    s.omega = vec(msg.w_rpy);
    for (int i = 0; i < 4; ++i) s.servo[i] = msg.servo[i];
    if (!s.finite() || (have_time_ && s.time == last_time_)) return;
    last_time_ = s.time;
    have_time_ = true;
    const auto out = controller_->update(s, ref_);
    palletrone_interfaces::msg::Wrench w;
    w.sample_time = s.time;
    assign(w.force, out.force);
    assign(w.moment, out.torque);
    assign(w.com, out.com);
    pub_->publish(w);
    palletrone_interfaces::msg::ControlStatus log;
    log.sample_time = s.time;
    assign(log.velocity_setpoint, out.velocity_sp);
    assign(log.rate_setpoint, out.rate_sp);
    assign(log.rate_pid_torque, out.pid_torque);
    assign(log.disturbance_torque, out.disturbance);
    assign(log.compensated_torque, out.torque);
    assign(log.com_estimate, out.com);
    assign(log.com_filtered, out.com_filtered);
    assign(log.com_rate, out.com_rate);
    log.adaptation_active = out.adapting;
    status_pub_->publish(log);
  }
  std::unique_ptr<CascadeController> controller_;
  Reference ref_;
  bool have_cmd_ = false, have_time_ = false;
  double last_time_ = 0;
  rclcpp::Subscription<palletrone_interfaces::msg::Cmd>::SharedPtr cmd_sub_;
  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr state_sub_;
  rclcpp::Publisher<palletrone_interfaces::msg::Wrench>::SharedPtr pub_;
  rclcpp::Publisher<palletrone_interfaces::msg::ControlStatus>::SharedPtr status_pub_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<WrenchController>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("wrench_controller"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
