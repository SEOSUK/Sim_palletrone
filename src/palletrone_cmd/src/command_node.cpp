#include <ament_index_cpp/get_package_share_directory.hpp>
#include <palletrone_interfaces/command.hpp>
#include <palletrone_interfaces/msg/cmd.hpp>
#include <palletrone_interfaces/msg/palletrone_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <optional>

using palletrone::CommandConfig;
using palletrone::CommandGenerator;
using palletrone::Vec3;

class TerminalInput {
 public:
  TerminalInput() {
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &original_) != 0) return;
    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0 &&
        fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK) == 0) {
      enabled_ = true;
    }
  }
  ~TerminalInput() {
    if (enabled_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_);
      fcntl(STDIN_FILENO, F_SETFL, original_flags_);
    }
  }
  bool enabled() const { return enabled_; }
  std::optional<char> read() const {
    char key = 0;
    return enabled_ && ::read(STDIN_FILENO, &key, 1) == 1 ? std::optional<char>(key) : std::nullopt;
  }

 private:
  termios original_{};
  int original_flags_ = 0;
  bool enabled_ = false;
};

class CommandNode : public rclcpp::Node {
 public:
  CommandNode() : Node("command_node") {
    const auto config_dir =
        ament_index_cpp::get_package_share_directory("palletrone_interfaces") + "/config/";
    const auto path = declare_parameter("command_config", config_dir + "command.yaml");
    const auto control_path = declare_parameter("control_config", config_dir + "control.yaml");
    config_ = std::make_unique<CommandConfig>(palletrone::Config(path));
    const palletrone::Config control(control_path);
    const bool initial_moce = control.get<bool>("moce.enabled");
    const bool initial_moce_z = control.get<bool>("moce.estimate_z");
    generator_ = std::make_unique<CommandGenerator>(*config_, initial_moce, initial_moce_z);

    command_pub_ = create_publisher<palletrone_interfaces::msg::Cmd>("/cmd", 10);
    state_sub_ = create_subscription<palletrone_interfaces::msg::PalletroneState>(
        "/palletrone_state", 10,
        [this](palletrone_interfaces::msg::PalletroneState::ConstSharedPtr msg) {
          if (!std::isfinite(msg->sample_time)) return;
          simulation_time_ = msg->sample_time;
          measured_position_ = Vec3(msg->pos[0], msg->pos[1], msg->pos[2]);
          measured_rpy_ = Vec3(msg->rpy[0], msg->rpy[1], msg->rpy[2]);
          have_state_ = measured_position_.allFinite() && measured_rpy_.allFinite();
        });
    // This topic provides the same one-character input when stdin is unavailable
    // (launch services, tests, or a remote terminal).
    key_sub_ = create_subscription<std_msgs::msg::String>(
        "/command/key", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
          if (msg->data.size() == 1) handleKey(msg->data[0]);
        });

    publish_timer_ = create_wall_timer(
        period(config_->publish_hz), [this]() { publishCommand(); });
    keyboard_timer_ = create_wall_timer(period(config_->keyboard_poll_hz), [this]() {
      while (const auto key = terminal_.read()) handleKey(*key);
    });

    RCLCPP_INFO(get_logger(), "Keyboard command config: %s", path.c_str());
    RCLCPP_INFO(get_logger(), "Initial MOCE state from control config: %s",
                initial_moce ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(),
                "W/S: +/-X, A/D: +/-Y, E/Q: +/-Z, Z/C: yaw, G: Lissajous, M: experiment, "
                "P: MOCE on/off, H: hold measured pose, X: reset, T: quit");
    if (!terminal_.enabled()) {
      RCLCPP_WARN(get_logger(),
                  "stdin is not a TTY. Publish one character on /command/key. "
                  "For keyboard control, launch with command:=false and run "
                  "`ros2 run palletrone_cmd command_node` in a separate terminal.");
    }
  }

 private:
  static std::chrono::nanoseconds period(double hz) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz));
  }
  void handleKey(char key) {
    if (std::tolower(static_cast<unsigned char>(key)) ==
            config_->keys.at("automated_experiment") &&
        !have_state_) {
      RCLCPP_WARN(get_logger(), "M ignored until the first vehicle state is received");
      return;
    }
    if (generator_->handle(key, simulation_time_, measured_position_, measured_rpy_)) {
      publishCommand();
      printStatus(key);
      if (generator_->quitRequested()) rclcpp::shutdown();
    }
  }
  void publishCommand() {
    const auto reference = generator_->at(simulation_time_);
    palletrone_interfaces::msg::Cmd msg;
    for (int i = 0; i < 3; ++i) {
      msg.pos_cmd[i] = reference.position[i];
      msg.vel_cmd[i] = reference.velocity[i];
      msg.acc_cmd[i] = reference.acceleration[i];
      msg.rpy_cmd[i] = reference.rpy[i];
      msg.rate_cmd[i] = reference.rate[i];
    }
    msg.moce_enabled = generator_->moceEnabled();
    msg.moce_z_enabled = generator_->moceZEnabled();
    command_pub_->publish(msg);
    last_reference_ = reference.position;
  }
  void printStatus(char key) {
    RCLCPP_INFO(get_logger(), "%c: %s | mode=%s MOCE=%s cmd=[%.2f %.2f %.2f] state=%s",
                key, generator_->status().c_str(),
                generator_->automatedExperimentActive()
                    ? "EXPERIMENT"
                    : (generator_->trajectoryActive() ? "LISSAJOUS" : "MANUAL"),
                generator_->moceEnabled() ? "ON" : "OFF", last_reference_.x(), last_reference_.y(),
                last_reference_.z(),
                have_state_ ? "ready" : "waiting");
  }

  std::unique_ptr<CommandConfig> config_;
  std::unique_ptr<CommandGenerator> generator_;
  TerminalInput terminal_;
  double simulation_time_ = 0;
  Vec3 measured_position_ = Vec3::Zero(), measured_rpy_ = Vec3::Zero(), last_reference_ = Vec3::Zero();
  bool have_state_ = false;
  rclcpp::Publisher<palletrone_interfaces::msg::Cmd>::SharedPtr command_pub_;
  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_, keyboard_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CommandNode>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("command_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
