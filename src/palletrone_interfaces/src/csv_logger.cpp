#include <ament_index_cpp/get_package_share_directory.hpp>
#include <palletrone_interfaces/msg/cmd.hpp>
#include <palletrone_interfaces/msg/control_status.hpp>
#include <palletrone_interfaces/msg/input.hpp>
#include <palletrone_interfaces/msg/palletrone_state.hpp>
#include <palletrone_interfaces/msg/wrench.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

class CsvLogger : public rclcpp::Node {
 public:
  CsvLogger() : Node("csv_logger") {
    const auto share_dir = ament_index_cpp::get_package_share_directory("palletrone_interfaces");
    const auto command_config = declare_parameter("command_config", share_dir + "/config/command.yaml");
    auto output_dir = declare_parameter("output_directory", std::string());
    if (output_dir.empty()) {
      // With --symlink-install, canonical() resolves command.yaml back to the source package.
      output_dir = (std::filesystem::canonical(command_config).parent_path().parent_path() / "bag").string();
    }
    std::filesystem::create_directories(output_dir);
    path_ = (std::filesystem::path(output_dir) / fileName()).string();
    const bool add_header = !std::filesystem::exists(path_) || std::filesystem::file_size(path_) == 0;
    csv_.open(path_, std::ios::app);
    if (!csv_) throw std::runtime_error("Could not open CSV log: " + path_);
    if (add_header) writeHeader();
    csv_ << std::setprecision(17);

    state_sub_ = create_subscription<palletrone_interfaces::msg::PalletroneState>(
        "/palletrone_state", 20,
        [this](palletrone_interfaces::msg::PalletroneState::ConstSharedPtr msg) {
          state_ = *msg;
          have_state_ = true;
        });
    command_sub_ = create_subscription<palletrone_interfaces::msg::Cmd>(
        "/cmd", 20, [this](palletrone_interfaces::msg::Cmd::ConstSharedPtr msg) {
          command_ = *msg;
          have_command_ = true;
        });
    wrench_sub_ = create_subscription<palletrone_interfaces::msg::Wrench>(
        "/wrench", 20, [this](palletrone_interfaces::msg::Wrench::ConstSharedPtr msg) {
          wrench_ = *msg;
          have_wrench_ = true;
        });
    input_sub_ = create_subscription<palletrone_interfaces::msg::Input>(
        "/input", 20, [this](palletrone_interfaces::msg::Input::ConstSharedPtr msg) {
          input_ = *msg;
          have_input_ = true;
        });
    status_sub_ = create_subscription<palletrone_interfaces::msg::ControlStatus>(
        "/control_status", 20,
        [this](palletrone_interfaces::msg::ControlStatus::ConstSharedPtr msg) { write(*msg); });
    RCLCPP_INFO(get_logger(), "Writing MATLAB-compatible flight log: %s", path_.c_str());
  }

 private:
  static std::string fileName() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
    localtime_r(&time, &local);
    std::ostringstream name;
    name << std::put_time(&local, "%m%d%H%M.csv");
    return name.str();
  }

  void writeHeader() {
    csv_ << "time,layout.data_offset";
    for (int i = 0; i < 124; ++i) csv_ << ",data[" << i << ']';
    csv_ << '\n';
  }

  void write(const palletrone_interfaces::msg::ControlStatus& status) {
    if (!have_state_ || !have_command_ || !have_wrench_ || !have_input_ ||
        !std::isfinite(status.sample_time)) {
      return;
    }
    if (!have_start_time_) {
      start_time_ = status.sample_time;
      have_start_time_ = true;
    }
    const auto nanoseconds = static_cast<int64_t>(
        std::llround(std::max(0.0, status.sample_time - start_time_) * 1e9));
    std::array<double, 124> data{};

    // Match the fixed column indices used by Exp_3_logger.m.
    for (int i = 0; i < 3; ++i) {
      data[i] = state_.pos[i];                 // values(:,2:4)
      data[i + 3] = command_.pos_cmd[i];       // values(:,5:7)
      data[i + 6] = state_.vel[i];
      data[i + 9] = command_.vel_cmd[i];
      data[i + 12] = state_.rpy[i];            // values(:,14:16)
      data[i + 15] = command_.rpy_cmd[i];       // values(:,17:19)
      data[i + 18] = state_.w_rpy[i];
      data[i + 21] = status.rate_setpoint[i];
      data[i + 24] = wrench_.force[i];
      data[i + 27] = wrench_.moment[i];         // values(:,29:31)
      data[i + 31] = status.disturbance_torque[i];  // values(:,33:35)
      data[i + 57] = status.com_filtered[i];    // values(:,59:61)
    }
    data[30] = 0.0;  // Fourth legacy torque channel, retained for MATLAB layout compatibility.
    for (int i = 0; i < 8; ++i) data[i + 34] = input_.u[i];
    for (int i = 0; i < 3; ++i) {
      data[i + 42] = state_.acc[i];
      data[i + 45] = command_.acc_cmd[i];
      data[i + 48] = state_.a_rpy[i];
      data[i + 51] = status.rate_pid_torque[i];
      data[i + 54] = status.com_estimate[i];
      data[i + 60] = status.com_rate[i];
    }
    data[63] = status.adaptation_active ? 1.0 : 0.0;
    data[64] = command_.moce_enabled ? 1.0 : 0.0;
    data[65] = command_.moce_z_enabled ? 1.0 : 0.0;
    // Append-only extension: legacy data[0:92] and MATLAB indices remain unchanged.
    data[93] = state_.common_effectiveness;
    for (int i = 0; i < 4; ++i) data[94 + i] = state_.actual_thrust_scale[i];
    for (int i = 0; i < 3; ++i) {
      data[98 + i] = state_.truth_acc[i];
      data[101 + i] = state_.truth_a_rpy[i];
      data[104 + i] = state_.ou_torque_body[i];
      data[107 + i] = state_.total_external_torque_body[i];
      data[110 + i] = state_.ou_force_body[i];
      data[113 + i] = state_.total_external_force_body[i];
    }
    for (int i = 0; i < 4; ++i) {
      data[116 + i] = state_.motor_force[i];
      data[120 + i] = state_.servo[i];
    }

    csv_ << nanoseconds << ",0";
    for (double value : data) csv_ << ',' << value;
    csv_ << '\n';
    csv_.flush();
  }

  std::string path_;
  std::ofstream csv_;
  double start_time_ = 0;
  bool have_start_time_ = false, have_state_ = false, have_command_ = false,
       have_wrench_ = false, have_input_ = false;
  palletrone_interfaces::msg::PalletroneState state_;
  palletrone_interfaces::msg::Cmd command_;
  palletrone_interfaces::msg::Wrench wrench_;
  palletrone_interfaces::msg::Input input_;
  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr state_sub_;
  rclcpp::Subscription<palletrone_interfaces::msg::Cmd>::SharedPtr command_sub_;
  rclcpp::Subscription<palletrone_interfaces::msg::Wrench>::SharedPtr wrench_sub_;
  rclcpp::Subscription<palletrone_interfaces::msg::Input>::SharedPtr input_sub_;
  rclcpp::Subscription<palletrone_interfaces::msg::ControlStatus>::SharedPtr status_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CsvLogger>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("csv_logger"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
