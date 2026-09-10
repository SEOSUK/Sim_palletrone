#include <GLFW/glfw3.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <mutex>
#include <palletrone_interfaces/msg/input.hpp>
#include <palletrone_interfaces/msg/palletrone_state.hpp>
#include <plant/model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace palletrone;
class Plant : public rclcpp::Node {
 public:
  Plant() : Node("plant") {
    const auto config_dir =
        ament_index_cpp::get_package_share_directory("palletrone_interfaces") + "/config/";
    Config control(declare_parameter("control_config", config_dir + "control.yaml"));
    ModelConfig model(Config(declare_parameter("model_config", config_dir + "model.yaml")));
    RCLCPP_INFO(get_logger(), "Configured vehicle/payload/total mass %.3f/%.3f/%.3f kg, CoM [%.4f %.4f %.4f]",
                model.vehicle_mass, model.payload_mass, model.mass, model.com.x(), model.com.y(),
                model.com.z());
    const auto scene = declare_parameter(
        "scene", ament_index_cpp::get_package_share_directory("plant") + "/xml/scene.xml");
    model_ = std::make_unique<PlantModel>(scene, control, model);
    state_pub_ =
        create_publisher<palletrone_interfaces::msg::PalletroneState>("/palletrone_state", 10);
    truth_pub_ =
        create_publisher<palletrone_interfaces::msg::PalletroneState>("/palletrone_truth", 10);
    input_sub_ = create_subscription<palletrone_interfaces::msg::Input>(
        "/input", 10, [this](palletrone_interfaces::msg::Input::ConstSharedPtr input) {
          Vec4 speed, angle;
          for (int i = 0; i < 4; ++i) {
            speed[i] = input->u[i];
            angle[i] = input->u[i + 4];
          }
          if (!speed.allFinite() || !angle.allFinite()) {
            RCLCPP_WARN(get_logger(), "Rejected nonfinite actuator input");
            return;
          }
          std::lock_guard<std::mutex> guard(mutex_);
          model_->setInput(speed, angle);
        });
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1 / (model.physics_hz * model.realtime_factor))),
        [this]() {
          std::lock_guard<std::mutex> guard(mutex_);
          const auto s = model_->step();
          if (s) {
            state_pub_->publish(message(*s));
            truth_pub_->publish(message(model_->truth()));
          }
        });
    if (declare_parameter("viewer", true)) viewer_ = std::thread([this]() { viewerLoop(); });
    const auto thrust_scale = control.vector4("motor.thrust_scale", true);
    RCLCPP_INFO(get_logger(),
                "Native MuJoCo %s: mass %.3f kg, physics %.1f Hz, sensor delay %.4f s, motor/servo "
                "delay %.4f/%.4f s, servo +/-%.3f rad, rotor thrust scale [%.3f %.3f %.3f %.3f]",
                mj_versionString(), mj_getTotalmass(model_->model()), model.physics_hz,
                model.sensor_delay, model.motor_delay, model.servo_delay, model.servo_limit,
                thrust_scale[0], thrust_scale[1], thrust_scale[2], thrust_scale[3]);
  }
  ~Plant() override {
    stop_ = true;
    timer_.reset();
    if (viewer_.joinable()) viewer_.join();
  }

 private:
  static palletrone_interfaces::msg::PalletroneState message(const PlantSample& s) {
    palletrone_interfaces::msg::PalletroneState msg;
    msg.sample_time = s.time;
    for (int i = 0; i < 3; ++i) {
      msg.pos[i] = s.position[i];
      msg.vel[i] = s.velocity[i];
      msg.acc[i] = s.acceleration[i];
      msg.rpy[i] = s.rpy[i];
      msg.w_rpy[i] = s.omega[i];
      msg.a_rpy[i] = s.angular_acceleration[i];
      msg.truth_acc[i] = s.truth_acceleration[i];
      msg.truth_a_rpy[i] = s.truth_angular_acceleration[i];
      msg.ou_torque_body[i] = s.ou_torque_body[i];
      msg.total_external_torque_body[i] = s.total_external_torque_body[i];
      msg.ou_force_body[i] = s.ou_force_body[i];
      msg.total_external_force_body[i] = s.total_external_force_body[i];
    }
    for (int i = 0; i < 4; ++i) msg.servo[i] = s.servo[i];
    msg.common_effectiveness = s.common_effectiveness;
    for (int i = 0; i < 4; ++i) msg.actual_thrust_scale[i] = s.actual_thrust_scale[i];
    for (int i = 0; i < 4; ++i) msg.motor_force[i] = s.motor_force[i];
    return msg;
  }
  struct View {
    mjModel* model;
    mjvCamera camera{};
    mjvScene scene{};
    double x = 0, y = 0;
  };
  void viewerLoop() {
    if (!glfwInit()) {
      RCLCPP_WARN(get_logger(), "GLFW unavailable; continuing headless");
      return;
    }
    GLFWwindow* window = glfwCreateWindow(1280, 800, "Palletrone | C++ MuJoCo", nullptr, nullptr);
    if (!window) {
      glfwTerminate();
      RCLCPP_WARN(get_logger(), "Viewer window unavailable");
      return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    View view;
    view.model = model_->model();
    mjv_defaultCamera(&view.camera);
    mjv_defaultScene(&view.scene);
    mjvOption option;
    mjv_defaultOption(&option);
    mjrContext context;
    mjr_defaultContext(&context);
    mjv_makeScene(view.model, &view.scene, 2000);
    mjr_makeContext(view.model, &context, mjFONTSCALE_150);
    view.camera.type = mjCAMERA_FREE;
    view.camera.lookat[0] = 0.0;
    view.camera.lookat[1] = 0.0;
    view.camera.lookat[2] = 0.7;
    // Start with a wide, nearly level front view similar to the lab camera.
    view.camera.distance = 3.6;
    view.camera.azimuth = 0;
    view.camera.elevation = -8;
    glfwSetWindowUserPointer(window, &view);
    glfwSetScrollCallback(window, [](GLFWwindow* w, double, double dy) {
      auto* v = static_cast<View*>(glfwGetWindowUserPointer(w));
      mjv_moveCamera(v->model, mjMOUSE_ZOOM, 0, -0.05 * dy, &v->scene, &v->camera);
    });
    glfwSetCursorPosCallback(window, [](GLFWwindow* w, double x, double y) {
      auto* v = static_cast<View*>(glfwGetWindowUserPointer(w));
      int width, height;
      glfwGetWindowSize(w, &width, &height);
      const bool left = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
      const bool right = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
      if (left || right)
        mjv_moveCamera(v->model, right ? mjMOUSE_MOVE_V : mjMOUSE_ROTATE_V,
                       (x - v->x) / std::max(height, 1), (y - v->y) / std::max(height, 1),
                       &v->scene, &v->camera);
      v->x = x;
      v->y = y;
    });
    RCLCPP_INFO(get_logger(), "GLFW viewer ready (native MuJoCo renderer)");
    while (rclcpp::ok() && !stop_ && !glfwWindowShouldClose(window)) {
      const auto next = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(1 / model_->config().viewer_hz);
      {
        std::lock_guard<std::mutex> guard(mutex_);
        mjv_updateScene(view.model, model_->data(), &option, nullptr, &view.camera, mjCAT_ALL,
                        &view.scene);
      }
      mjrRect viewport{0, 0, 0, 0};
      glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
      mjr_render(viewport, &view.scene, &context);
      glfwSwapBuffers(window);
      glfwPollEvents();
      if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(window, 1);
      std::this_thread::sleep_until(next);
    }
    mjr_freeContext(&context);
    mjv_freeScene(&view.scene);
    glfwDestroyWindow(window);
    glfwTerminate();
    if (!stop_ && rclcpp::ok()) rclcpp::shutdown();
  }
  std::unique_ptr<PlantModel> model_;
  std::mutex mutex_;
  std::atomic<bool> stop_{false};
  std::thread viewer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<palletrone_interfaces::msg::Input>::SharedPtr input_sub_;
  rclcpp::Publisher<palletrone_interfaces::msg::PalletroneState>::SharedPtr state_pub_, truth_pub_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Plant>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("plant"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
