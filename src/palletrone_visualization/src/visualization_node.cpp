#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <palletrone_interfaces/config.hpp>
#include <palletrone_interfaces/msg/cmd.hpp>
#include <palletrone_interfaces/msg/control_status.hpp>
#include <palletrone_interfaces/msg/palletrone_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using palletrone::Config;
using palletrone::Mat3;
using palletrone::ModelConfig;
using palletrone::Vec3;

class VisualizationNode : public rclcpp::Node {
 public:
  VisualizationNode() : Node("palletrone_visualization") {
    const auto config_dir =
        ament_index_cpp::get_package_share_directory("palletrone_interfaces") + "/config/";
    const auto command_path = declare_parameter("command_config", config_dir + "command.yaml");
    const auto model_path = declare_parameter("model_config", config_dir + "model.yaml");
    Config command(command_path);
    model_ = std::make_unique<ModelConfig>(Config(model_path));
    publish_hz_ = command.scalar("visualization.publish_hz", 0, true);
    world_frame_ = command.get<std::string>("visualization.frames.world");
    pose_frame_ = command.get<std::string>("visualization.frames.pose");
    base_frame_ = command.get<std::string>("visualization.frames.base");
    command_frame_ = command.get<std::string>("visualization.frames.command");
    estimated_topic_ = command.get<std::string>("visualization.marker.estimated_com_topic");
    true_topic_ = command.get<std::string>("visualization.marker.true_com_topic");
    payload_topic_ = command.get<std::string>("visualization.marker.payload_topic");
    com_distance_scale_ = command.scalar("visualization.marker.com_distance_scale", 0, true);
    estimated_scale_ = command.scalar("visualization.marker.estimated_scale", 0, true);
    true_scale_ = command.scalar("visualization.marker.true_scale", 0, true);
    payload_dimensions_ = command.vector("visualization.marker.payload_dimensions", true);
    if ((payload_dimensions_.array() <= 0).any())
      throw std::runtime_error("visualization.marker.payload_dimensions must be positive");
    estimated_color_ = color(command, "visualization.marker.estimated_rgba");
    true_color_ = color(command, "visualization.marker.true_rgba");
    payload_color_ = color(command, "visualization.marker.payload_rgba");
    calculateNeutralBaseCom();

    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    estimated_pub_ = create_publisher<visualization_msgs::msg::Marker>(estimated_topic_, 10);
    true_pub_ = create_publisher<visualization_msgs::msg::Marker>(true_topic_, 10);
    payload_pub_ = create_publisher<visualization_msgs::msg::Marker>(payload_topic_, 10);
    state_sub_ = create_subscription<palletrone_interfaces::msg::PalletroneState>(
        "/palletrone_state", 10,
        [this](palletrone_interfaces::msg::PalletroneState::ConstSharedPtr msg) {
          state_ = *msg;
          have_state_ = true;
        });
    truth_sub_ = create_subscription<palletrone_interfaces::msg::PalletroneState>(
        "/palletrone_truth", 10,
        [this](palletrone_interfaces::msg::PalletroneState::ConstSharedPtr msg) {
          truth_ = *msg;
          have_truth_ = true;
        });
    command_sub_ = create_subscription<palletrone_interfaces::msg::Cmd>(
        "/cmd", 10, [this](palletrone_interfaces::msg::Cmd::ConstSharedPtr msg) {
          command_ = *msg;
          have_command_ = true;
        });
    status_sub_ = create_subscription<palletrone_interfaces::msg::ControlStatus>(
        "/control_status", 10,
        [this](palletrone_interfaces::msg::ControlStatus::ConstSharedPtr msg) {
          estimated_com_ = Vec3(msg->com_estimate[0], msg->com_estimate[1], msg->com_estimate[2]);
          have_estimate_ = estimated_com_.allFinite();
        });
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_hz_)),
        [this]() { publish(); });
    RCLCPP_INFO(get_logger(), "RViz TF/model/CoM visualization ready");
  }

 private:
  static std::array<float, 4> color(const Config& config, const std::string& key) {
    const auto node = config.at(key);
    if (!node.IsSequence() || node.size() != 4) throw std::runtime_error(key + " must contain RGBA");
    std::array<float, 4> result{};
    for (int i = 0; i < 4; ++i) {
      result[i] = node[i].as<float>();
      if (!std::isfinite(result[i]) || result[i] < 0 || result[i] > 1)
        throw std::runtime_error(key + " values must be within [0,1]");
    }
    return result;
  }
  static Eigen::Quaterniond quaternion(const Vec3& rpy) {
    return Eigen::Quaterniond(palletrone::rotation(rpy));
  }
  static geometry_msgs::msg::TransformStamped transform(const rclcpp::Time& stamp,
                                                         const std::string& parent,
                                                         const std::string& child, const Vec3& position,
                                                         const Vec3& rpy) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.transform.translation.x = position.x();
    tf.transform.translation.y = position.y();
    tf.transform.translation.z = position.z();
    const Eigen::Quaterniond q = quaternion(rpy);
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    return tf;
  }
  visualization_msgs::msg::Marker marker(const rclcpp::Time& stamp, const std::string& name,
                                          const Vec3& position, double scale,
                                          const std::array<float, 4>& rgba) const {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = base_frame_;
    marker.ns = "palletrone_com";
    marker.id = name == "estimated_com" ? 0 : (name == "true_com" ? 1 : 2);
    marker.type = name == "payload" ? visualization_msgs::msg::Marker::CUBE
                                    : visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1;
    if (name == "payload") {
      marker.scale.x = payload_dimensions_.x();
      marker.scale.y = payload_dimensions_.y();
      marker.scale.z = payload_dimensions_.z();
    } else {
      marker.scale.x = marker.scale.y = marker.scale.z = scale;
    }
    marker.color.r = rgba[0];
    marker.color.g = rgba[1];
    marker.color.b = rgba[2];
    marker.color.a = rgba[3];
    return marker;
  }
  void calculateNeutralBaseCom() {
    Vec3 rotor_moment = Vec3::Zero();
    for (int i = 0; i < 4; ++i) rotor_moment += model_->rotor_mass * (model_->rotorPosition(i) + model_->rotor_com);
    base_com_ = (model_->mass * model_->com - rotor_moment) /
                (model_->mass - 4.0 * model_->rotor_mass);
  }
  Vec3 trueCom() const {
    Vec3 weighted = (model_->mass - 4.0 * model_->rotor_mass) * base_com_;
    const double yaw[4] = {M_PI / 4, 3 * M_PI / 4, -3 * M_PI / 4, -M_PI / 4};
    for (int i = 0; i < 4; ++i) {
      const Mat3 rotor_rotation =
          Eigen::AngleAxisd(yaw[i], Vec3::UnitZ()).toRotationMatrix() *
          Eigen::AngleAxisd(truth_.servo[i], Vec3::UnitX()).toRotationMatrix();
      weighted += model_->rotor_mass * (model_->rotorPosition(i) + rotor_rotation * model_->rotor_com);
    }
    return weighted / model_->mass;
  }
  void publish() {
    if (!have_state_) return;
    const auto stamp = now();
    const Vec3 position(state_.pos[0], state_.pos[1], state_.pos[2]);
    const Vec3 rpy(state_.rpy[0], state_.rpy[1], state_.rpy[2]);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(transform(stamp, world_frame_, pose_frame_, position, rpy));
    transforms.push_back(transform(stamp, pose_frame_, base_frame_, Vec3::Zero(), Vec3::Zero()));
    if (have_command_) {
      transforms.push_back(transform(stamp, world_frame_, command_frame_,
                                     Vec3(command_.pos_cmd[0], command_.pos_cmd[1], command_.pos_cmd[2]),
                                     Vec3(command_.rpy_cmd[0], command_.rpy_cmd[1], command_.rpy_cmd[2])));
    }
    tf_->sendTransform(transforms);

    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    joints.name = {"j_servo1", "j_servo2", "j_servo3", "j_servo4"};
    joints.position.assign(state_.servo.begin(), state_.servo.end());
    joints_pub_->publish(joints);
    if (have_estimate_) estimated_pub_->publish(marker(stamp, "estimated_com", com_distance_scale_ * estimated_com_, estimated_scale_, estimated_color_));
    if (have_truth_) true_pub_->publish(marker(stamp, "true_com", com_distance_scale_ * trueCom(), true_scale_, true_color_));
    payload_pub_->publish(marker(stamp, "payload", model_->payload_position, 0, payload_color_));
  }

  std::unique_ptr<ModelConfig> model_;
  Vec3 base_com_ = Vec3::Zero(), estimated_com_ = Vec3::Zero(), payload_dimensions_ = Vec3::Ones();
  double publish_hz_ = 30, estimated_scale_ = 0.08, true_scale_ = 0.06;
  double com_distance_scale_ = 10;
  std::string world_frame_, pose_frame_, base_frame_, command_frame_, estimated_topic_, true_topic_,
      payload_topic_;
  std::array<float, 4> estimated_color_{}, true_color_{}, payload_color_{};
  palletrone_interfaces::msg::PalletroneState state_;
  palletrone_interfaces::msg::PalletroneState truth_;
  palletrone_interfaces::msg::Cmd command_;
  bool have_state_ = false, have_truth_ = false, have_command_ = false, have_estimate_ = false;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr estimated_pub_, true_pub_, payload_pub_;
  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr state_sub_;
  rclcpp::Subscription<palletrone_interfaces::msg::PalletroneState>::SharedPtr truth_sub_;
  rclcpp::Subscription<palletrone_interfaces::msg::Cmd>::SharedPtr command_sub_;
  rclcpp::Subscription<palletrone_interfaces::msg::ControlStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VisualizationNode>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("palletrone_visualization"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
