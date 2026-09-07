cmake_minimum_required(VERSION 3.8)
project(palletrone_interfaces)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(msg_files
  "msg/Input.msg"
  "msg/PalletroneState.msg"
  "msg/Cmd.msg"
  "msg/Wrench.msg"
  "msg/ControlStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

add_executable(csv_logger src/csv_logger.cpp)
ament_target_dependencies(csv_logger rclcpp ament_index_cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(csv_logger "${cpp_typesupport_target}")

install(TARGETS csv_logger DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY bag DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(Eigen3 yaml-cpp)
ament_package()
