cmake_minimum_required(VERSION 3.8)
project(palletrone_controller)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(include)
find_package(palletrone_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# wrench_controller
add_executable(wrench_controller src/wrench_controller.cpp)
ament_target_dependencies(wrench_controller rclcpp ament_index_cpp palletrone_interfaces)
target_link_libraries(wrench_controller Eigen3::Eigen yaml-cpp)

# allocator_controller
add_executable(allocator_controller src/allocator_controller.cpp)
ament_target_dependencies(allocator_controller rclcpp ament_index_cpp palletrone_interfaces)
target_link_libraries(allocator_controller Eigen3::Eigen yaml-cpp)

install(TARGETS
  wrench_controller
  allocator_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(palletrone_interfaces Eigen3 yaml-cpp)
if(BUILD_TESTING)
  enable_testing()
  add_executable(control_test test/control_test.cpp)
  ament_target_dependencies(control_test palletrone_interfaces)
  target_link_libraries(control_test Eigen3::Eigen yaml-cpp)
  add_test(NAME control_algorithms COMMAND control_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/control.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/model.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/command.yaml)
endif()
ament_package()
