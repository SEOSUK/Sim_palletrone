cmake_minimum_required(VERSION 3.16)
project(plant)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(palletrone_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)
find_package(mujoco CONFIG QUIET)
if(NOT TARGET mujoco::mujoco)
  set(MUJOCO_ROOT "" CACHE PATH "MuJoCo SDK or installed wheel directory")
  if(NOT MUJOCO_ROOT)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import pathlib,mujoco; print(pathlib.Path(mujoco.__file__).parent)"
      OUTPUT_VARIABLE MUJOCO_ROOT OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE MUJOCO_DISCOVERY)
    if(NOT MUJOCO_DISCOVERY EQUAL 0)
      message(FATAL_ERROR "Set MUJOCO_ROOT to a MuJoCo SDK. Python is used only to locate an installed wheel at build time.")
    endif()
  endif()
  find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS ${MUJOCO_ROOT}/include REQUIRED)
  file(GLOB MUJOCO_LIBRARIES "${MUJOCO_ROOT}/libmujoco.so*" "${MUJOCO_ROOT}/lib/libmujoco.so*")
  list(SORT MUJOCO_LIBRARIES COMPARE NATURAL ORDER DESCENDING)
  if(NOT MUJOCO_LIBRARIES)
    message(FATAL_ERROR "No libmujoco.so found below MUJOCO_ROOT=${MUJOCO_ROOT}")
  endif()
  list(GET MUJOCO_LIBRARIES 0 MUJOCO_LIBRARY)
  add_library(mujoco::mujoco SHARED IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES IMPORTED_LOCATION "${MUJOCO_LIBRARY}" INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
  get_filename_component(MUJOCO_LIBRARY_DIR "${MUJOCO_LIBRARY}" DIRECTORY)
  set(CMAKE_INSTALL_RPATH "${MUJOCO_LIBRARY_DIR}")
endif()
add_library(palletrone_plant src/model.cpp)
target_include_directories(palletrone_plant PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(palletrone_plant palletrone_interfaces)
target_link_libraries(palletrone_plant Eigen3::Eigen yaml-cpp mujoco::mujoco)
add_executable(plant src/plant.cpp)
ament_target_dependencies(plant rclcpp ament_index_cpp palletrone_interfaces)
target_link_libraries(plant palletrone_plant glfw OpenGL::GL Threads::Threads)
install(TARGETS plant DESTINATION lib/${PROJECT_NAME})
install(TARGETS palletrone_plant DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY xml DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(palletrone_controller REQUIRED)
  enable_testing()
  add_executable(simulation_test test/simulation_test.cpp)
  ament_target_dependencies(simulation_test palletrone_controller palletrone_interfaces)
  target_link_libraries(simulation_test palletrone_plant)
  add_executable(effectiveness_test test/effectiveness_test.cpp)
  ament_target_dependencies(effectiveness_test palletrone_interfaces)
  target_link_libraries(effectiveness_test palletrone_plant)
  add_executable(drift_validation test/drift_validation.cpp)
  ament_target_dependencies(drift_validation palletrone_controller palletrone_interfaces)
  target_link_libraries(drift_validation palletrone_plant)
  add_executable(noise_test test/noise_test.cpp)
  ament_target_dependencies(noise_test palletrone_interfaces)
  target_link_libraries(noise_test palletrone_plant)
  add_executable(residual_torque_test test/residual_torque_test.cpp)
  ament_target_dependencies(residual_torque_test palletrone_interfaces)
  target_link_libraries(residual_torque_test palletrone_plant)
  add_test(NAME colored_residual_torque COMMAND residual_torque_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/control.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/model_identified_v4_torque.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/model_identified_v3_measurement.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/xml/scene.xml)
  add_executable(residual_force_test test/residual_force_test.cpp)
  ament_target_dependencies(residual_force_test palletrone_interfaces)
  target_link_libraries(residual_force_test palletrone_plant)
  add_test(NAME colored_residual_force COMMAND residual_force_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/control.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/model.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/model_identified_v4_torque.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/xml/scene.xml)
  add_test(NAME measurement_uncertainty COMMAND noise_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/control.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/model.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/xml/scene.xml)
  add_test(NAME thrust_effectiveness_drift COMMAND effectiveness_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/control.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/model.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/control_identified_v1.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/model_identified_v1.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/xml/scene.xml)
  add_test(NAME identified_v1_automated_flight COMMAND drift_validation
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/control_identified_v1.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/model_identified_v1.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/command.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/xml/scene.xml v1)
  add_test(NAME identified_v2_drift_automated_flight COMMAND drift_validation
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/control.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/model_identified_v2_drift.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/command.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/xml/scene.xml v2_drift)
  add_test(NAME identified_v3_measurement_automated_flight COMMAND drift_validation
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/control.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/model_identified_v3_measurement.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/command.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/xml/scene.xml v3_measurement)
  add_test(NAME identified_v4_colored_torque_automated_flight COMMAND drift_validation
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/control.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/model_identified_v4_torque.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/command.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/xml/scene.xml v4_colored_torque)
  add_test(NAME identified_v5_colored_force_automated_flight COMMAND drift_validation
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/control.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/model.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/command.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/xml/scene.xml v5_colored_force)
  add_test(NAME native_moce_convergence COMMAND simulation_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/control_identified_v1.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/model_identified_v1.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../palletrone_interfaces/config/command.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/xml/scene.xml 600 - com)
endif()
ament_package()
