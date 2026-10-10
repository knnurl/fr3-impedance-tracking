cmake_minimum_required(VERSION 3.8)
project(fr3_tracking)

# Release unless told otherwise. colcon passes no build type, and an empty one
# means no optimisation: an unoptimised build missed the 1 kHz FCI deadline in
# HOLD and TRACK (communication_constraints_violation reflexes).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (default Release)" FORCE)
endif()

# Marker/vision tracking on top of the impedance controller: the tracking law
# (header-only, tested without a robot), tracking_node that streams the
# controller's equilibrium from it, and a read-only 1 kHz robot-state recorder.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(franka_msgs REQUIRED)

add_executable(tracking_node src/tracking_node.cpp)
target_include_directories(tracking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tracking_node PUBLIC cxx_std_17)
ament_target_dependencies(tracking_node
  rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs franka_msgs lifecycle_msgs
  controller_manager_msgs diagnostic_msgs)

add_executable(state_recorder src/state_recorder.cpp)
target_compile_features(state_recorder PUBLIC cxx_std_17)
ament_target_dependencies(state_recorder rclcpp franka_msgs)

install(TARGETS tracking_node state_recorder
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../.clang-format")
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../setup.cfg")
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_mating_geometry test/test_mating_geometry.cpp)
  target_include_directories(test_mating_geometry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_mating_geometry tf2)

  ament_add_gtest(test_tracking_law test/test_tracking_law.cpp)
  target_include_directories(test_tracking_law PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_tracking_law tf2)
endif()

ament_package()
