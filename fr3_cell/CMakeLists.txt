cmake_minimum_required(VERSION 3.8)
project(fr3_cell NONE)

# Python only, run in place: the launch file, the panel and its helpers are
# installed as files into share/fr3_cell (with --symlink-install they point
# back at this source tree, so an edit needs no rebuild).

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config calib parts cell sim vision scripts setup
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../setup.cfg")
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  # The panel, grip and analysis tests (no robot; ROS on isolated domains).
  ament_add_pytest_test(fr3_cell_pytest "${CMAKE_CURRENT_SOURCE_DIR}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 900)
  # The real tracking_node in a fake cell, on DDS domain 87.
  find_program(PYTHON3 python3 REQUIRED)
  ament_add_test(tracking_smoke
    COMMAND "${PYTHON3}" "${CMAKE_CURRENT_SOURCE_DIR}/sim/tracking_smoke.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300)
endif()

ament_package()
