cmake_minimum_required(VERSION 3.12.2)
project(multivehicle_sim)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(px4_offboard_demo 
  src/RosconPx4OffboardDemo.cpp)

ament_target_dependencies(px4_offboard_demo  rclcpp  geometry_msgs px4_msgs Eigen3)
target_compile_features(px4_offboard_demo  PUBLIC c_std_99 cxx_std_17)

install(TARGETS px4_offboard_demo  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config mavros_params
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

# bridge.py / bridges.py live alongside the other scripts/ but are a small
# python LIBRARY (used by bb_robotx_dashboard's robotx_2026_sim.launch.py), so
# install them INTO the multivehicle_sim python package — they import as
# multivehicle_sim.bridge / multivehicle_sim.bridges.
install(FILES
  scripts/bridge.py
  scripts/bridges.py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

install(PROGRAMS
  scripts/ground_truth_to_mavros.py
  scripts/bluerov_movement.py
  scripts/gz_pose_to_odom.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Update GZ_SIM_RESOURCE_PATH
# Reference: https://colcon.readthedocs.io/en/released/developer/environment.html#dsv-files
ament_environment_hooks(hooks/setup.dsv)

ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE models worlds)

