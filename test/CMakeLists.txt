find_package(ament_cmake_gtest REQUIRED)

# StatusModel is ROS-free by construction -- this target deliberately links no rclcpp/mrs_lib.
ament_add_gtest(test_status_model status/test_status_model.cpp "${PROJECT_SOURCE_DIR}/src/status/status_model.cpp")
target_include_directories(test_status_model PRIVATE "${PROJECT_SOURCE_DIR}/include")

add_test(
  NAME check_tui_ros_free
  COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/check_tui_ros_free.sh" "${PROJECT_SOURCE_DIR}"
)
