cmake_minimum_required(VERSION 3.8)
project(camera_fusion_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(camera_fusion_cpp_node src/camera_fusion_cpp_node.cpp)
ament_target_dependencies(camera_fusion_cpp_node rclcpp sensor_msgs cv_bridge OpenCV message_filters ament_index_cpp std_srvs)
target_link_libraries(camera_fusion_cpp_node yaml-cpp)

add_executable(camera_reader_cpp_node src/camera_reader_cpp_node.cpp)
ament_target_dependencies(camera_reader_cpp_node rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
  camera_fusion_cpp_node
  camera_reader_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
