cmake_minimum_required(VERSION 3.5)
project(lslidar_n301_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(lslidar_n301_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

# Node
add_executable(lslidar_n301_driver_node src/lslidar_n301_driver_node.cc src/lslidar_n301_driver.cc)
target_link_libraries(lslidar_n301_driver_node ${rclcpp_LIBRARIES})
ament_target_dependencies(lslidar_n301_driver_node rclcpp lslidar_n301_msgs diagnostic_updater)

include_directories(
		include
)

install(TARGETS
  lslidar_n301_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp lslidar_n301_msgs)
ament_export_include_directories(include)

ament_package()

