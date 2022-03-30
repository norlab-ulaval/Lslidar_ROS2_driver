cmake_minimum_required(VERSION 3.5)
project(lidar_c32_driver)

#add_compile_options(-std=c++11)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(libpcap_LIBRARIES -lpcap)       
set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

find_package(angles REQUIRED)
find_package(lidar_c32_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(lidar_c32_decoder REQUIRED)
find_package(PkgConfig REQUIRED)

# Node
add_executable(lidar_node src/lidar_c32_driver_node.cpp src/lidar_c32_driver.cpp src/input.cc)
target_link_libraries(lidar_node ${rclcpp_LIBRARIES} ${libpcap_LIBRARIES})
ament_target_dependencies(lidar_node rclcpp std_msgs lidar_c32_msgs sensor_msgs diagnostic_updater)

# Composition (driver + pointcloud)
add_executable(lidar_c32_executor src/lidar_c32_executor.cc src/lidar_c32_driver.cpp src/input.cc)
target_link_libraries(lidar_c32_executor ${rclcpp_LIBRARIES} ${libpcap_LIBRARIES})
ament_target_dependencies(lidar_c32_executor rclcpp std_msgs lidar_c32_msgs sensor_msgs diagnostic_updater lidar_c32_decoder)

include_directories(
		include
)

install(TARGETS
  lidar_node
  lidar_c32_executor
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp std_msgs lidar_c32_msgs sensor_msgs pcl_conversions)
ament_export_include_directories(include ${PCL_COMMON_INCLUDE_DIRS})

ament_package()
