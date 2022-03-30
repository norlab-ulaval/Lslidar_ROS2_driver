cmake_minimum_required(VERSION 3.5)
project(lslidar_n301_decoder)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lslidar_n301_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)											   

add_executable(lslidar_n301_decoder_node src/lslidar_n301_decoder_node.cpp src/lslidar_n301_decoder.cpp)
target_link_libraries(lslidar_n301_decoder_node ${rclcpp_LIBRARIES})
ament_target_dependencies(lslidar_n301_decoder_node rclcpp std_msgs pluginlib lslidar_n301_msgs sensor_msgs pcl_conversions)

include_directories(
        include
        ${PCL_COMMON_INCLUDE_DIRS}
)

install(DIRECTORY launch params 
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  lslidar_n301_decoder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp std_msgs pluginlib lslidar_n301_msgs sensor_msgs pcl_conversions)
ament_export_include_directories(include ${PCL_COMMON_INCLUDE_DIRS})

ament_package()
