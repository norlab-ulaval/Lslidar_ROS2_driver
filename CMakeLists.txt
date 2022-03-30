cmake_minimum_required(VERSION 3.5)
project(lsm10_v2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/include/lsm10_v2)
include_directories(include)

add_executable(lsm10_node src/lsm10.cpp src/lsiosr.cpp )

ament_target_dependencies(lsm10_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Difop.msg"
)

install(TARGETS lsm10_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params 
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
