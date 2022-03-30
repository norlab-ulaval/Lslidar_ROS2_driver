cmake_minimum_required(VERSION 3.5)
project(ls01b_v2)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include) 
#set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)
#find_package(Boost REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/include/ls01b_v2)
include_directories(include)

add_executable(ls01b_node src/ls01b.cpp src/lsiosr.cpp )

#add_library(ls01b_node SHARED src/ls01b.cpp src/lsiosr.cpp)			
ament_target_dependencies(ls01b_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
)


rosidl_generate_interfaces(AddThreeInts
  "srv/AddThreeInts.srv"
 )
 
#target_link_libraries(ls01b_node ${Boost_LIBRARIES})

install(TARGETS ls01b_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params 
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
