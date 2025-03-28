cmake_minimum_required(VERSION 3.5)
project(tf2_lecture)

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
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(tf2_broadcaster
  src/tf2_broadcaster.cpp
)

add_executable(tf2_listener
  src/tf2_listener.cpp
)

ament_target_dependencies(tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

ament_target_dependencies(tf2_listener
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

install(TARGETS
  tf2_broadcaster
  tf2_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)

endif()

ament_package()
