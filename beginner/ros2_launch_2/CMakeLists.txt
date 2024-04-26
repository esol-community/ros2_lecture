cmake_minimum_required(VERSION 3.5)
project(ros2_launch_2)

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
find_package(pub_sub_comm REQUIRED)
find_package(ros2_launch_1 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# create executable talker
add_executable(param_talker
  src/param_talker.cpp
)

# link library to talker
ament_target_dependencies(param_talker
  rclcpp
  std_msgs
)

# install configuration
install(TARGETS
  param_talker
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
