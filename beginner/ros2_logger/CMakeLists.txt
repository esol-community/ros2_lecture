cmake_minimum_required(VERSION 3.5)
project(ros2_logger)

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

add_executable(logger_sample
  src/logger_sample.cpp
)

add_executable(logger_switch_level
  src/logger_switch_level.cpp
)

ament_target_dependencies(logger_sample
  rclcpp
)

ament_target_dependencies(logger_switch_level
  rclcpp
)

install(TARGETS
  logger_sample
  logger_switch_level
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
