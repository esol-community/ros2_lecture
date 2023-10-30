cmake_minimum_required(VERSION 3.5)
project(pub_sub_comm)

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
find_package(std_msgs REQUIRED)

# create executable talker
add_executable(talker
  src/simple_talker.cpp
)

# link library to talker
ament_target_dependencies(talker
  rclcpp
  std_msgs
)

# create executable listener
add_executable(listener
  src/simple_listener.cpp
)

# link library to listener
ament_target_dependencies(listener
  rclcpp
  std_msgs
)

# install configuration
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
