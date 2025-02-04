cmake_minimum_required(VERSION 3.5)
project(lifecycle_pubsub)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Lifecycle talker
add_executable(lifecycle_talker
  src/lifecycle_talker.cpp
)

ament_target_dependencies(lifecycle_talker
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

install(
  TARGETS lifecycle_talker
  DESTINATION lib/${PROJECT_NAME}
)

# Lifecycle listener
add_executable(lifecycle_listener
  src/lifecycle_listener.cpp
)

ament_target_dependencies(lifecycle_listener
  rclcpp
  lifecycle_msgs
  std_msgs
)

install(
  TARGETS lifecycle_listener
  DESTINATION lib/${PROJECT_NAME}
)


# Lifecycle service client
add_executable(lifecycle_service_client
  src/lifecycle_service_client.cpp
)

ament_target_dependencies(lifecycle_service_client
  rclcpp
  lifecycle_msgs
)

install(
  TARGETS lifecycle_service_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
