cmake_minimum_required(VERSION 3.5)
project(component_pubsub)

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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)


# talker_component
add_library(talker_component SHARED
  src/talker_component.cpp
)

ament_target_dependencies(talker_component
  rclcpp
  rclcpp_components
  std_msgs
)

rclcpp_components_register_node(talker_component
  PLUGIN "component_pubsub::SimpleTalker"
  EXECUTABLE talker_component_node
)

# listener component
add_library(listener_component SHARED
  src/listener_component.cpp
)

ament_target_dependencies(listener_component
  rclcpp
  rclcpp_components
  std_msgs
)

rclcpp_components_register_node(listener_component
  PLUGIN "component_pubsub::SimpleListener"
  EXECUTABLE listener_component_node
)

# install settings
install(
  TARGETS talker_component listener_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
