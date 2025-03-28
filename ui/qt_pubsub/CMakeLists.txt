cmake_minimum_required(VERSION 3.5)
project(qt_pubsub)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# packages for Qt
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)

add_executable(qt_talker src/qt_talker.cpp src/qt_talker_GUI.cpp src/qt_talker_rosNode.cpp)
add_executable(qt_listener src/qt_listener.cpp src/qt_listener_GUI.cpp src/qt_listener_rosNode.cpp)

ament_target_dependencies(qt_talker rclcpp Qt5Core Qt5Widgets rclcpp std_msgs)
ament_target_dependencies(qt_listener rclcpp Qt5Core Qt5Widgets rclcpp std_msgs)

install(
  TARGETS
  qt_talker
  qt_listener
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
