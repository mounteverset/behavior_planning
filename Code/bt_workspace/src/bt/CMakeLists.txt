cmake_minimum_required(VERSION 3.5)
project(bt)

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
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(bt_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp_action)
find_package(action_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
# find_package(rclcpp_components)

# add executables
add_executable(bt_test src/bt_test.cpp)
ament_target_dependencies(bt_test rclcpp behaviortree_cpp_v3)

add_executable(bt_rosbot src/bt_rosbot.cpp)
ament_target_dependencies(bt_rosbot 
  rclcpp 
  behaviortree_cpp_v3 
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  bt_msgs
  nav2_msgs
  rclcpp_action
  visualization_msgs
  rcl_interfaces
  lifecycle_msgs)

add_executable(execution_checker_service src/execution_checker_service.cpp)
ament_target_dependencies(execution_checker_service rclcpp std_srvs sensor_msgs nav_msgs gazebo_msgs tf2 bt_msgs action_msgs)


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

# target_include_directories(bt_lidar PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# target_compile_definitions(bt_lidar
#   PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")

include_directories(
  include
  ) 

# rclcpp_components_register_node(action_client PLUGIN "CancelNavGoal" EXECUTABLE fibonacci_action_client)

install(
  TARGETS bt_test bt_rosbot execution_checker_service
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
