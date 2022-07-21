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

message("behaviortree_cpp_v3: " ${behaviortree_cpp_v3_INCLUDE_DIRS})

if(behavior_tree_cpp_v3_FOUND)
    message([STATUS] " behavior_tree_cpp_v3 is available.")
endif()

add_executable(bt_test src/bt_test.cpp)
ament_target_dependencies(bt_test rclcpp behaviortree_cpp_v3)
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

install(
  TARGETS bt_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
