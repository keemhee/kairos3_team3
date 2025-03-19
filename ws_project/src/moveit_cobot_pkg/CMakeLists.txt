cmake_minimum_required(VERSION 3.8)
project(moveit_cobot_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(kairos_interfaces REQUIRED)

# Executable for test_node
add_executable(test_node src/test_node.cpp)

# Executable for pickplace_test
add_executable(pickplace_test_node src/pickplace_test.cpp)

#add_executable(pickplace_action_client src/pickplace_action_client.cpp)
add_executable(new_pickplace_action_server src/new_pickplace_action_server.cpp)

add_executable(new_pickplace_action_server_v2 src/new_pickplace_action_server_v2.cpp)

add_executable(pick_place_action_server_test src/pick_place_action_server_test.cpp)

add_executable(coord_joint_test src/coord_joint_test.cpp)

# Include directories for both executables
target_include_directories(test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(pickplace_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

include_directories(${vision_msgs_INCLUDE_DIRS})
include_directories(${moveit_visual_tools_INCLUDE_DIRS})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  /opt/ros/humble/include
)


# Compile features for both executables
target_compile_features(test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(pickplace_test_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(new_pickplace_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(new_pickplace_action_server_v2 PUBLIC c_std_99 cxx_std_17)
target_compile_features(pick_place_action_server_test PUBLIC c_std_99 cxx_std_17)

# Target dependencies for test_node
ament_target_dependencies(
  test_node
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
  visualization_msgs
  vision_msgs
)

# Target dependencies for pickplace_test
ament_target_dependencies(
  pickplace_test_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  vision_msgs
  sensor_msgs
  geometry_msgs
  moveit_visual_tools
  rviz_visual_tools
)

ament_target_dependencies(
  new_pickplace_action_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  kairos_interfaces
  srdfdom
)

ament_target_dependencies(
  new_pickplace_action_server_v2
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  kairos_interfaces
  srdfdom
)

ament_target_dependencies(
  pick_place_action_server_test
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  kairos_interfaces
  srdfdom
)
ament_target_dependencies(
  coord_joint_test
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  kairos_interfaces
  srdfdom
)


target_link_libraries(pickplace_test_node 
    moveit_visual_tools::moveit_visual_tools
    rviz_visual_tools::rviz_visual_tools
)
# Install the targets
install(TARGETS test_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS pickplace_test_node
  DESTINATION lib/${PROJECT_NAME})

#install(TARGETS pickplace_action_client
#  DESTINATION lib/${PROJECT_NAME}
#)
install(TARGETS new_pickplace_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS new_pickplace_action_server_v2
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS pick_place_action_server_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  coord_joint_test
  DESTINATION lib/${PROJECT_NAME}
)

# Test dependencies and linter setup (if needed)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter for copyright (remove comments if license is added)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
