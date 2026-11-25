cmake_minimum_required(VERSION 3.5)
project(local_planner)

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

include_directories(include include/nlohmann)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
#find_package(unitree_api REQUIRED)
#find_package(go2_sport_api REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MpcRequest.msg" "msg/MpcResult.msg"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs nav_msgs)
ament_export_dependencies(rosidl_default_runtime)
add_executable(localPlanner src/localPlanner.cpp)
target_link_libraries(localPlanner Eigen3::Eigen)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(localPlanner "${cpp_typesupport_target}")
add_executable(planar_mpc_test test/planar_mpc_test.cpp)
target_link_libraries(planar_mpc_test Eigen3::Eigen)
enable_testing()
add_test(NAME planar_mpc COMMAND planar_mpc_test)
add_executable(pose_mpc_test test/pose_mpc_test.cpp)
target_link_libraries(pose_mpc_test Eigen3::Eigen)
add_test(NAME pose_mpc COMMAND pose_mpc_test)
add_executable(pathFollower src/pathFollower.cpp)
add_executable(astribotCoordinator src/astribotCoordinator.cpp)
target_link_libraries(astribotCoordinator Eigen3::Eigen yaml-cpp "${cpp_typesupport_target}")
ament_target_dependencies(astribotCoordinator rclcpp ament_index_cpp std_msgs sensor_msgs nav_msgs geometry_msgs)
add_executable(astribot_core_test test/astribot_core_test.cpp)
target_link_libraries(astribot_core_test Eigen3::Eigen yaml-cpp)
ament_target_dependencies(astribot_core_test rclcpp ament_index_cpp std_msgs sensor_msgs nav_msgs geometry_msgs)
add_test(NAME astribot_core COMMAND astribot_core_test)
add_executable(astribot_coordinator_test test/astribot_coordinator_test.cpp)
target_link_libraries(astribot_coordinator_test Eigen3::Eigen yaml-cpp "${cpp_typesupport_target}")
ament_target_dependencies(astribot_coordinator_test rclcpp ament_index_cpp std_msgs sensor_msgs nav_msgs geometry_msgs)
add_test(NAME astribot_coordinator COMMAND astribot_coordinator_test)
set_tests_properties(astribot_coordinator PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=228;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/astribot-native-tests" TIMEOUT 45)
add_executable(astribot_ros_smoke test/astribot_ros_smoke.cpp)
target_link_libraries(astribot_ros_smoke Eigen3::Eigen)
ament_target_dependencies(astribot_ros_smoke rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs)

# target_include_directories(pathFollower PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(localPlanner rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions)
ament_target_dependencies(pathFollower rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions)

# get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
# foreach(dir ${dirs})
#   message(WARNING "dir='${dir}'")
# endforeach()

install(TARGETS
  localPlanner
  pathFollower
  planar_mpc_test
  pose_mpc_test
  astribotCoordinator
  astribot_core_test
  astribot_coordinator_test
  astribot_ros_smoke
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/run_astribot.sh DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS python/local_planner_node.py DESTINATION lib/${PROJECT_NAME} RENAME astribotCoordinatorPython)
install(FILES python/local_planner_core.py python/navigation_feedback.py python/collision_feedback.py python/receipt_feedback.py python/planner_diagnostics.py python/auto_base_enable.py DESTINATION lib/${PROJECT_NAME})
install(FILES python/pose_mpc.py DESTINATION lib/${PROJECT_NAME})
add_test(NAME planner_python COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_CURRENT_SOURCE_DIR}/test)
set_tests_properties(planner_python PROPERTIES ENVIRONMENT "PYTHONNOUSERSITE=1;PYTHONDONTWRITEBYTECODE=1;OPENBLAS_NUM_THREADS=1;ROS_LOG_DIR=/tmp/local-planner-ctest")
install(PROGRAMS test/run_ros_smoke.sh DESTINATION share/${PROJECT_NAME}/test)

install(
  DIRECTORY
  launch
  paths
  config
  docs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
