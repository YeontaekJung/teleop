cmake_minimum_required(VERSION 3.8)
project(rby1_core_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DrivingFollowWaypoint.action"
  "action/Rby1Command.action"
  "srv/GetJointMotionPlan.srv"
  "srv/GetRTAJointMotionPlan.srv"
  "srv/GetRTATaskMotionPlan.srv"
  "srv/GetTaskMotionPlan.srv"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs sensor_msgs moveit_msgs
)

ament_package()
