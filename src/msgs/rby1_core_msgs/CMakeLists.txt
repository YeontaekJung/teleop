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
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConnectRobot.srv"
  "srv/MoveToJointPosition.srv"
  "srv/SetControlMode.srv"
  "srv/SetPower.srv"
  "srv/SetServo.srv"
  "srv/SetNullspaceJointRef.srv"
  "srv/SetCartesianJointLimits.srv"
  "srv/SetNullspaceWeight.srv"
  "srv/SetStream.srv"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs sensor_msgs
)

ament_package()
