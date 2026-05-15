cmake_minimum_required(VERSION 3.8)
project(scm_recording_msgs)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartRecording.srv"
  "srv/EndRecording.srv"
  "srv/GetStatus.srv"
  "srv/TogglePause.srv"
  "srv/SetTeleOpPose.srv"
  DEPENDENCIES sensor_msgs
)

ament_package()
