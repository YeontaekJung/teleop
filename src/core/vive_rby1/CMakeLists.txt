cmake_minimum_required(VERSION 3.10)
project(vive_rby1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(rby1_core_msgs REQUIRED)
find_package(scm_recording_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_executable(vive_rby1_node
  src/vive_rby1_node.cpp
)

target_compile_features(vive_rby1_node PUBLIC cxx_std_17)
target_link_libraries(vive_rby1_node Eigen3::Eigen)

if(TARGET pinocchio::pinocchio)
  target_link_libraries(vive_rby1_node pinocchio::pinocchio)
else()
  target_include_directories(vive_rby1_node PRIVATE ${pinocchio_INCLUDE_DIRS})
  target_link_libraries(vive_rby1_node ${pinocchio_LIBRARIES})
endif()

ament_target_dependencies(vive_rby1_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  interbotix_xs_msgs
  rby1_core_msgs
  scm_recording_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS vive_rby1_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/vive_rby1_debug_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
