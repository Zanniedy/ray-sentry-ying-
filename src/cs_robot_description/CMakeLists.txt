cmake_minimum_required(VERSION 3.8)
project(rm_9_2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cs_interface REQUIRED)

# 安装资源目录
install(DIRECTORY config launch meshes textures urdf models world
  DESTINATION share/${PROJECT_NAME}
)

# controller_joint 节点
add_executable(controller_joint src/controller_joint.cpp)
ament_target_dependencies(controller_joint
  rclcpp
  std_msgs
  geometry_msgs
  cs_interface
)
install(TARGETS controller_joint
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
