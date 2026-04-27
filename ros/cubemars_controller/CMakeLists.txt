cmake_minimum_required(VERSION 3.8)
project(cubemars_controller)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cubemars_msgs REQUIRED)

add_executable(cubemars_controller_node
  src/cubemars_controller_node.cpp
  src/ak_mit_adapter.c
)

target_include_directories(cubemars_controller_node PRIVATE
  src/
)

ament_target_dependencies(cubemars_controller_node
  rclcpp
  cubemars_msgs
)

install(TARGETS cubemars_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
