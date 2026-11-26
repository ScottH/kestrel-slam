# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(kestrel_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(KESTREL_WARNINGS_AS_ERRORS "Treat compiler warnings as errors (CI)" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(kestrel_core REQUIRED)
find_package(kestrel_interfaces REQUIRED)

# The node is built as a component (loadable into a container for zero-copy intra-process image
# transport with the camera driver) and also as a standalone executable.
add_library(kestrel_node_component SHARED src/kestrel_node.cpp)
target_link_libraries(kestrel_node_component
  kestrel_core::kestrel_core
  rclcpp::rclcpp
  rclcpp_components::component
  ${kestrel_interfaces_TARGETS}
)
target_compile_options(kestrel_node_component PRIVATE -Wall -Wextra
  $<$<BOOL:${KESTREL_WARNINGS_AS_ERRORS}>:-Werror>)
rclcpp_components_register_node(kestrel_node_component
  PLUGIN "kestrel_ros::KestrelNode"
  EXECUTABLE kestrel_node
)

install(TARGETS kestrel_node_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
