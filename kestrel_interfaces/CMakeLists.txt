# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(kestrel_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
  "srv/LoadMap.srv"
  "srv/ResetMap.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
