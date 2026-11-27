# SPDX-License-Identifier: Apache-2.0
#
# kestrel_core — the ROS-independent SLAM library.
#
# Builds two ways:
#   * inside a colcon workspace (ament_cmake found)  → installs as a ROS 2 package
#   * standalone:  cmake -S kestrel_core -B build && cmake --build build && ctest --test-dir build
cmake_minimum_required(VERSION 3.16)
project(kestrel_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # for clangd / VS Code IntelliSense
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)  # SLAM is unusably slow in Debug
endif()

option(KESTREL_BUILD_TESTS "Build unit tests" ON)
option(KESTREL_WARNINGS_AS_ERRORS "Treat compiler warnings as errors (CI)" OFF)

# ------------------------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------------------------
find_package(ament_cmake QUIET)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs videoio features2d video calib3d)
find_package(GTSAM 4.2 REQUIRED)  # poses, IMU preintegration, factor graphs, iSAM2
# Fixed-lag smoothers are in gtsam_unstable up to GTSAM 4.2 (core library from 4.3 on).
find_package(GTSAM_UNSTABLE QUIET)
if(TARGET gtsam_unstable)
  set(KESTREL_GTSAM_UNSTABLE gtsam_unstable)
endif()
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(KESTREL_YAML_TARGET yaml-cpp::yaml-cpp)
else()
  set(KESTREL_YAML_TARGET yaml-cpp)
endif()
find_package(Threads REQUIRED)

configure_file(include/kestrel/version.hpp.in
               ${CMAKE_CURRENT_BINARY_DIR}/generated/kestrel/version.hpp @ONLY)

# ------------------------------------------------------------------------------------------------
# Library
# ------------------------------------------------------------------------------------------------
add_library(kestrel_core
  src/logging.cpp
  src/config.cpp
  src/euroc.cpp
  src/feature_tracker.cpp
  src/imu.cpp
  src/odometry.cpp
  src/stereo_rig.cpp
  src/vio_estimator.cpp
)
target_include_directories(kestrel_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kestrel_core PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBS}
  gtsam
  ${KESTREL_GTSAM_UNSTABLE}
  ${KESTREL_YAML_TARGET}
  Threads::Threads
)
target_include_directories(kestrel_core SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
# -Wno-error=maybe-uninitialized: GCC 13 reports false positives inside Eigen's SVD when it is
# instantiated through GTSAM headers (a known GCC/Eigen issue). It stays a warning, just not fatal.
target_compile_options(kestrel_core PRIVATE -Wall -Wextra -Wpedantic
  $<$<BOOL:${KESTREL_WARNINGS_AS_ERRORS}>:-Werror -Wno-error=maybe-uninitialized>)
set_target_properties(kestrel_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ------------------------------------------------------------------------------------------------
# Command-line tools
# ------------------------------------------------------------------------------------------------
add_executable(kestrel_info apps/kestrel_info.cpp)
target_link_libraries(kestrel_info PRIVATE kestrel_core)
add_executable(run_frontend apps/run_frontend.cpp)
add_executable(run_vio apps/run_vio.cpp)
target_link_libraries(run_vio PRIVATE kestrel_core)

# Optional: Foxglove visualization (live WebSocket + MCAP). Built when the Foxglove SDK is found
# (the dev image ships it in /opt/foxglove-sdk); kestrel_core itself never depends on it.
find_package(foxglove-sdk CONFIG QUIET HINTS $ENV{FOXGLOVE_SDK_ROOT} /opt/foxglove-sdk)
if(foxglove-sdk_FOUND)
  foxglove_sdk_add_cpp_library(kestrel_foxglove_sdk TYPE STATIC REMOTE_ACCESS OFF)
  set_target_properties(kestrel_foxglove_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
  add_library(kestrel_viz src/foxglove_visualizer.cpp)
  target_link_libraries(kestrel_viz PUBLIC kestrel_core PRIVATE kestrel_foxglove_sdk)
  target_compile_options(kestrel_viz PRIVATE -Wall -Wextra)
  set_target_properties(kestrel_viz PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(run_vio PRIVATE kestrel_viz)
  target_compile_definitions(run_vio PRIVATE KESTREL_WITH_FOXGLOVE)
  message(STATUS "Foxglove SDK found: building kestrel_viz")
else()
  message(STATUS "Foxglove SDK not found: visualization disabled")
endif()
target_link_libraries(run_frontend PRIVATE kestrel_core)

# ------------------------------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------------------------------
if(KESTREL_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  include(GoogleTest)
  set(KESTREL_TESTS
    test_config
    test_euroc
    test_feature_tracker
    test_gtsam_smoke
    test_imu
    test_stereo_rig
    test_vio_estimator
  )
  foreach(test_name IN LISTS KESTREL_TESTS)
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE kestrel_core GTest::gtest_main)
    gtest_discover_tests(${test_name} DISCOVERY_TIMEOUT 30
      PROPERTIES ENVIRONMENT "KESTREL_REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/..")
  endforeach()
endif()

# ------------------------------------------------------------------------------------------------
# Install / export (so kestrel_ros can `find_package(kestrel_core)`)
# ------------------------------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS kestrel_core EXPORT kestrel_coreTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS kestrel_info run_frontend run_vio RUNTIME DESTINATION lib/${PROJECT_NAME})

if(ament_cmake_FOUND)
  ament_export_targets(kestrel_coreTargets HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3 OpenCV GTSAM yaml-cpp)
  if(KESTREL_GTSAM_UNSTABLE)
    ament_export_dependencies(GTSAM_UNSTABLE)
  endif()
  ament_package()
else()
  install(EXPORT kestrel_coreTargets NAMESPACE kestrel_core:: DESTINATION lib/cmake/kestrel_core)
endif()
