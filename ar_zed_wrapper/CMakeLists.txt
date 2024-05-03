cmake_minimum_required(VERSION 3.5)
project(ar_zed_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ##############################################################################
# Generate symbols for IDE indexer (VSCode)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# ##############################################################################

# Default to C99
if(NOT CMAKE_C_STANDARiD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE MATCHES Release)
  # message(" * Release Mode")
  add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  # message(" * Release with Debug Info Mode")
  add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
  message(" * Debug Mode")
endif()

exec_program(
  uname ARGS
  -p
  OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if(CMAKE_SYSTEM_NAME2 MATCHES "aarch64") # Jetson TX
  set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
endif()

# ##############################################################################
# Dependencies
find_package(ZED 3 REQUIRED)
find_package(CUDA REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(zed_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(std_msgs REQUIRED)
find_package(zed_interfaces REQUIRED)
find_package(ar_zed_msgs REQUIRED)

# INCLUDES and LIBS
include_directories(${CUDA_INCLUDE_DIRS} ${ZED_INCLUDE_DIRS})

include_directories(include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

set(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_LIBRARIES})

# Zed streamer
add_library(zed_streamer_node SHARED src/zed_streamer.cpp)
target_include_directories(
  zed_streamer_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
target_compile_features(zed_streamer_node PUBLIC c_std_99 cxx_std_17)
target_link_libraries(zed_streamer_node ${ZED_LIBS})

ament_target_dependencies(
  zed_streamer_node
  rclcpp
  rclcpp_components
  zed_components
  zed_interfaces
  std_msgs
  ar_zed_msgs
  ZED
  CUDA)

rclcpp_components_register_node(zed_streamer_node PLUGIN "ZedStreamer"
                                EXECUTABLE zed_streamer_node_exe)

install(
  TARGETS zed_streamer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
