cmake_minimum_required(VERSION 3.3)
project(pcl_recorder)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common io registration)
include_directories(${PCL_INCLUDE_DIRS})

ament_auto_add_executable(ndt_scan_matcher
  src/mainROS2.cpp
  src/PclRecorderROS2.cpp)

link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(ndt_scan_matcher ${PCL_LIBRARIES})

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
