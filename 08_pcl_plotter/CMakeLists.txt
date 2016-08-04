cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(pcl_plotter_demo`)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
add_executable(pcl_plotter_demo "src/pcl_plotter_demo.cpp") 
target_link_libraries (pcl_plotter_demo ${PCL_LIBRARIES})
