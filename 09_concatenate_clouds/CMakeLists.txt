cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(pcl_plotter_demo`)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
add_executable(concatenate_clouds "src/concatenate_clouds.cpp") 
target_link_libraries (concatenate_clouds ${PCL_LIBRARIES})