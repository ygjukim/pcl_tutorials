cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(cloud_viewer)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
add_executable(cloud_viewer "src/cloud_viewer.cpp") 
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})
