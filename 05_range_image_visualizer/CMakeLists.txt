cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(cloud_viewer)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
add_executable(range_image_visualizer "src/range_image_visualizer.cpp") 
target_link_libraries (range_image_visualizer ${PCL_LIBRARIES})
