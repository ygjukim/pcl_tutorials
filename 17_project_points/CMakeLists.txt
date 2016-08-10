cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(project_points)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
add_executable(project_points "src/project_points.cpp") 
target_link_libraries (project_points ${PCL_LIBRARIES})
