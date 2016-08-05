cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(openni_grabber)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
add_executable(openni_grabber "src/openni_grabber.cpp") 
target_link_libraries (openni_grabber ${PCL_LIBRARIES})
