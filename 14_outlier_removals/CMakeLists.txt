cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(outlier_removals)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
add_executable(remove_outliers "src/remove_outliers.cpp") 
target_link_libraries (remove_outliers ${PCL_LIBRARIES})
