cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(cluster_extraction)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
add_executable(extract_clusters "src/extract_clusters.cpp") 
target_link_libraries (extract_clusters ${PCL_LIBRARIES})
