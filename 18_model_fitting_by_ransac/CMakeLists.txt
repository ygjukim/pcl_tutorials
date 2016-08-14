cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(model_fitting_by_ransac)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
add_executable(model_fitting_by_ransac "src/model_fitting_by_ransac.cpp") 
target_link_libraries (model_fitting_by_ransac ${PCL_LIBRARIES})
