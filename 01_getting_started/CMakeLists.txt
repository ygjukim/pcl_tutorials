cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(getting_started)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
#file(GLOB PCL_openni_viewer_SRC
#    "src/*.h"
#    "src/main.cpp"
#)
#
#add_executable(openniViewer ${PCL_openni_viewer_SRC}) 
#target_link_libraries (openniViewer ${PCL_LIBRARIES})

add_executable(pcd_write_test "src/pcd_write.cpp") 
target_link_libraries (pcd_write_test ${PCL_LIBRARIES})