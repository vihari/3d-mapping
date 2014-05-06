cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(OS object_segmentation)
set(OT object_track)
set(OD odometry)
set(RECON reconstruction)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)
project( BTP )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( OS ${OS}/farneback.cpp )
target_link_libraries( OS ${OpenCV_LIBS} )

add_executable( OT ${OT}/lk_based.cpp )
target_link_libraries( OT ${OpenCV_LIBS} )

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (test ${RECON}/VisualHull.cpp)
target_link_libraries (test ${PCL_LIBRARIES})
target_link_libraries( test ${OpenCV_LIBS} )
