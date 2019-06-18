cmake_minimum_required(VERSION 2.8)
project(img_findPoint)

set( CMAKE_CXX_FLAGS "-std=c++11" )
set( CMAKE_BUILD_TYPE "Debug" )

find_package( OpenCV  REQUIRED )
find_package( PCL REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions( ${PCL_DEFINITIONS} )

add_executable( img_findPoint main.cpp findPoint.cpp kmeans.cpp point.cpp get_Rt.cpp Tsai.cpp)
target_link_libraries( img_findPoint ${OpenCV_LIBS} ${PCL_LIBRARIES})
