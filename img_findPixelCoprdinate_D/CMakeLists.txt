cmake_minimum_required(VERSION 2.8)
project(img_findPixelCoprdinate)

set( CMAKE_CXX_FLAGS "-std=c++11" )
set( CMAKE_BUILD_TYPE "Debug" )

find_package( OpenCV  REQUIRED ) 

include_directories( ${OpenCV_INCLUDE_DIRS}) 

add_executable( img_findPixelCoprdinate main.cpp )
target_link_libraries( img_findPixelCoprdinate ${OpenCV_LIBS})
