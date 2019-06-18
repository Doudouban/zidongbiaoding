cmake_minimum_required(VERSION 2.8)
project(grubImg)
# 添加c++ 11标准支持
#set( CMAKE_CXX_FLAGS "-std=c++11" )
# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )
#set( CMAKE_BUILD_TYPE "Release" )
# 变量的定义
set(TARGET_LIB tycam)
set(INCLUDE_PATH ./include)
#option(ENABLE_PCL "Enable PCL" OFF)
set(COMMON_SOURCES common/MatViewer.cpp common/PointCloudViewer.cpp) #common, pcl is too slow, so build a lib to speed up
#添加共享库
add_library(common_shared SHARED ${COMMON_SOURCES})
#添加common里面的头文件
include_directories(${INCLUDE_PATH})
# ========================================
# === OpenCV
# ========================================
find_package( OpenCV REQUIRED ) #the path of OpenCVConfig.camke:/usr/local/share/OpenCV
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV library not found")
else()
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()
# ========================================
# === PCL
# ========================================
find_package( PCL REQUIRED )
if (NOT PCL_FOUND)
      message(WARNING "PCL library not found, some samples will not be compiled")
else()
      include_directories( ${PCL_INCLUDE_DIRS} )
#向工程添加多个特定的头文件搜索路径,路径之间用空格分隔
      add_definitions( ${PCL_DEFINITIONS} )
#add_definitions 为源文件的编译添加由-D引入的define flag
endif()

#添加可执行文件
add_executable(grubImg main.cpp)
#链接库,为target添加需要链接的共享库
target_link_libraries(grubImg common_shared ${TARGET_LIB} ${OpenCV_LIBS} ${PCL_LIBRARIES})

