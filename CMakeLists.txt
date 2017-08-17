cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O3 -std=c++11")


find_package(OpenCV 3.3 REQUIRED)
include_directories(${INCLUDE_OPENCV_DIR})

find_package(Glog REQUIRED)
include_directories(${INCLUDE_GLOG_DIRS})

find_package(GFlags REQUIRED)
include_directories(${INCLUDE_GFLAGS_DIRS})


add_executable(simple_play_image simple_play_image.cpp)
target_link_libraries(simple_play_image glog gflags ${OpenCV_LIBS})
