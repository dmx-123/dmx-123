cmake_minimum_required(VERSION 2.8)
project(monocular_vo)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(mvo main.cpp)
target_link_libraries(mvo ${OpenCV_LIBS})
