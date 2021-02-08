cmake_minimum_required(VERSION 3.4...3.18)
project(listener LANGUAGES CXX)

find_package(gazebo REQUIRED)  

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}") 

# include appropriate directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})  

add_subdirectory(extern/pybind11)

add_executable(listener_node listener_node.cpp)
target_link_libraries(listener_node PRIVATE pybind11::embed ${GAZEBO_LIBRARIES} listener)

add_library(listener
    listener.cpp
    listener.h
    )  
target_link_libraries(listener PRIVATE pybind11::embed ${GAZEBO_LIBRARIES} pthread) 
